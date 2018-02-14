#include "Fitting.h"

#include "util/mylogger.h"
#include "util/tictoc.h"

#include "tracker/Worker.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/HModel/JacobianVerifier.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HModel/Model.h"
#include "tracker/OpenGL/OffscreenRenderer.h"
#include "tracker/OpenGL/CustomFrameBuffer.h" 
#include "tracker/TwSettings.h"

#ifdef WITH_CUDA
#include "cudax/CudaHelper.h"
#include "cudax/CublasHelper.h"

#include <cuda_gl_interop.h>
struct MappedResource {
	struct cudaGraphicsResource* resouce = NULL;
	cudaArray* array = NULL;
	GLuint texid = 0;

	void init(GLuint texid) {
		this->texid = texid;
		checkCudaErrors(cudaGraphicsGLRegisterImage(&resouce, texid, GL_TEXTURE_2D, cudaGraphicsMapFlagsReadOnly));
	}
	void cleanup() {
		checkCudaErrors(cudaGraphicsUnregisterResource(resouce));
	}

	cudaArray* bind() {
		checkCudaErrors(cudaGraphicsMapResources(1, &resouce, 0));
		checkCudaErrors(cudaGraphicsSubResourceGetMappedArray(&array, resouce, 0, 0));
		return array;
	}
	void unbind() {
		checkCudaErrors(cudaGraphicsUnmapResources(1, &resouce, 0));
	}
};

MappedResource sensor_depth_mapped_resource;
MappedResource rendered_block_indices_mapped_resource;
MappedResource rendered_depth_mapped_resource;
//DistanceTransform distance_transform;

void energy::Fitting::cleanup() {
	kernel_cleanup(); ///< disposes of static resources
	sensor_depth_mapped_resource.cleanup();
	if (settings->fit2D_silhouette_enable) {
		rendered_block_indices_mapped_resource.cleanup();
		rendered_depth_mapped_resource.cleanup();
	}
	distance_transform.cleanup();

	cudax::CublasHelper::cleanup();
	cudax::CudaHelper::cleanup();
}

void energy::Fitting::process_model_silhouette() {
	static cv::Mat rendered_silhouette_mat;
	if (rendered_silhouette_mat.empty())
		rendered_silhouette_mat = cv::Mat(camera->height(), camera->width(), CV_8UC1, cv::Scalar(0));

	glBindTexture(GL_TEXTURE_2D, offscreen_renderer->frame_buffer->silhouette_texture_id());
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, rendered_silhouette_mat.data);
	glBindTexture(GL_TEXTURE_2D, 0);

	cv::flip(rendered_silhouette_mat, rendered_silhouette_mat, 0);
	//cv::imshow("rendered_silhouette", rendered_silhouette_mat); cv::waitKey(1);
	num_rendered_points = 0;
	bool to_break = false;
	for (int row = 0; row < rendered_silhouette_mat.rows; ++row) {
		for (int col = 0; col < rendered_silhouette_mat.cols; ++col) {
			if (rendered_silhouette_mat.at<uchar>(row, col) != 255 && handfinder->sensor_silhouette.at<uchar>(row, col) != 255) {
				if (num_rendered_points >= upper_bound_num_rendered_silhouette_points) {
					cout << "num_rendered_points is bigger than expected" << endl;
					to_break = true;
					break;
				}
				rendered_indicator[num_rendered_points] = row * camera->width() + col;
				num_rendered_points++;
			}
		}
		if (to_break) break;
	}

	bool display = false;
	if (display) {
		cv::Mat rendered_indicator_image;
		rendered_indicator_image = cv::Mat(camera->height(), camera->width(), CV_8UC1, cv::Scalar(0));
		for (size_t i = 0; i < num_rendered_points; i++) {
			int row = rendered_indicator[i] / camera->width();
			int col = rendered_indicator[i] - camera->width() * row;
			rendered_indicator_image.at<uchar>(row, col) = 255;
		}
		cv::imshow("rendered_indicator_image", rendered_indicator_image);
	}
}

void energy::Fitting::init(Worker *worker) {
	if (worker->E_fitting.settings->fit2D_silhouette_enable) {
		rendered_indicator = new int[upper_bound_num_rendered_silhouette_points];
	}

	this->camera = worker->camera;
	this->sensor_depth_texture = worker->sensor_depth_texture;
	this->handfinder = worker->handfinder;
	this->model = worker->model;
	this->offscreen_renderer = &(worker->offscreen_renderer);

	this->jacobian_verifier = worker->jacobian_verifier;

	cudax::CudaHelper::init();
	cudax::CublasHelper::init();

	///--- Run some tests before we get started
	kernel_memory_tests();

	///--- Init worker for GPU computation of normals
	distance_transform.init(camera->width(), camera->height());

	///--- init resource mapper for cuda
	sensor_depth_mapped_resource.init(sensor_depth_texture->texture_id());
	if (worker->E_fitting.settings->fit2D_silhouette_enable) {
		rendered_block_indices_mapped_resource.init(offscreen_renderer->frame_buffer->silhouette_texture_id());
		rendered_depth_mapped_resource.init(offscreen_renderer->frame_buffer->depth_texture_id());
	}

	kernel_init(this->settings, camera->width(), camera->height(), model->num_parameters, camera->focal_length_x(), camera->focal_length_y(), camera->inv_projection_matrix().data(),
		d, model->centers.size(), model->blocks.size(), model->max_num_outlines, model->num_tangent_fields, model->num_outline_fields, 
		worker->settings->multiply_on_cpu, worker->settings->calibrate, worker->settings->verify_jacobian, worker->settings->compute_weighted_metrics);
	CHECK_ERROR_GL();
}

void energy::Fitting::track(DataFrame& frame, LinearSystem& system, bool rigid_only, bool eval_error, bool calibrate, float & push_error, float & pull_error, float & weighted_error, int iter) {

	/// > Make sure sensor has necessary data
	assert(sensor_depth_texture->check_loaded(frame.id));
	static int last_uploaded_id = -1;

	/// > Upload hand model
	kernel_upload_model(d, model->num_parameters, model->centers.size(), model->blocks.size(), model->outline_finder.outline3D.size(), model->num_tangent_fields, model->num_outline_fields,
		model->host_pointer_centers, model->host_pointer_radii, model->host_pointer_blocks,
		model->host_pointer_tangent_points, model->host_pointer_outline, model->host_pointer_blockid_to_pose_unit_id_map, model->host_pointer_blockid_to_shape_unit_id_map, calibrate);

	kernel_upload_kinematic(model->theta_infos, model->host_pointer_kinematic_chains, model->beta_infos, model->host_pointer_shape_chains);

	if (last_uploaded_id != frame.id) kernel_upload_data_energy_data(handfinder->sensor_indicator, handfinder->num_sensor_points);

	/// > Compute distance transform		
	if ((last_uploaded_id != frame.id) && (settings->fit2D_silhouette2outline_enable || settings->fit2D_silhouette_enable || settings->fit2D_outline_enable)) {
		static cv::Mat sensor_silhouette_flipped;
		if (settings->fit2D_silhouette2outline_enable || settings->fit2D_silhouette_enable) {
			cv::flip(handfinder->sensor_silhouette, sensor_silhouette_flipped, 0);
		}
		if (settings->fit2D_outline_enable) {
			cv::flip(handfinder->sensor_silhouette_closed, sensor_silhouette_flipped, 0);
		}
		distance_transform.exec(sensor_silhouette_flipped.data, 125);
		kernel_upload_sensor_silhouette_distance_transform(distance_transform.idxs_image_ptr());

		if (settings->fit_wrist_separately) {
			cv::flip(handfinder->sensor_silhouette_wrist, sensor_silhouette_flipped, 0);
			distance_transform.exec(sensor_silhouette_flipped.data, 125);
			kernel_upload_sensor_silhouette_wrist_distance_transform(distance_transform.idxs_image_ptr());
		}
	}	

	/// > Compute model outline
	if (settings->fit2D_silhouette2outline_enable) {
		model->compute_outline_energy_data(handfinder->sensor_silhouette, camera, settings);
		kernel_upload_outline_energy_data(model->rendered_pixels, model->rendered_points, model->rendered_normals, model->rendered_block_ids, model->num_rendered_points);
		num_rendered_points = model->num_rendered_points;
		for (size_t i = 0; i < num_rendered_points; i++) {
			if (abs(model->rendered_points[3 * i + 2]) < 1e-6) 	cout << "z of an outline point = 0, projection jacobian will fail" << endl;
		}
	}

	/// > Process model silhouette
	if (settings->fit2D_silhouette_enable) {
		process_model_silhouette();
		kernel_upload_silhouette_energy_data(rendered_indicator, num_rendered_points);
		offscreen_renderer->frame_buffer->bind();
		cudax::rendered_block_indices_array = rendered_block_indices_mapped_resource.bind();
		cudax::rendered_depth_array = rendered_depth_mapped_resource.bind();
	}

	/// > Process data outline
	if ((last_uploaded_id != frame.id) && settings->fit2D_outline_enable) {
		static cv::Mat sensor_outline_flipped;
		cv::flip(handfinder->sensor_outline, sensor_outline_flipped, 0);
		distance_transform.exec(sensor_outline_flipped.data, 125);
		
		kernel_upload_sensor_outline_distance_transform(distance_transform.idxs_image_ptr());
	}

	if(settings->fit2D_outline_enable) {
		model->compute_outline_energy_data(handfinder->sensor_silhouette_closed, camera, settings);		
		num_rendered_points = model->num_rendered_points;		
		kernel_upload_outline_energy_data(model->rendered_pixels, model->rendered_points, model->rendered_normals, model->rendered_block_ids, model->num_rendered_points);
	}
	last_uploaded_id = frame.id;

	cudax::sensor_depth_array = sensor_depth_mapped_resource.bind();
	kernel_bind();

	bool reweight = settings->fit3D_reweight;
	if (rigid_only && settings->fit3D_reweight && !(settings->fit3D_reweight_rigid))
		reweight = false; // allows fast rigid motion

	kernel(system.lhs.data(), system.rhs.data(), push_error, pull_error, weighted_error, eval_error,
		reweight, frame.id, iter, handfinder->num_sensor_points, num_rendered_points,
		jacobian_verifier->correspondences_data_points, jacobian_verifier->correspondences_model_points, jacobian_verifier->correspondences_model_indices, jacobian_verifier->correspondences_block_indices, jacobian_verifier->jacobian_data,
		jacobian_verifier->correspondences_silhouette_data_points, jacobian_verifier->correspondences_silhouette_model_points, jacobian_verifier->correspondences_silhouette_indices, jacobian_verifier->jacobian_silhouette);

	kernel_unbind();
	sensor_depth_mapped_resource.unbind();
	if (settings->fit2D_silhouette_enable) {
		rendered_block_indices_mapped_resource.unbind();
		rendered_depth_mapped_resource.unbind();
		offscreen_renderer->frame_buffer->unbind();
	}

	// Check for nans
	if (system.has_nan()) cout << "of the Fitting term" << endl;
}

energy::Fitting::~Fitting() {
	kernel_delete();
}

#endif
