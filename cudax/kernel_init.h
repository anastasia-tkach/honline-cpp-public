#pragma once
#include "kernel.h"
#include "cudax/Kinematic.h"

//=============================================================================
/// Constructor
//=============================================================================
void kernel_init(energy::fitting::Settings* _settings, int H_width, int H_height, int num_parameters, float H_focal_length_x, float H_focal_length_y, const float* H_inv_proj_matrix,
	int d, int num_centers, int num_blocks, int max_num_outlines, int num_tangent_fields, int num_outline_fields, bool multiply_on_cpu, bool calibrate, bool verify_jacobian, bool compute_weighted_metric) {
	cudax::settings = _settings;

	using namespace cudax;
	bool is_init = false;
	if (is_init) { std::cout << "!!!ERROR initialized cuda kernel twice" << std::endl; exit(0); }
	is_init = true;

	t = new cudax::CudaTimer();
	t->restart("Kernel Initialization");	

	///--- allocate linear system
	J = new thrust::device_vector<float>();
	F = new thrust::device_vector<float>();

	//J->reserve(upper_bound_num_constraints);
	//e->reserve(upper_bound_num_constraints);
	int upper_bound_num_sensor_points = 76800;
	J->resize(upper_bound_num_sensor_points * num_parameters);
	F->resize(upper_bound_num_sensor_points);

	pull_indices = new thrust::device_vector<int>(upper_bound_num_sensor_points);
	_sensor_indicator = new thrust::device_vector<int>(upper_bound_num_sensor_points);

	int upper_bound_num_outlines = 5000;

	//if (settings->fit2D_outline_enable || settings->fit2D_silhouette2outline_enable || settings->fit2D_silhouette_enable) {
		push_indices = new thrust::device_vector<int>(upper_bound_num_outlines);
		_rendered_pixels = new thrust::device_vector<int>(upper_bound_num_outlines);
		_rendered_points = new thrust::device_vector<float>(3 * upper_bound_num_outlines);
		_rendered_normals = new thrust::device_vector<float>(2 * upper_bound_num_outlines);
		_rendered_block_ids = new thrust::device_vector<int>(upper_bound_num_outlines);
	//}

	if (verify_jacobian) {
		_correspondences_data_points = new thrust::device_vector<float>(upper_bound_num_sensor_points, 0.0f);
		_correspondences_model_points = new thrust::device_vector<float>(upper_bound_num_sensor_points, 0.0f);
		_correspondences_model_indices = new thrust::device_vector<int>(upper_bound_num_sensor_points, 0.0f);
		_correspondences_block_indices = new thrust::device_vector<int>(upper_bound_num_sensor_points / 3, 0.0f);
		_jacobian_data = new thrust::device_vector<float>(upper_bound_num_sensor_points * num_parameters, 0.0f);

		_correspondences_silhouette_data_points = new thrust::device_vector<float>(3 * upper_bound_num_outlines, 0.0f);
		_correspondences_silhouette_model_points = new thrust::device_vector<float>(3 * upper_bound_num_outlines, 0.0f);
		_correspondences_silhouette_indices = new thrust::device_vector<int>(3 * upper_bound_num_outlines, 0.0f);
		_jacobian_silhouette = new thrust::device_vector<float>(3 * upper_bound_num_outlines * num_parameters, 0.0f);
	}

	if (compute_weighted_metric) {
		_weighted_metric_sensor_palm = new thrust::device_vector<float>(upper_bound_num_sensor_points / 3);
		_weighted_metric_sensor_fingers = new thrust::device_vector<float>(upper_bound_num_sensor_points / 3);
		_weighted_metric_rendered_palm = new thrust::device_vector<float>(upper_bound_num_outlines / 3);
		_weighted_metric_rendered_fingers = new thrust::device_vector<float>(upper_bound_num_outlines / 3);
	}

	JtJ = new thrust::device_vector<float>(num_parameters * num_parameters);
	JtF = new thrust::device_vector<float>(num_parameters);

	///--- copy  floats to GPU
	cudax::H_width = H_width;
	cudax::H_height = H_height;
	cudaMemcpyToSymbol(focal_length_x, &H_focal_length_x, sizeof(float));
	cudaMemcpyToSymbol(focal_length_y, &H_focal_length_y, sizeof(float));
	cudaMemcpyToSymbol(width, &H_width, sizeof(int));
	cudaMemcpyToSymbol(height, &H_height, sizeof(int));

	///--- copy camera matrix
	camera_matrix = new KinectCamera(H_inv_proj_matrix);

	_sensor_silhouette_distance_transform = new thrust::device_vector<int>(H_width * H_height, -1);
	_sensor_outline_distance_transform = new thrust::device_vector<int>(H_width * H_height, -1);
	if (settings->fit_wrist_separately)
		_sensor_silhouette_wrist_distance_transform = new thrust::device_vector<int>(H_width * H_height, -1);

	///-- allocate memory for Hmodel data
	_num_parameters = num_parameters;
	cudaMemcpyToSymbol(D, &d, sizeof(int));
	cudaMemcpyToSymbol(NUM_PARAMETERS, &num_parameters, sizeof(int));
	cudaMemcpyToSymbol(NUM_CENTERS, &num_centers, sizeof(int));
	cudaMemcpyToSymbol(NUM_BLOCKS, &num_blocks, sizeof(int));
	cudaMemcpyToSymbol(NUM_TANGENT_FIELDS, &num_tangent_fields, sizeof(int));
	cudaMemcpyToSymbol(NUM_OUTLINE_FIELDS, &num_outline_fields, sizeof(int));

	device_pointer_centers = new thrust::device_vector<float>(d * num_centers);
	device_pointer_radii = new thrust::device_vector<float>(num_centers);
	device_pointer_blocks = new thrust::device_vector<int>(d * num_blocks);
	device_pointer_tangent_points = new thrust::device_vector<float>(d * num_tangent_fields * num_blocks);
	device_pointer_outline = new thrust::device_vector<float>(d * num_outline_fields * max_num_outlines);
	device_pointer_blockid_to_pose_unit_id_map = new thrust::device_vector<int>(num_blocks);
	device_pointer_blockid_to_shape_unit_id_map = new thrust::device_vector<int>(num_blocks);

	cudax::_multiply_on_cpu = multiply_on_cpu;
	cudax::host_calibrate = calibrate;
	cudax::_verify_jacobian = verify_jacobian;
	cudax::_compute_weighted_metric = compute_weighted_metric;
	cudaMemcpyToSymbol(device_multiply_on_cpu, &multiply_on_cpu, sizeof(int));
	cudaMemcpyToSymbol(device_calibrate, &calibrate, sizeof(bool));

	t->display();
	// t->set_prefix(" + ");
	is_init = true;
}

//=============================================================================
/// Destructor
//=============================================================================
void kernel_cleanup() {
	using namespace cudax;
	delete t;
	delete J;
	delete F;
	delete JtJ; 
	delete JtF;

	delete camera_matrix;

	delete _sensor_indicator;
	delete push_indices;
	delete _rendered_pixels;
	delete _rendered_points;
	delete _rendered_normals;
	delete _rendered_block_ids;

	delete _sensor_silhouette_distance_transform;
	delete _sensor_silhouette_wrist_distance_transform;
	delete _sensor_outline_distance_transform;

	delete device_pointer_centers;
	delete device_pointer_radii;
	delete device_pointer_blocks;
	delete device_pointer_tangent_points;
	delete device_pointer_outline;
	delete device_pointer_blockid_to_pose_unit_id_map;
	delete device_pointer_blockid_to_shape_unit_id_map;
}

//=============================================================================
/// Checks
//=============================================================================
void kernel_memory_tests() {
	/// Make sure data alignment is not f'd when we pass data to cuda

	if (!(sizeof(glm::mat4x4) == (16 * sizeof(float)))) {
		printf("glm::mat4x4 memory alignment error\n");
		exit(0);
	}

	/*/// Make sure memory transfers are done correctly
	if (!(sizeof(cudax::J_row) == (sizeof(float) * NUM_PARAMETERS))) {
		printf("memory alignment error\n");
		printf("sizeof(cudax::J_row) = %d\n", sizeof(cudax::J_row));
		printf("sizeof(float) * NUM_PARAMETERS = %d\n", sizeof(float) * NUM_PARAMETERS);
		printf("sizeof(float) = %d\n", sizeof(float));
		printf("NUM_PARAMETERS = %d\n", NUM_PARAMETERS);
		exit(0);
	}*/
}

void kernel_delete() {

}
