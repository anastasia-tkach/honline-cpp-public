
#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h" 
#include "apps/honline_atb/AntTweakBarEventFilter.h"

#include "TwSettings.h"
#include "GLWidget.h"
#include "tracker/Data/Camera.h"
#include "tracker/Worker.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Data/SolutionStream.h"
#include "tracker/HModel/GroundTruthLoader.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/HModel/JacobianVerifier.h"
#include "tracker/HModel/KalmanFilter.h"
#include "tracker/HModel/BatchSolver.h"

#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"

#include <iomanip>
#include <chrono>

#define M_PI 3.14159265358979323846

GLWidget::GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solutions, bool playback, bool real_color, std::string data_path, std::string sequence_path) :
QGLWidget(OpenGL32Format()),
	worker(worker),
	datastream(datastream),
	solutions(solutions),
	camera(worker->camera),
	convolution_renderer(worker->model, real_color, data_path, worker->camera->height(), worker->camera->width()) {

	//QSurfaceFormat format; format.setVersion(3, 2); setFormat(format);

	this->playback = playback;
	this->data_path = data_path;
	this->sequence_path = sequence_path;
	this->resize(640 * 2, 480 * 2);
	this->move(1250, 375);
	convolution_renderer.window_width = this->width();
	convolution_renderer.window_height = this->height();

	if (worker->settings->show_initialization_calls) cout << "exiting constructor of glwidget" << endl;
	DebugRenderer::instance().set_data_path(worker->settings->data_path);
}

GLWidget::~GLWidget() {
	worker->cleanup_graphic_resources();
	tw_settings->tw_cleanup();
}

std::vector<std::pair<Vector3, Vector3>> GLWidget::prepare_data_correspondences_for_degub_renderer() {
	worker->jacobian_verifier->num_sensor_points = worker->handfinder->num_sensor_points;
	std::vector<std::pair<Vector3, Vector3>> segments;
	for (size_t i = 0; i < worker->jacobian_verifier->num_sensor_points; i++) {
		Vector3 data_point = Vector3(
			worker->jacobian_verifier->correspondences_data_points[3 * i + 0],
			worker->jacobian_verifier->correspondences_data_points[3 * i + 1],
			worker->jacobian_verifier->correspondences_data_points[3 * i + 2]);

		Vector3 model_point = Vector3(
			worker->jacobian_verifier->correspondences_model_points[3 * i + 0],
			worker->jacobian_verifier->correspondences_model_points[3 * i + 1],
			worker->jacobian_verifier->correspondences_model_points[3 * i + 2]);
		segments.push_back(std::pair<Vector3, Vector3>(data_point, model_point));
	}
	return segments;
}

std::vector<std::pair<Vector3, Vector3>> GLWidget::prepare_silhouette_correspondences_for_degub_renderer() {
	worker->jacobian_verifier->num_rendered_points = 0;
	if (worker->E_fitting.settings->fit2D_silhouette2outline_enable || worker->E_fitting.settings->fit2D_outline_enable)
		worker->jacobian_verifier->num_rendered_points = worker->model->num_rendered_points;
	if (worker->E_fitting.settings->fit2D_silhouette_enable)
		worker->jacobian_verifier->num_rendered_points = worker->E_fitting.num_rendered_points;
	std::vector<std::pair<Vector3, Vector3>> segments;
	for (size_t i = 0; i < worker->jacobian_verifier->num_rendered_points; i++) {
		Vector3 silhouette_data_point = Vector3(
			worker->jacobian_verifier->correspondences_silhouette_data_points[3 * i + 0],
			worker->jacobian_verifier->correspondences_silhouette_data_points[3 * i + 1],
			worker->jacobian_verifier->correspondences_silhouette_data_points[3 * i + 2]);

		Vector3 silhouette_model_point = Vector3(
			worker->jacobian_verifier->correspondences_silhouette_model_points[3 * i + 0],
			worker->jacobian_verifier->correspondences_silhouette_model_points[3 * i + 1],
			worker->jacobian_verifier->correspondences_silhouette_model_points[3 * i + 2]);
		segments.push_back(std::pair<Vector3, Vector3>(silhouette_data_point, silhouette_model_point));
	}
	return segments;
}

void GLWidget::initializeGL() {

	initialize_glew();
	tw_settings->tw_init(this->width(), this->height()); ///< FIRST!!

	glEnable(GL_DEPTH_TEST);

	kinect_renderer.init(camera, worker->settings->dataset_type == SHARP);

	///--- Initialize other graphic resources
	this->makeCurrent();
	worker->init_graphic_resources();

	///--- Setup with data from worker
	kinect_renderer.setup(worker->sensor_color_texture->texture_id(), worker->sensor_depth_texture->texture_id());

	convolution_renderer.projection = camera->view_projection_matrix();
	convolution_renderer.init(ConvolutionRenderer::NORMAL);
	if (worker->settings->display_estimated_certainty) convolution_renderer.display_estimated_certainty = true;
	if (worker->settings->display_measured_certainty) convolution_renderer.display_measured_certainty = true;
	worker->ground_truth_loader->load_ground_truth_marker_positions();

	if (worker->settings->show_initialization_calls) cout << "finished initialize gl" << endl;
}

void GLWidget::paintGL() {

	glViewport(0, 0, this->width(), this->height());
	glClearColor(1, 1, 1, 1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	Eigen::Matrix4f view_projection = camera->view_projection_matrix() * view;
	kinect_renderer.enable_colormap(true);

	kinect_renderer.set_zNear(100);
	kinect_renderer.set_zFar(1500); 
	kinect_renderer.set_alpha(0.87);
	kinect_renderer.set_uniform("view_projection", view_projection);
	int period = 60;
	if (worker->calibration_finished && convolution_renderer.num_frames_since_calibrated < 0) convolution_renderer.num_frames_since_calibrated = 0; // for playback
	if (convolution_renderer.num_frames_since_calibrated > 0 && convolution_renderer.num_frames_since_calibrated < 2 * period) {		
		float brightess;
		if (convolution_renderer.num_frames_since_calibrated < period)
			brightess = (1 - 0.7 * (convolution_renderer.num_frames_since_calibrated % period) / period);
		else
			brightess = (1 - 0.7 * (period - convolution_renderer.num_frames_since_calibrated % period) / period);
		kinect_renderer.set_alpha(0.87 * brightess);
	}

	if (display_sensor_data && (convolution_renderer.num_frames_since_calibrated < period * 0.5 || convolution_renderer.num_frames_since_calibrated > 1.5 * period)) kinect_renderer.render();
	
	glDisable(GL_BLEND);

	if (display_hand_model) convolution_renderer.render(!worker->settings->stop_tracking_without_wristband || worker->handfinder->wristband_found());

	if (worker->settings->display_estimated_certainty || worker->settings->display_measured_certainty) {
		//if (worker->current_frame.id == 700 && convolution_renderer.num_frames_since_calibrated < 0) convolution_renderer.num_frames_since_calibrated = 0;
		//if (convolution_renderer.num_frames_since_calibrated == 0 && !playback) {

		// fabrice - 400, // timur - 599 // pier2 - 599 // jan2 - 1199 // edoardo4 - 599 // filippe - 599 // matthieu2 - 1359 // jacomo3 - 599 // madeleine2 - 599 // stefano2 - 599 // anastasia - 519 // isinsu - 599
		// mina - 599 // andrii5 - 399 // luca 859 // alexis3 - 1339
		int last_calibration_frame = 599;
		if (convolution_renderer.num_frames_since_calibrated == 0 || (worker->current_frame.id == last_calibration_frame || worker->current_frame.id == last_calibration_frame + 1 && convolution_renderer.num_frames_since_calibrated < 0)) {
			cout << "CALIBRATED: " << worker->current_frame.id << endl;
			convolution_renderer.num_frames_since_calibrated = 0;

			{ /// Write calibrated model to file
				std::vector<float> theta = worker->model->get_theta();
				worker->model->set_initial_pose();
				worker->model->write_model(sequence_path);
				worker->model->update_theta(theta); worker->model->update_centers();
			}

			{ /// Stop calibrating
				//worker->settings->restart_clock = true;
				worker->set_calibration_type(NONE);
				worker->E_fitting.settings->fit2D_outline_enable = false;
				worker->E_fitting.settings->fit2D_weight = 0.4f;
				worker->settings->termination_max_iters = 6;
				worker->E_fingertips._settings.enable_fingertips_prior = false;
				worker->E_pose._settings.weight_proj = 4 * 10e2;
				convolution_renderer.display_estimated_certainty = false;
				convolution_renderer.display_measured_certainty = false;
				worker->settings->run_kalman_filter = false;
				worker->E_temporal._settings.temporal_coherence1_weight = 0.05f;
				worker->E_temporal._settings.temporal_coherence2_weight = 0.05f;
			}
			
		}
		if (convolution_renderer.num_frames_since_calibrated >= 0) {
			convolution_renderer.num_frames_since_calibrated++;
		}

	}
	
	if (display_model_outline || display_data_outline || display_data_correspondences || display_silhouette_correspondences || display_ground_truth_marker_positions || display_fingertips || display_ground_truth_reinit_constraints) {
		DebugRenderer::instance().clear();

		if (display_ground_truth_reinit_constraints) {

		}

		if (display_fingertips) {
			std::vector<std::pair<Vector3, Vector3>> active_fingertips_segments;
			std::vector<std::pair<Vector3, Vector3>> inactive_fingertips_segments;
			std::vector<Vector3> data_fingertips_points;
			std::vector<Vector3> model_fingertips_points;
			for (size_t f = 0; f < num_fingers; f++) {
				if (glm::length(worker->E_fingertips.ordered_data_fingertips[f]) < std::numeric_limits<float>::epsilon()) {
					continue;
				}
				if (glm::length(worker->E_fingertips.ordered_data_fingertips[f] - worker->E_fingertips.ordered_model_fingertips[f]) <= 10.0) {
					active_fingertips_segments.push_back(std::pair<Vector3, Vector3>(
						Vector3(worker->E_fingertips.ordered_data_fingertips[f][0], worker->E_fingertips.ordered_data_fingertips[f][1], worker->E_fingertips.ordered_data_fingertips[f][2]),
						Vector3(worker->E_fingertips.ordered_model_fingertips[f][0], worker->E_fingertips.ordered_model_fingertips[f][1], worker->E_fingertips.ordered_model_fingertips[f][2])));
				}
				else {
					inactive_fingertips_segments.push_back(std::pair<Vector3, Vector3>(
						Vector3(worker->E_fingertips.ordered_data_fingertips[f][0], worker->E_fingertips.ordered_data_fingertips[f][1], worker->E_fingertips.ordered_data_fingertips[f][2]),
						Vector3(worker->E_fingertips.ordered_model_fingertips[f][0], worker->E_fingertips.ordered_model_fingertips[f][1], worker->E_fingertips.ordered_model_fingertips[f][2])));
				}

				data_fingertips_points.push_back(Vector3(worker->E_fingertips.ordered_data_fingertips[f][0], worker->E_fingertips.ordered_data_fingertips[f][1], worker->E_fingertips.ordered_data_fingertips[f][2]));	
				model_fingertips_points.push_back(Vector3(worker->E_fingertips.ordered_model_fingertips[f][0], worker->E_fingertips.ordered_model_fingertips[f][1], worker->E_fingertips.ordered_model_fingertips[f][2]));
			}
			DebugRenderer::instance().add_segments(active_fingertips_segments, Vector3(0.0, 0.6, 0.6));
			DebugRenderer::instance().add_segments(inactive_fingertips_segments, Vector3(0.9, 0.0, 0.6));
			DebugRenderer::instance().add_points(data_fingertips_points, Vector3(0.0, 0.6, 0.6));
			DebugRenderer::instance().add_points(model_fingertips_points, Vector3(0.0, 0.0, 0.0));
		}
		if (display_model_outline) {
			worker->model->render_outline();
		}
		if (display_data_outline) {
			std::vector<Vector3> data_outline_points;			
			for (int row = 0; row < worker->handfinder->sensor_outline.rows; row++) {
				for (int col = 0; col < worker->handfinder->sensor_outline.cols; col++) {
					if (worker->handfinder->sensor_outline.at<unsigned char>(row, col) != 255) continue;
					unsigned short depth_value = worker->current_frame.depth.at<unsigned short>(row, col);
					if (depth_value == 0) continue; 
					Vector3 data_point = worker->camera->unproject(col, worker->camera->height() - 1 - row, depth_value);
					data_outline_points.push_back(data_point);
				}
			}
			DebugRenderer::instance().add_points(data_outline_points, Vector3(0.0, 0.6, 0.6));
		}
		if (display_data_correspondences) {
			std::vector<std::pair<Vector3, Vector3>> segments = prepare_data_correspondences_for_degub_renderer();
			DebugRenderer::instance().add_segments(segments, Vector3(0.65, 0.65, 0.65));
		}
		if (display_silhouette_correspondences) {
			std::vector<std::pair<Vector3, Vector3>> segments = prepare_silhouette_correspondences_for_degub_renderer();
			DebugRenderer::instance().add_segments(segments, Vector3(0.65, 0.65, 0.65));
		}

		if (display_ground_truth_marker_positions || display_model_marker_positions) {
			std::vector<Vector3> ground_truth_marker_positions;
			std::vector<Vector3> model_marker_positions;
			std::vector<size_t> model_marker_block_indices;
			if (display_ground_truth_marker_positions) {
				ground_truth_marker_positions = worker->ground_truth_loader->get_ground_truth_marker_positions(worker->current_frame.id);
				DebugRenderer::instance().add_points(ground_truth_marker_positions, Vector3(0, 0, 0));
			}
			if (display_model_marker_positions) {				
				worker->ground_truth_loader->get_marker_positions(model_marker_positions, model_marker_block_indices);
				DebugRenderer::instance().add_points(model_marker_positions, Vector3(1, 0, 0));
			}
			if (display_ground_truth_marker_positions && display_model_marker_positions) {
				std::vector<std::pair<Vector3, Vector3>> segments;
				for (size_t i = 0; i < ground_truth_marker_positions.size(); i++) {
					segments.push_back(std::pair<Vector3, Vector3>(ground_truth_marker_positions[i], model_marker_positions[i]));
				}
				DebugRenderer::instance().add_segments(segments, Vector3(1, 0, 0));
			}

			//DebugRenderer::instance().add_points(worker->ground_truth_loader->active_model_points, Vector3(1, 0, 0));
			//DebugRenderer::instance().add_points(worker->ground_truth_loader->active_data_points, Vector3(0, 0, 0));
			//std::vector<std::pair<Vector3, Vector3>> segments;
			//for (size_t i = 0; i < worker->ground_truth_loader->active_model_points.size(); i++) {
			//	segments.push_back(std::pair<Vector3, Vector3>(worker->ground_truth_loader->active_model_points[i], worker->ground_truth_loader->active_data_points[i]));
			//}
			//DebugRenderer::instance().add_segments(segments, Vector3(0, 0, 1));
		}

		DebugRenderer::instance().set_uniform("view_projection", view_projection);
		DebugRenderer::instance().render();
	}

	if (worker->settings->run_batch_solver) worker->batch_solver->display_frames();
}

void GLWidget::process_mouse_movement(GLfloat cursor_x, GLfloat cursor_y) {
	glm::vec3 image_center_glm = worker->model->centers[worker->model->centers_name_to_id_map["palm_back"]] +
		worker->model->centers[worker->model->centers_name_to_id_map["palm_middle"]];
	image_center = Eigen::Vector3f(image_center_glm[0] / 2, image_center_glm[1] / 2 + 30, image_center_glm[2] / 2);
	float d = (camera_center - image_center).norm();

	float delta_x = cursor_x - cursor_position[0];
	float delta_y = cursor_y - cursor_position[1];

	float theta = initial_euler_angles[0] + cursor_sensitivity * delta_x;
	float phi = initial_euler_angles[1] + cursor_sensitivity * delta_y;

	Eigen::Vector3f x = sin(theta) * sin(phi) * Eigen::Vector3f::UnitX();
	Eigen::Vector3f y = cos(phi) * Eigen::Vector3f::UnitY();
	Eigen::Vector3f z = cos(theta) * sin(phi) * Eigen::Vector3f::UnitZ();

	camera_center = image_center + d * (x + y + z);
	euler_angles = Eigen::Vector2f(theta, phi);

	Vector3 f, u, s;
	f = (image_center - camera_center).normalized();
	u = camera_up.normalized();
	s = u.cross(f).normalized();
	u = f.cross(s);
	view.block(0, 0, 1, 3) = s.transpose();
	view(0, 3) = -s.dot(camera_center);
	view.block(1, 0, 1, 3) = u.transpose();
	view(1, 3) = -u.dot(camera_center);
	view.block(2, 0, 1, 3) = f.transpose();
	view(2, 3) = -f.dot(camera_center);

	// set view matrix 
	convolution_renderer.camera.view = view;
	convolution_renderer.camera.camera_center = camera_center;

	worker->offscreen_renderer.convolution_renderer->camera.view = view;
	worker->offscreen_renderer.convolution_renderer->camera.camera_center = camera_center;
}

void GLWidget::process_mouse_wheel(int delta) {
	camera_center = image_center + (1 + wheel_sensitivity * delta) * (camera_center - image_center);

	Vector3 f, u, s;
	f = (image_center - camera_center).normalized();
	u = camera_up.normalized();
	s = u.cross(f).normalized();
	u = f.cross(s);
	view.block(0, 0, 1, 3) = s.transpose();
	view(0, 3) = -s.dot(camera_center);
	view.block(1, 0, 1, 3) = u.transpose();
	view(1, 3) = -u.dot(camera_center);
	view.block(2, 0, 1, 3) = f.transpose();
	view(2, 3) = -f.dot(camera_center);

	// set view matrix 
	convolution_renderer.camera.view = view;
	convolution_renderer.camera.camera_center = camera_center;

	worker->offscreen_renderer.convolution_renderer->camera.view = view;
	worker->offscreen_renderer.convolution_renderer->camera.camera_center = camera_center;
}

void GLWidget::process_mouse_button_pressed(GLfloat cursor_x, GLfloat cursor_y) {
	mouse_button_pressed = true;
	cursor_position = Eigen::Vector2f(cursor_x, cursor_y);
}

void GLWidget::process_mouse_button_released() {
	initial_euler_angles = euler_angles;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event) {
	if (event->buttons() == Qt::LeftButton) {
		process_mouse_movement(event->x(), event->y());
	}
	else {
		if (mouse_button_pressed == true) {
			process_mouse_button_released();
			mouse_button_pressed = false;
		}
	}
}

void GLWidget::mousePressEvent(QMouseEvent *event) {
	process_mouse_button_pressed(event->x(), event->y());
}

void GLWidget::wheelEvent(QWheelEvent * event) {
	if (event->delta() > 0)
		process_mouse_wheel(-1);
	else
		process_mouse_wheel(1);
}

void GLWidget::keyPressEvent(QKeyEvent *event) {
	GLWidget* qglviewer = this;
	switch (event->key()) {

	case Qt::Key_E: { // E - experiment
		//std::vector<float> beta = worker->model->get_beta(); beta[36] -= 0.5;
		//worker->model->update_beta(beta);
		//std::vector<float> theta = std::vector<float>(num_thetas, 0);
		//theta[1] = -50; theta[2] = 375;
		//worker->model->update_theta(theta);
		//worker->model->update_centers();

		/// Adjust initial transformations
		/*{ 
			Mat3f R = worker->model->phalanges[worker->model->phalanges_name_to_id_map["HandThumb1"]].init_local.block(0, 0, 3, 3);
			Eigen::Vector3f euler_angles = R.eulerAngles(0, 1, 2);
			euler_angles[2] -= 0.5;
			cout << euler_angles.transpose() << endl;
			R = Eigen::AngleAxisf(euler_angles[0], Eigen::Vector3f::UnitX())
				*Eigen::AngleAxisf(euler_angles[1], Eigen::Vector3f::UnitY())
				*Eigen::AngleAxisf(euler_angles[2], Eigen::Vector3f::UnitZ());
			worker->model->phalanges[worker->model->phalanges_name_to_id_map["HandThumb1"]].init_local.block(0, 0, 3, 3) = R;
		}*/

		/// Perturb parameters
		/*{
			worker->model->perturb_parameters(std::chrono::system_clock::now().time_since_epoch().count() % RAND_MAX, worker->settings->uniform_scaling_mean, 0.4, 0.0);
		}*/

	} break;

	case Qt::Key_A: { // A - add frame
		if (worker->settings->run_batch_solver == false) {
			cout << "batch solver is off" << endl;
			return;
		}
		cout << "adding a frame to batch solver" << endl;
		worker->batch_solver->add_frame();
		activateWindow();
	} break;

	case Qt::Key_B: { // B - batch solve
		if (event->modifiers()&Qt::ShiftModifier) {
			cout << "stopping batch calibration" << endl;
			worker->batch_solver->first_batch_solve_iteration = true;
			worker->set_calibration_type(NONE);

			/// > write calibrated model
			worker->model->set_initial_pose();
			worker->model->write_model(worker->model->calibrated_model_path);

			worker->settings->pause_tracking = false;
			{
				/*this is because sensor_depth_texture_cuda is all zeros for some reason
				right after switching back to tracking, which creates nans in fit2D_unproject*/
				worker->E_fitting.settings->fit2D_outline_enable = true;
				worker->E_fitting.settings->fit2D_silhouette2outline_enable = false;
				worker->E_fitting.settings->fit2D_silhouette_enable = false;
				worker->E_fitting.settings->fit2D_unproject = true;
			}
			glFinish();
			return;
		}

		if (worker->settings->run_batch_solver == false) {
			cout << "batch solver is off" << endl;
			return;
		}
		cout << "starting batch calibration" << endl;
		worker->settings->pause_tracking = true;
		{
			worker->E_fitting.settings->fit2D_outline_enable = true;
			worker->E_fitting.settings->fit2D_silhouette2outline_enable = false;
			worker->E_fitting.settings->fit2D_silhouette_enable = false;
			worker->E_fitting.settings->fit2D_unproject = true;

			worker->E_shape._settings.enable_semantic_limits = false;
			worker->E_shape._settings.enable_shape_dofs_blockers = false;
		}
		worker->set_calibration_type(FULL);
		cout << "changed calibration type" << endl;
		//worker->batch_solver->batch_solve_iteration();
		worker->batch_solver->batch_solve(5);
		cout << "finished batch iteration" << endl;

	} break;

	case Qt::Key_W: { // W - weight change

		cout << "write to file and load again" << endl;
		
		worker->settings->pause_tracking = true;
		std::vector<float> theta = worker->model->get_theta();
		worker->model->set_initial_pose();
		worker->model->write_model(worker->model->calibrated_model_path);

		worker->set_calibration_type(NONE);
		worker->model->load_model_from_file(true);
		worker->model->update_parameters(std::vector<float>(num_thetas, 0));
		worker->model->initialize_offsets(false);

		worker->model->update_theta(theta);
		worker->model->update_centers();


	} break;

	case Qt::Key_M: {
		cout << "displaying model" << endl;
		display_hand_model = !display_hand_model;
	} break;

	case Qt::Key_D: {
		cout << "displaying data" << endl;
		display_sensor_data = !display_sensor_data;
	} break;

	case Qt::Key_O: {
		if (event->modifiers()&Qt::ShiftModifier) {
			cout << "displaying data outline" << endl;
			display_data_outline = !display_data_outline;
		}
		else {
			cout << "displaying model outline" << endl;
			display_model_outline = !display_model_outline;
		}
	} break;

	case Qt::Key_H: {
		if (event->modifiers()&Qt::ShiftModifier) {
			cout << "displaying estimated certainty" << endl;
			worker->settings->display_estimated_certainty = !worker->settings->display_estimated_certainty;
			convolution_renderer.display_estimated_certainty = worker->settings->display_estimated_certainty;
			worker->settings->display_measured_certainty = false;
		}
		else {
			cout << "displaying measured certainty" << endl;
			worker->settings->display_measured_certainty = !worker->settings->display_measured_certainty;
			convolution_renderer.display_measured_certainty = worker->settings->display_measured_certainty;
			worker->settings->display_estimated_certainty = false;
		}
		cout << "display_estimated_certainty = " << worker->settings->display_estimated_certainty << endl;
		cout << "display_measured_certainty = " << worker->settings->display_measured_certainty << endl;
		cout << "conv:display_estimated_certainty = " << convolution_renderer.display_estimated_certainty << endl << endl;
		cout << "conv:display_measured_certainty = " << convolution_renderer.display_measured_certainty << endl << endl;
	} break;

	case Qt::Key_C: {
		if (worker->settings->verify_jacobian == false) {
			cout << "to be able to display correspondences, set 'verify_jacobian' to true";
			return;
		}

		if (event->modifiers()&Qt::ShiftModifier) {
			cout << "displaying data corresponences" << endl;
			display_data_correspondences = !display_data_correspondences;
		}
		else {
			cout << "displaying silhouette corresponences" << endl;
			display_silhouette_correspondences = !display_silhouette_correspondences;
		}	
	} break;

	case Qt::Key_G: {
		cout << "displaying ground truth and model markers" << endl;
		display_ground_truth_marker_positions = !display_ground_truth_marker_positions;
		display_model_marker_positions = !display_model_marker_positions;
	} break;

	case Qt::Key_Minus: {
		if (worker->model->calibration_type == NONE) {
			cout << "already no calibration" << endl;
			return;
		}
		cout << "removing calibration dofs" << endl;
		switch (worker->model->calibration_type) {
		case FINGERS_PARTIAL: worker->set_calibration_type(NONE); break;
		case FINGERS: worker->set_calibration_type(FINGERS_PARTIAL); break;
		case FINGERS_AND_PALM: worker->set_calibration_type(FINGERS); break;
		case FULL: worker->set_calibration_type(NONE); break;
		}
		worker->model->set_initial_pose();
		worker->model->write_model(worker->model->calibrated_model_path);
	} break;

	case Qt::Key_Plus: {
		if (worker->model->calibration_type == FULL) {
			cout << "already full calibration" << endl;
			return;
		}
		cout << "adding calibration dofs" << endl;
		switch (worker->model->calibration_type) {
		case NONE: worker->set_calibration_type(FULL); break;	
		}
	} break;

	case Qt::Key_Asterisk: {
		if (worker->model->calibration_type == NONE) {
			cout << "no calibration" << endl;
			return;
		}
		if (worker->model->calibration_stage == DN) {
			cout << "already last calibration stage" << endl;
			return;
		}
		cout << "increasing calibration stage" << endl;
		switch (worker->model->calibration_stage) {
		case DEFAULT: worker->set_calibration_stage(D1); break;
		case D1: worker->set_calibration_stage(D4); break;
		case D4: worker->set_calibration_stage(DN); break;
		}
	} break;

	case Qt::Key_Left: {
		if (worker->model->beta_saved.empty()) { /// Saving
			cout << "saving current beta" << endl;
			worker->model->beta_saved = worker->model->beta;
		}
		else { /// Comparing
			cout << "setting saved beta" << endl;
			worker->settings->use_online_beta = true;
			
			if (worker->model->calibration_type == NONE) worker->set_calibration_type(FULL);
			std::vector<float> beta_temp = worker->model->beta_saved;
			worker->model->beta_saved = worker->model->get_beta();
			worker->model->beta = beta_temp;
			
			worker->model->update_beta(worker->model->beta);
			worker->set_calibration_type(NONE);
		}
	} break;

	case Qt::Key_K: {
		apply_estimated_shape = !apply_estimated_shape;
		if (apply_estimated_shape == true) {
			worker->set_calibration_type(NONE);
			{ /// write calibrated model
			//worker->model->set_initial_pose();
			//worker->model->write_model(worker->model->calibrated_model_path);
			}
		}
		if (apply_estimated_shape == false) {
			worker->settings->restart_clock = true;
			worker->set_calibration_type(FULL);
		}
	} break;

	case Qt::Key_Escape: {
		this->close();
	} break;

	case Qt::Key_P: {
		if (event->modifiers()&Qt::ShiftModifier) {
			cout << "pause current frame" << endl;
			worker->settings->pause_current_frame = !worker->settings->pause_current_frame;
		}
		else {
			cout << "pause tracking" << endl;
			worker->settings->pause_tracking = !worker->settings->pause_tracking;
		}
	} break;

	case Qt::Key_J: {
		if (worker->settings->verify_jacobian == false) {
			cout << "'verify_jacobian' is set to false";
			return;
		}
		else {
			worker->settings->pause_tracking = true;
			worker->jacobian_verifier->num_sensor_points = worker->handfinder->num_sensor_points;
			worker->jacobian_verifier->num_rendered_points = worker->model->num_rendered_points;
			//worker->jacobian_verifier->write_corresponences_to_file(worker->current_frame.id);
			//worker->model->write_model(worker->settings->logs_path, 0);

			{ // return to previous iteration, since we have gpu jacobians from there 
				std::vector<float> beta_float(worker->jacobian_verifier->beta_double.begin(), worker->jacobian_verifier->beta_double.end());
				std::vector<float> theta_float(worker->jacobian_verifier->theta_double.begin(), worker->jacobian_verifier->theta_double.end());
				worker->model->update_beta(beta_float);
				worker->model->update_theta(theta_float);
				worker->model->update_centers();
				worker->model->compute_outline();
			}

			if (event->modifiers()&Qt::ShiftModifier) {
				worker->jacobian_verifier->main_verify_jacobian_for_one_point();
			}
			else {
				worker->jacobian_verifier->main_verify_jacobian_for_all_points();
			}
		}
	} break;

	case Qt::Key_I: {
		cout << "run one iteration" << endl;
		worker->settings->pause_tracking = true;
		worker->track(1);
		worker->updateGL();
		worker->updateGL();
		if (worker->settings->run_batch_solver) {
			worker->batch_solver->update_pose_icons();
		}
	} break;

	case Qt::Key_N: {
		if (event->modifiers()&Qt::ShiftModifier) {
			if (worker->settings->pause_current_frame) {
				cout << "previous frame" << endl;
				worker->current_frame.id--;
			}
			else {
				cout << "current frame is not paused" << endl;
			}
		}
		else {			
			if (worker->settings->pause_current_frame) {
				cout << "next frame" << endl;
				worker->current_frame.id++;
			}
			else {
				cout << "current frame is not paused" << endl;
			}
		}
	} break;

	case Qt::Key_S: {
		cout << "set up path for saving images" << std::endl;

		/*int last_folder_index = -1; int new_folder_index = 0;
		{ /// > Create new directory			
			WIN32_FIND_DATA search_data;
			memset(&search_data, 0, sizeof(WIN32_FIND_DATA));
			HANDLE handle = FindFirstFile((path + "\*").c_str(), &search_data);
			while (handle != INVALID_HANDLE_VALUE) {
				std::string filename = search_data.cFileName;
				cout << filename << endl;
				if (isdigit(filename[0])) {
					int folder_index = std::stoi(filename);
					if (folder_index > last_folder_index)
						last_folder_index = folder_index;
				}
				if (FindNextFile(handle, &search_data) == FALSE) break;
			}
			FindClose(handle);
			new_folder_index = last_folder_index + 1;
			if (CreateDirectory((path + "\\" + std::to_string(new_folder_index)).c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError());
			path +=  std::to_string(new_folder_index) + "/";
		}*/

		datastream->save_as_images(worker->settings->sequence_path + "experiment/");

		/// Write sequence length
		std::string sequence_length_filename = worker->settings->sequence_path + "experiment/" + "sequence_length.txt";
		static ofstream sequence_length_file(sequence_length_filename);
		if (sequence_length_file.is_open())	sequence_length_file << datastream->get_last_frame_index() + 1 <<  endl;
		sequence_length_file.close();

	} break;

	case Qt::Key_F: {

		if (event->modifiers()&Qt::ShiftModifier) {
			cout << "displaying fingertips" << endl;
			display_fingertips = !display_fingertips;
		}
		else {
			cout << "saving current frame" << std::endl;
			size_t frame_index = datastream->get_last_frame_index();
			datastream->save_as_image(worker->settings->logs_path, frame_index);

			std::ostringstream stringstream;
			stringstream << frame_index;
			std::string filename = data_path + "mask-" + stringstream.str() + ".png";
			cv::imwrite(filename, worker->handfinder->sensor_silhouette);

			worker->model->write_model(data_path, frame_index);
		}
	} break;

	case Qt::Key_1: {
		cout << "uniform scaling up" << endl;
		worker->model->resize_model(1.05, 1.0, 1.0);
	} break;

	case Qt::Key_2: {
		cout << "uniform scaling down" << endl;
		worker->model->resize_model(0.95, 1.0, 1.0);
	} break;

	case Qt::Key_3: {
		cout << "width scaling up" << endl;
		worker->model->resize_model(1.0, 1.05, 1.0);
	} break;

	case Qt::Key_4: {
		cout << "width scaling down" << endl;
		worker->model->resize_model(1.0, 0.95, 1.0);
	} break;

	case Qt::Key_5: {
		cout << "thickness scaling up" << endl;
		worker->model->resize_model(1.0, 1.0, 1.05);
	} break;

	case Qt::Key_6: {
		cout << "thickness scaling down" << endl;
		worker->model->resize_model(1.0, 1.0, 0.95);
	} break;

	}
}

