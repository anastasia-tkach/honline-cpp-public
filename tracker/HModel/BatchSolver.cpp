#include "tracker/HModel/BatchSolver.h"
#include "tracker/Worker.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Data/TextureDepth16UC1.h"

namespace batchsolver_callback_namespace {
	int current_pose_index = -1;
	static void on_mouse_click(int event, int pixels_x, int pixels_y, int f, void* ptr) {
		size_t heigth = 240;
		size_t width = 320;
		if (event == cv::EVENT_LBUTTONDOWN) {
			current_pose_index = pixels_x / width;
		}
	}
}

BatchSolver::BatchSolver(Worker * worker) {
	if (!worker->settings->use_online_betas_for_batch_solver && worker->model->calibration_type != NONE) {
		cout << endl << "RUNNING OFFLINE BATCH WHILE CALIBRATION ONLINE" << endl << endl;
		return;
	}
	if (worker->settings->use_online_betas_for_batch_solver && worker->model->calibration_type == NONE) {
		cout << endl << "RUNNING ONLINE BATCH WITHOUT CALIBRATING ONLINE" << endl << endl;
		return;
	}
	if (worker->settings->use_online_betas_for_batch_solver && !worker->E_shape._settings.enable_shape_prior) {
		cout << "ONLINE BATCH DOES NOT RUN WITHOUT SHAPE PRIOR" << endl;
		return;
	}

	this->worker = worker;
	this->use_online_betas = worker->settings->use_online_betas_for_batch_solver;
	window_name = "poses";
}

cv::Mat BatchSolver::get_current_pose_icon() {
	if (displayed_pose_index == -1) {
		cout << "can't update icon, no pose is set to current" << endl;
		return cv::Mat();
	}

	// get first channel of model image
	cv::Mat model_image = worker->model->silhouette_texture.clone();
	cv::Mat split_channels_array[3];
	split(model_image, split_channels_array);
	cv::flip(split_channels_array[0], split_channels_array[0], 0);

	// assemble three channels of model and data image
	cv::Mat empty_channel = cv::Mat::zeros(worker->camera->height(), worker->camera->width(), CV_8UC1); empty_channel.setTo(255);
	std::vector<cv::Mat> merge_channels_vector;
	merge_channels_vector.push_back(empty_channel);
	merge_channels_vector.push_back(sensor_silhoettes[displayed_pose_index].clone());
	cv::bitwise_not(merge_channels_vector[1], merge_channels_vector[1]);
	merge_channels_vector.push_back(split_channels_array[0]);

	// merge the channels
	cv::Mat merge_channels_image;
	cv::merge(merge_channels_vector, merge_channels_image);
	return merge_channels_image;
}

void BatchSolver::update_pose_icons() {
	cv::Mat pose_icon = get_current_pose_icon();
	cv::Mat auxillary = pose_icons.colRange(displayed_pose_index * worker->camera->width(), (displayed_pose_index + 1) * worker->camera->width());
	pose_icon.copyTo(auxillary);
}

void BatchSolver::apply_pose(int pose_index) {
	worker->sensor_depth_texture->load(data_images[pose_index].data, counter);
	worker->current_frame.depth = data_images[pose_index];

	worker->handfinder->sensor_silhouette = sensor_silhoettes[pose_index];
	if (worker->E_fitting.settings->fit2D_outline_enable) {
		worker->handfinder->sensor_silhouette_closed = sensor_silhoettes_closed[pose_index];
		worker->handfinder->sensor_outline = sensor_outlines[pose_index];
	}
	if (worker->E_fingertips.settings->enable_fingertips_prior) worker->E_fingertips.data_fingertips_2d = data_fingertips_2d[pose_index];

	worker->handfinder->num_sensor_points = 0; int count = 0;
	for (int row = 0; row < worker->handfinder->sensor_silhouette.rows; ++row) {
		for (int col = 0; col < worker->handfinder->sensor_silhouette.cols; ++col) {
			if (worker->handfinder->sensor_silhouette.at<uchar>(row, col) != 255) continue;
			if (count % worker->settings->downsampling_factor == 0) {
				worker->handfinder->sensor_indicator[worker->handfinder->num_sensor_points] = row * worker->camera->width() + col;
				worker->handfinder->num_sensor_points++;
			}
			count++;
		}
	}
	worker->current_frame.id = counter++;

	if (worker->model->num_betas > 0) {
		if (use_online_betas) {
			worker->model->update_beta(betas[pose_index]);
		}
		else {
			worker->model->update_beta(beta);
		}		
	}

	worker->model->update_theta(thetas[pose_index]);
	worker->model->update_centers();
	worker->model->compute_outline();

}

void BatchSolver::add_frame() {
	displayed_pose_index = num_poses;
	num_poses++;
	data_images.push_back(worker->current_frame.depth.clone());
	sensor_silhoettes.push_back(worker->handfinder->sensor_silhouette.clone());
	if (worker->E_fitting.settings->fit2D_outline_enable) {
		sensor_silhoettes_closed.push_back(worker->handfinder->sensor_silhouette_closed.clone());
		sensor_outlines.push_back(worker->handfinder->sensor_outline.clone());
	}
	if (worker->E_fingertips.settings->enable_fingertips_prior) data_fingertips_2d.push_back(worker->E_fingertips.data_fingertips_2d);

	thetas.push_back(worker->model->get_theta());
	if (use_online_betas) {
		betas.push_back(worker->model->get_beta());
		betas_latent.push_back(worker->E_shape.get_beta_latent());
	}

	cv::Mat pose_icon = get_current_pose_icon();
	if (pose_icons.cols == 0) {
		cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);
		pose_icons = pose_icon;
	}
	else {
		cv::hconcat(pose_icons, pose_icon, pose_icons);
	}
	displayed_pose_index = -1;
}

void BatchSolver::display_frames() {
	if (worker->settings->pause_tracking == true && batchsolver_callback_namespace::current_pose_index != previous_pose_index) {
		previous_pose_index = batchsolver_callback_namespace::current_pose_index;
		displayed_pose_index = batchsolver_callback_namespace::current_pose_index; 
		apply_pose(displayed_pose_index);
	}
	if (num_poses > 0) {
		cv::imshow(window_name, pose_icons);
		cv::setMouseCallback(window_name, batchsolver_callback_namespace::on_mouse_click);
	}
}

LinearSystem BatchSolver::build_batch_system_for_offline_beta(const std::vector<LinearSystem> & systems) {
	cout << "building batch system" << endl;
	int num_betas = worker->model->num_betas;
	if (num_betas == 0) {
		cout << "batch solver can't run with num_betas == 0" << endl;
		return LinearSystem(0);
	}
	LinearSystem batch_system = LinearSystem(num_poses * (num_thetas + num_thetas_latent) + (num_betas + num_betas_latent));

	size_t start_betas = num_poses * (num_thetas + num_thetas_latent);
	for (size_t pose_index = 0; pose_index < num_poses; pose_index++) {
		size_t start_thetas = pose_index * (num_thetas + num_thetas_latent);		

		// (theta, theta)
		batch_system.lhs.block(start_thetas, start_thetas, num_thetas, num_thetas) += systems[pose_index].lhs.block(0, 0, num_thetas, num_thetas);
		batch_system.rhs.segment(start_thetas, num_thetas) += systems[pose_index].rhs.segment(0, num_thetas);

		// (theta, theta-latent) x 2
		batch_system.lhs.block(start_thetas, start_thetas + num_thetas, num_thetas, num_thetas_latent) += systems[pose_index].lhs.block(0, num_thetas + num_betas, num_thetas, num_thetas_latent);
		batch_system.lhs.block(start_thetas + num_thetas, start_thetas, num_thetas_latent, num_thetas) += systems[pose_index].lhs.block(num_thetas + num_betas, 0, num_thetas_latent, num_thetas);

		// (theta-latent, theta-latent)
		batch_system.lhs.block(start_thetas + num_thetas, start_thetas + num_thetas, num_thetas_latent, num_thetas_latent) += systems[pose_index].lhs.block(num_thetas + num_betas, num_thetas + num_betas, num_thetas_latent, num_thetas_latent);
		batch_system.rhs.segment(start_thetas + num_thetas, num_thetas_latent) += systems[pose_index].rhs.segment(num_thetas + num_betas, num_thetas_latent);

		// (beta, beta)
		batch_system.lhs.block(start_betas, start_betas, num_betas, num_betas) += systems[pose_index].lhs.block(num_thetas, num_thetas, num_betas, num_betas);
		batch_system.rhs.segment(start_betas, num_betas) += systems[pose_index].rhs.segment(num_thetas, num_betas);

		// (theta, beta) x  2
		batch_system.lhs.block(start_thetas, start_betas, num_thetas, num_betas) = systems[pose_index].lhs.block(0, num_thetas, num_thetas, num_betas);
		batch_system.lhs.block(start_betas, start_thetas, num_betas, num_thetas) = systems[pose_index].lhs.block(num_thetas, 0, num_betas, num_thetas);

		if (worker->E_shape._settings.enable_shape_prior) {
			// (beta, beta-latent) x 2
			batch_system.lhs.block(start_betas, start_betas + num_betas, num_betas, num_betas_latent) = systems[pose_index].lhs.block(num_thetas, num_thetas + num_betas + num_thetas_latent, num_betas, num_betas_latent);
			batch_system.lhs.block(start_betas + num_betas, start_betas, num_betas_latent, num_betas) = systems[pose_index].lhs.block(num_thetas + num_betas + num_thetas_latent, num_thetas, num_betas_latent, num_betas);

			// (beta-latent, beta-latent) 
			batch_system.lhs.block(start_betas + num_betas, start_betas + num_betas, num_betas_latent, num_betas_latent) = systems[pose_index].lhs.block(num_thetas + num_betas + num_thetas_latent, num_betas + num_thetas + num_thetas_latent, num_betas_latent, num_betas_latent);
			batch_system.rhs.segment(start_betas + num_betas, num_betas_latent) = systems[pose_index].rhs.segment(num_thetas + num_betas + num_thetas_latent, num_betas_latent);
		}

		//std::ofstream file_lhs(worker->settings->logs_path + "system-" + std::to_string(pose_index) + "-lhs.txt"); file_lhs << systems[pose_index].lhs;
		//std::ofstream file_rhs(worker->settings->logs_path + "system-" + std::to_string(pose_index) + "-rhs.txt"); file_rhs << systems[pose_index].rhs;
	}
	//std::ofstream file_lhs(worker->settings->logs_path + "batch-system-lhs.txt"); file_lhs << batch_system.lhs;
	//std::ofstream file_rhs(worker->settings->logs_path + "batch-system-rhs.txt"); file_rhs << batch_system.rhs;
	return batch_system;
}

void BatchSolver::parse_parameters_for_offline_beta(const VectorN & solution) {
	size_t num_betas = worker->model->num_betas;

	/// Parse thetas
	for (size_t pose_index = 0; pose_index < num_poses; pose_index++) {
		worker->model->update_theta(thetas[pose_index]); // because "get_updated_parameters" is using global transformation
		std::vector<float> delta_theta = vector<float>(solution.data() + pose_index * (num_thetas + num_thetas_latent), solution.data() + pose_index * (num_thetas + num_thetas_latent) + num_thetas);
		thetas[pose_index] = worker->model->get_updated_parameters(thetas[pose_index], delta_theta);
	}

	/// Parse beta
	for (size_t i = 0; i < num_betas; i++) {
		beta[i] += solution[num_poses * (num_thetas + num_thetas_latent) + i];
	}
	if (worker->E_shape._settings.enable_shape_prior) {
		std::vector<float> delta_beta_latent = std::vector<float>(solution.data() + num_poses * (num_thetas + num_thetas_latent) + num_betas, solution.data() + num_poses * (num_thetas + num_thetas_latent) + num_betas + num_betas_latent);
		worker->E_shape.update_beta_latent(delta_beta_latent);
	}
}

LinearSystem BatchSolver::build_batch_system_for_online_betas(const std::vector<LinearSystem> & systems) {
	//if (show_calls) cout << "building batch system" << endl;
	size_t num_betas = worker->model->num_betas;
	size_t num_parameters = num_thetas + num_thetas_latent + num_betas + num_betas_latent;

	if (num_betas == 0) {
		cout << "batch solver can't run with num_betas == 0" << endl;
		return LinearSystem(0);
	}

	/// Independent systems
	LinearSystem batch_system = LinearSystem(num_poses * num_parameters + num_betas + num_betas_latent);
	
	for (size_t pose_index = 0; pose_index < num_poses; pose_index++) {
		size_t start_pose = pose_index * num_parameters;
		batch_system.lhs.block(start_pose, start_pose, num_parameters, num_parameters) += systems[pose_index].lhs;
		batch_system.rhs.segment(start_pose, num_parameters) += systems[pose_index].rhs;		
	}

	/// Beta common
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> F = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_poses * (num_betas + num_betas_latent), 1);
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_poses * (num_betas + num_betas_latent), num_poses * num_parameters + num_betas + num_betas_latent);

	Eigen::Matrix<float, Eigen::Dynamic, 1> beta_difference = Eigen::Matrix<float, Eigen::Dynamic, 1>::Zero(num_betas + num_betas_latent);

	for (size_t pose_index = 0; pose_index < num_poses; pose_index++) {

		/// Betas
		J.block(pose_index * (num_betas + num_betas_latent), pose_index * num_parameters + num_thetas, num_betas, num_betas) +=
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas, num_betas);
		J.block(pose_index * (num_betas + num_betas_latent), num_poses * num_parameters, num_betas, num_betas) +=
			- Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas, num_betas);

		/// Betas latent
		J.block(pose_index * (num_betas + num_betas_latent) + num_betas, pose_index * num_parameters + num_thetas + num_betas + num_thetas_latent, num_betas_latent, num_betas_latent) +=
			Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas_latent, num_betas_latent);
		J.block(pose_index * (num_betas + num_betas_latent) + num_betas, num_poses * num_parameters + num_betas, num_betas_latent, num_betas_latent) +=
			-Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas_latent, num_betas_latent);

		for (size_t i = 0; i < num_betas; i++) beta_difference[i] = betas[pose_index][i] - beta_common[i];
		for (size_t i = 0; i < num_betas_latent; i++) beta_difference[num_betas + i] = betas_latent[pose_index][i] - beta_common_latent[i];
		F.segment(pose_index * (num_betas + num_betas_latent), num_betas + num_betas_latent) += beta_difference;
	}

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Jt = J.transpose();
		
	batch_system.lhs += weight * Jt * J;
	batch_system.rhs += weight * (-Jt * F);

	//std::ofstream file_J(worker->settings->logs_path + "J.txt"); file_J << J;
	//std::ofstream file_F(worker->settings->logs_path + "F.txt"); file_F << F;
	//std::ofstream file_lhs(worker->settings->logs_path + "batch-system-lhs.txt"); file_lhs << batch_system.lhs;
	//std::ofstream file_rhs(worker->settings->logs_path + "batch-system-rhs.txt"); file_rhs << batch_system.rhs;

	cout << "F.norm() = " << F.norm() * F.norm() / weight << endl;

	return batch_system;
}

void BatchSolver::parse_parameters_for_online_betas(const VectorN & solution) {
	size_t num_betas = worker->model->num_betas;
	size_t num_parameters = num_thetas + num_thetas_latent + num_betas + num_betas_latent;

	for (size_t pose_index = 0; pose_index < num_poses; pose_index++) {

		{ /// Parse thetas 
			worker->model->update_theta(thetas[pose_index]); // because "get_updated_parameters" is using global transformation
			std::vector<float> delta_theta = vector<float>(solution.data() + pose_index * num_parameters, solution.data() + pose_index * num_parameters + num_thetas);
			thetas[pose_index] = worker->model->get_updated_parameters(thetas[pose_index], delta_theta);
		}
		
		{ /// Parse betas 
			std::vector<float> delta_beta = vector<float>(solution.data() + pose_index * num_parameters + num_thetas, solution.data() + pose_index * num_parameters + num_thetas + num_betas);
			for (size_t i = 0; i < num_betas; i++) betas[pose_index][i] += delta_beta[i];

			std::vector<float> delta_beta_latent = std::vector<float>(solution.data() + pose_index * num_parameters + num_thetas + num_betas + num_thetas_latent,
				solution.data() + pose_index * num_parameters + num_thetas + num_betas + num_thetas_latent + num_betas_latent);
			
			for (size_t i = 0; i < num_betas_latent; i++) {
				betas_latent[pose_index][i] += delta_beta_latent[i];				
			}
		}
	}

	std::vector<float> delta_beta_common = vector<float>(solution.data() + num_poses * num_parameters, solution.data() + num_poses * num_parameters + num_betas);
	for (size_t i = 0; i < num_betas; i++) beta_common[i] += delta_beta_common[i];

	std::vector<float> delta_beta_common_latent = std::vector<float>(solution.data() + num_poses * num_parameters + num_betas,
		solution.data() + num_poses * num_parameters + num_betas + num_betas_latent);
	for (size_t i = 0; i < num_betas_latent; i++) {
		beta_common_latent[i] += delta_beta_common_latent[i];		
	}
}

void BatchSolver::batch_solve_iteration() {
	if (first_batch_solve_iteration == true) {
		first_batch_solve_iteration = false;
		if (use_online_betas) {
			beta_common = betas[0];
			beta_common_latent = betas_latent[0];
		}
		else {
			beta = worker->model->get_beta();
		}			
	}
	worker->settings->solve_linear_system = false;

	std::vector<LinearSystem> systems = std::vector<LinearSystem>(num_poses, LinearSystem(worker->model->num_parameters));
	bool display_iterations = false;
	for (size_t pose_index = 0; pose_index < num_poses; pose_index++) {

		//cout << "applying the pose " << pose_index << endl;
		displayed_pose_index = pose_index;
		apply_pose(pose_index);
		worker->track(1);
		systems[pose_index] = worker->system;

		worker->offscreen_renderer.render_offscreen(true, false, false);
		update_pose_icons();

		if (display_iterations) {
			worker->offscreen_renderer.render_offscreen(true, false, false);
			worker->updateGL();
			Sleep(500);
		}

	}
	displayed_pose_index = -1;

	if (use_online_betas) {
		LinearSystem batch_system = build_batch_system_for_online_betas(systems);
		VectorN solution = energy::Energy::solve(batch_system);
		parse_parameters_for_online_betas(solution);
	}
	else {
		LinearSystem batch_system = build_batch_system_for_offline_beta(systems);
		VectorN solution = energy::Energy::solve(batch_system);
		parse_parameters_for_offline_beta(solution);
	}

	worker->settings->solve_linear_system = true;
}

void BatchSolver::batch_solve(size_t num_iterations) {
	float damping_factor = 0.95f;
	float factor = 10.0f;
	for (size_t iter = 0; iter < num_iterations; iter++) {
		cout << "iter = " << iter << endl;
		worker->E_damping.settings->beta_vs_theta_damping = factor;
		worker->batch_solver->batch_solve_iteration();
		factor = factor * damping_factor;
	}
	worker->E_damping.settings->beta_vs_theta_damping = 1.0f;
}
