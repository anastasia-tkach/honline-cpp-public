#include "tracker/HModel/KalmanFilter.h"
#include "cudax/cuda_glm.h"
#include <Eigen/Dense>
#include <fstream>

void KalmanFilter::find_elipse_parameters(const Eigen::Matrix<Scalar, 2, 2> & covariance, float & a, float & b, float & phi) {
	Eigen::EigenSolver<Eigen::Matrix<Scalar, 2, 2>> eigen_solver(covariance);
	Eigen::Matrix<Scalar, 2, 1> eigenvalues = eigen_solver.eigenvalues().real();
	Eigen::Matrix<Scalar, 2, 2> eigenvectors = eigen_solver.eigenvectors().real();

	float smallest_eigenvalue, largest_eigenvalue;
	Eigen::Matrix<Scalar, 2, 1> largest_eigenvector, smallest_eigenvector;
	if (eigenvalues(0) < eigenvalues(1)) {
		smallest_eigenvalue = eigenvalues(0);
		largest_eigenvalue = eigenvalues(1);
		smallest_eigenvector = eigenvectors.col(0);
		largest_eigenvector = eigenvectors.col(1);
	}
	else {
		smallest_eigenvalue = eigenvalues(1);
		largest_eigenvalue = eigenvalues(0);
		smallest_eigenvector = eigenvectors.col(1);
		largest_eigenvector = eigenvectors.col(0);
	}

	phi = atan2(largest_eigenvector(1), largest_eigenvector(0));
	if (phi < 0) phi = phi + 2 * M_PI;
	a = sqrt(largest_eigenvalue);
	b = sqrt(smallest_eigenvalue);

	//cout << covariance << endl;
	//cout << "phi = " << phi << endl;
	//cout << "a = " << a << endl;
	//cout << "b = " << b << endl;
}

void KalmanFilter::display_covariance(std::vector<size_t> beta_ids, std::vector<std::string> beta_names, size_t i, size_t j, cv::Mat & image) {

	Eigen::Matrix<Scalar, 2, 2> sub_measured_hessian;
	sub_measured_hessian << measured_hessian(beta_ids[i], beta_ids[i]), measured_hessian(beta_ids[i], beta_ids[j]), measured_hessian(beta_ids[j], beta_ids[i]), measured_hessian(beta_ids[j], beta_ids[j]);
	sub_measured_hessian = settings->measurement_noise_value * sub_measured_hessian + 350 * Eigen::Matrix<Scalar, 2, 2>::Identity();
	if (i == j) { sub_measured_hessian(0, 1) = 0; sub_measured_hessian(1, 0) = 0; }

	Eigen::Matrix<Scalar, 2, 2> sub_estimated_hessian;
	sub_estimated_hessian << estimated_hessian(beta_ids[i], beta_ids[i]), estimated_hessian(beta_ids[i], beta_ids[j]), estimated_hessian(beta_ids[j], beta_ids[i]), estimated_hessian(beta_ids[j], beta_ids[j]);
	sub_estimated_hessian = settings->measurement_noise_value * sub_estimated_hessian + 350 * Eigen::Matrix<Scalar, 2, 2>::Identity();
	if (i == j) { sub_estimated_hessian(0, 1) = 0; sub_estimated_hessian(1, 0) = 0; }

	float rectangle_width = image.cols / beta_ids.size();
	float rectangle_height = image.rows / beta_ids.size();
	float top = rectangle_height * i;
	float left = rectangle_width * j;
	float bottom = rectangle_height * (i + 1);
	float right = rectangle_width * (j + 1);
	cv::rectangle(image, cv::Point(top, left), cv::Point(bottom, right), dark_grey, thickness, line_type);
	cv::Point center = cv::Point((top + bottom) / 2, (left + right) / 2);

	cv::Scalar color = dark_grey;
	if (i == j) {
		cv::Size text_size = cv::getTextSize(beta_names[i], font_face, font_scale, font_thickness, &baseline);
		cv::Point text_origin(left + (rectangle_width - text_size.width) / 2, top + text_size.height + 10);
		putText(image, beta_names[i], text_origin, font_face, font_scale, dark_grey, font_thickness, line_type);
		color = crimson;
		float factor = 20;
		cv::Point shifted_center = center - cv::Point(factor * (ground_truth_values[i] - estimated_values[i]), factor * (ground_truth_values[i] - estimated_values[i]));

		cv::circle(image, shifted_center, 2, orange, 3, line_type);
		cv::circle(image, center, 2, color, 3, line_type);
	}
	float factor = 1500; cv::Size axes; float a, b, phi, rotation_angle;

	find_elipse_parameters(sub_measured_hessian.inverse(), a, b, phi);
	axes = cv::Size(factor * a, factor * b);
	rotation_angle = 360 - phi * 180 / M_PI;
	cv::ellipse(image, center, axes, rotation_angle, 0, 360, color, thickness, line_type);

	//find_elipse_parameters(sub_estimated_hessian.inverse(), a, b, phi);
	//axes = cv::Size(factor * a, factor * b);
	//rotation_angle = 360 - phi * 180 / M_PI;
	//cv::ellipse(image, center, axes, rotation_angle, 0, 360, green, thickness, line_type);
}

KalmanFilter::KalmanFilter(const std::vector<float> & beta, KalmanType kalman_filter_type, float kalman_filter_weight, bool enable_shape_dofs_blockers) {
	this->num_betas = beta.size();
	if (num_betas == 0) {
		cout << "num_betas = 0, kalman filter returns" << endl;
		return;
	}
	estimated_hessian = 0.001 * Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas, num_betas);
	estimated_values = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_betas, 1);
	measured_hessian = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_betas, num_betas);
	measured_values = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_betas, 1);
	ground_truth_values = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_betas, 1);

	this->settings->kalman_filter_type = kalman_filter_type;
	this->settings->weight = kalman_filter_weight;
	this->settings->enable_shape_dofs_blockers = enable_shape_dofs_blockers;

	for (size_t i = 0; i < num_betas; i++) {
		ground_truth_values(i) = beta[i];
		cout << beta[i] << " ";
		estimated_values(i) = beta[i];
	}
	cout << endl;
}

std::vector<float> KalmanFilter::get_estimate() {
	std::vector<float> beta = std::vector<float>(num_betas, 0);
	for (size_t i = 0; i < num_betas; i++) {
		beta[i] = estimated_values(i);
	}
	return beta;
}

void KalmanFilter::set_measured_hessian(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & measured_hessian,
	const std::vector<bool> & shape_dofs_blockers_conditions, const std::vector<std::vector<int>> & shape_dofs_blockers_beta_sets,
	const std::vector<float> & theta, const float fingers_bending_latent_variable) {
	this->measured_hessian = 1 / settings->measurement_noise_value * measured_hessian;

	if (settings->enable_shape_dofs_blockers) {
		for (size_t i = 0; i < shape_dofs_blockers_conditions.size(); i++) {
			/*if (i == 5) {
				cout << "condition = " << shape_dofs_blockers_conditions[i] << endl;
				if (shape_dofs_blockers_conditions[i] == true) {
					cout << "blocked betas = ";
					for (size_t j = 0; j < shape_dofs_blockers_beta_sets[i].size(); j++)
						cout << shape_dofs_blockers_beta_sets[i][j] << " ";
					cout << endl;
				}
			}*/
			if (shape_dofs_blockers_conditions[i] == true) {
				for (size_t j = 0; j < shape_dofs_blockers_beta_sets[i].size(); j++) {
					size_t beta_index = shape_dofs_blockers_beta_sets[i][j];
					this->measured_hessian.col(beta_index) = Eigen::VectorXf::Zero(num_betas);
					this->measured_hessian.row(beta_index) = Eigen::VectorXf::Zero(num_betas);
				}
			}
		}
	}

	for (size_t i = 0; i < measured_hessian.rows(); i++) {
		for (size_t j = 0; j < measured_hessian.cols(); j++) {
			if (i != j) {
				this->measured_hessian(i, j) = 0;
			}
		}
	}
}

void KalmanFilter::update_estimate(const std::vector<float>  & beta, const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & hessian) {
	
	/// Set measured values
	for (size_t i = 0; i < num_betas; i++) {
		measured_values(i) = beta[i];
	}

	/// Update estimated values
	{
		if (settings->kalman_filter_type == UNIFORM) {
			for (size_t i = 0; i < num_betas; i++) {
				int number_of_measurements = measured_values_history.size() + 1;
				estimated_values(i) = ((number_of_measurements - 1) * estimated_values(i) + measured_values(i)) / number_of_measurements;
			}
		}

		if (settings->kalman_filter_type == DIAGONAL) {
			for (size_t i = 0; i < num_betas; i++) {
				float x1 = estimated_values(i);	float x2 = measured_values(i);
				float h1 = estimated_hessian(i, i); float h2 = measured_hessian(i, i);
				estimated_values(i) = (h1 * x1 + h2 * x2) / (h1 + h2);
			}
		}

		if (settings->kalman_filter_type == STANDARD || settings->kalman_filter_type == HYBRID) {
			estimated_values = (estimated_hessian + measured_hessian).inverse() * (estimated_hessian * estimated_values + measured_hessian * measured_values);
		}

		if (settings->kalman_filter_type == EXTENDED) {
			estimated_values = measured_values;
		}

		estimated_hessian = estimated_hessian + measured_hessian;
	}

	/// Add system noise
	{
		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> estimated_hessian_inverse = estimated_hessian.inverse();
		estimated_hessian_inverse = estimated_hessian_inverse + settings->system_noise_value * Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas, num_betas);
		estimated_hessian = estimated_hessian_inverse.inverse();
	}

	if (record_history) {
		measured_values_history.push_back(measured_values);
		measured_certainties_history.push_back(measured_hessian.diagonal());
		estimated_values_history.push_back(estimated_values);
		estimated_certainties_history.push_back(estimated_hessian.diagonal());
	}
}

void KalmanFilter::write_estimate(size_t frame_id, std::string logs_path) {
	std::ostringstream stringstream;
	stringstream << frame_id;

	std::ofstream estimated_hessian_file(logs_path + "estimated_hessian-" + stringstream.str() + ".txt"); if (estimated_hessian_file.is_open()) estimated_hessian_file << estimated_hessian;
	std::ofstream measured_hessian_file(logs_path + "measured_hessian-" + stringstream.str() + ".txt"); if (measured_hessian_file.is_open()) measured_hessian_file << measured_hessian;
	std::ofstream estimated_values_file(logs_path + "estimated_values-" + stringstream.str() + ".txt"); if (estimated_values_file.is_open()) estimated_values_file << estimated_values;
	std::ofstream measured_values_file(logs_path + "measured_values-" + stringstream.str() + ".txt"); if (measured_values_file.is_open()) measured_values_file << measured_values;

}

void KalmanFilter::display_covariances() {
	size_t image_width = 960;
	cv::Mat image = cv::Mat::zeros(image_width, image_width, CV_8UC3);
	image = cv::Scalar(255, 255, 255);

	//std::vector<size_t> beta_ids = { 3, 4, 5, 18, 19, 20 };
	std::vector<size_t> beta_ids = { 6, 7, 8 };
	std::vector<string> beta_names = { "bottom l", "middle l ", "top l" };

	for (size_t i = 0; i < beta_ids.size(); i++) {
		for (size_t j = 0; j <= i; j++) {
			display_covariance(beta_ids, beta_names, i, j, image);
		}
	}
	cv::moveWindow("covariance", 272, 375);
	cv::imshow("covariance", image);
}

void KalmanFilter::display_estimate_history(size_t beta_id) {

	if (measured_values_history.size() == 0) return;

	size_t image_width = 960;
	size_t image_height = 960;
	cv::Mat image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
	image = cv::Scalar(255, 255, 255);
	float step = ((float)image_width) / (measured_values_history.size() - 1);
	float amplitude = 40;// ground_truth_values[beta_id];
	max_measured_certainty_so_far = max(max_measured_certainty_so_far, measured_certainties_history[measured_certainties_history.size() - 1](beta_id));
	float measured_certainty_scaling = image_height / 4 / max_measured_certainty_so_far;
	max_estimated_certainty_so_far = estimated_certainties_history[estimated_certainties_history.size() - 1](beta_id);
	float estimated_certainty_scaling = image_height / 4 / max_estimated_certainty_so_far;

	cv::Point start, end; float start_value, end_value, stard_certainty, end_certainty;
	for (size_t t = 0; t < measured_values_history.size() - 1; t++) {

		/// Measured value
		start_value = measured_values_history[t](beta_id) - ground_truth_values[beta_id];
		end_value = measured_values_history[t + 1](beta_id) - ground_truth_values[beta_id];
		start = cv::Point(t * step, image_height / 2 - start_value * image_height / amplitude);
		end = cv::Point((t + 1) * step, image_height / 2 - end_value * image_height / amplitude);
		cv::line(image, start, end, crimson, thickness, line_type);

		/// Estimated value
		start_value = estimated_values_history[t](beta_id) - ground_truth_values[beta_id];
		end_value = estimated_values_history[t + 1](beta_id) - ground_truth_values[beta_id];
		start = cv::Point(t * step, image_height / 2 - start_value * image_height / amplitude);
		end = cv::Point((t + 1) * step, image_height / 2 - end_value * image_height / amplitude);
		cv::line(image, start, end, green, thickness, line_type);

		/// Measured certainty
		stard_certainty = measured_certainties_history[t](beta_id);
		end_certainty = measured_certainties_history[t + 1](beta_id);
		start = cv::Point(t * step, image_height - stard_certainty * measured_certainty_scaling);
		end = cv::Point((t + 1) * step, image_height - end_certainty * measured_certainty_scaling);
		cv::line(image, start, end, pink, thickness, line_type);

		/// Estimated certainty
		stard_certainty = estimated_certainties_history[t](beta_id);
		end_certainty = estimated_certainties_history[t + 1](beta_id);
		start = cv::Point(t * step, image_height - stard_certainty * estimated_certainty_scaling);
		end = cv::Point((t + 1) * step, image_height - end_certainty * estimated_certainty_scaling);
		cv::line(image, start, end, light_green, thickness, line_type);
	}
	cv::line(image, cv::Point(0, image_height / 2), cv::Point(image_width, image_height / 2), dark_grey, thickness, line_type);

	std::ostringstream stringstream; cv::Point text_origin; cv::Size text_size;
	stringstream << "true = " << ground_truth_values[beta_id]
		<< ", measured = " << measured_values_history[measured_values_history.size() - 1](beta_id)
		<< ", estimated = " << estimated_values_history[measured_values_history.size() - 1](beta_id);
	text_size = cv::getTextSize(stringstream.str(), font_face, font_scale, font_thickness, &baseline);
	text_origin = cv::Point(image_width / 2 - text_size.width / 2, text_size.height + 10);
	putText(image, stringstream.str(), text_origin, font_face, font_scale, dark_grey, font_thickness, line_type);

	stringstream.str(std::string());
	stringstream << "kalman hessian = " << estimated_certainties_history[estimated_certainties_history.size() - 1][beta_id]
		<< ", measured hessian = " << measured_certainties_history[measured_certainties_history.size() - 1][beta_id];

	text_size = cv::getTextSize(stringstream.str(), font_face, font_scale, font_thickness, &baseline);
	text_origin = cv::Point(image_width / 2 - text_size.width / 2, 2 * (text_size.height + 10));
	putText(image, stringstream.str(), text_origin, font_face, font_scale, dark_grey, font_thickness, line_type);

	cv::moveWindow("history", 272, 375);
	cv::imshow("history", image);
}

void KalmanFilter::display_several_estimate_histories(vector<size_t> beta_ids) {

	if (measured_values_history.size() == 0) return;

	size_t image_width = 960;
	size_t image_height = 960;
	cv::Mat image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
	image = cv::Scalar(255, 255, 255);
	float step = ((float)image_width) / (measured_values_history.size() - 1);
	std::vector<cv::Scalar> colors = { crimson, green, pink, light_green, orange, dark_grey };

	float max_certainty = 0;
	for (size_t i = 0; i < beta_ids.size(); i++) {
		float current_certainty = estimated_certainties_history[estimated_certainties_history.size() - 1](beta_ids[i]);
		if (current_certainty > max_certainty) max_certainty = current_certainty;
	}
	float certainty_scaling = image_height / max_certainty / 2;
	float amplitude = 40;

	cv::Point start, end; float start_value, end_value, stard_certainty, end_certainty;
	for (size_t i = 0; i < beta_ids.size(); i++) {
		for (size_t t = 0; t < estimated_certainties_history.size() - 1; t++) {

			/// Estimated values
			start_value = estimated_values_history[t](beta_ids[i]) - ground_truth_values[beta_ids[i]];
			end_value = estimated_values_history[t + 1](beta_ids[i]) - ground_truth_values[beta_ids[i]];
			start = cv::Point(t * step, image_height / 3 - start_value * image_height / amplitude);
			end = cv::Point((t + 1) * step, image_height / 3 - end_value * image_height / amplitude);
			cv::line(image, start, end, colors[i], thickness, line_type);

			/// Estimated certainties
			stard_certainty = estimated_certainties_history[t](beta_ids[i]);
			end_certainty = estimated_certainties_history[t + 1](beta_ids[i]);
			start = cv::Point(t * step, image_height - stard_certainty * certainty_scaling);
			end = cv::Point((t + 1) * step, image_height - end_certainty * certainty_scaling);
			cv::line(image, start, end, colors[i], thickness, line_type);
		}

		std::ostringstream stringstream; cv::Point text_origin; cv::Size text_size;
		stringstream << "beta " << beta_ids[i] << ", certainty = " << estimated_certainties_history[estimated_certainties_history.size() - 1](beta_ids[i]);
		text_size = cv::getTextSize(stringstream.str(), font_face, font_scale, font_thickness, &baseline);
		text_origin = cv::Point(15, (i + 1) * (text_size.height + 10));
		putText(image, stringstream.str(), text_origin, font_face, font_scale, colors[i], font_thickness, line_type);
	}

	cv::line(image, cv::Point(0, image_height / 3), cv::Point(image_width, image_height / 3), dark_grey, thickness, line_type);

	cv::moveWindow("history", 272, 375);
	cv::imshow("history", image);
}

void KalmanFilter::display_several_measure_histories(vector<size_t> beta_ids) {

	if (measured_values_history.size() == 0) return;

	size_t image_width = 960;
	size_t image_height = 960;
	cv::Mat image = cv::Mat::zeros(image_height, image_width, CV_8UC3);
	image = cv::Scalar(255, 255, 255);
	float step = ((float)image_width) / (measured_values_history.size() - 1);
	std::vector<cv::Scalar> colors = { pink, green, light_green, crimson, orange, dark_grey };

	float max_certainty = 0;
	for (size_t i = 0; i < beta_ids.size(); i++) {
		float current_certainty = measured_certainties_history[measured_certainties_history.size() - 1](beta_ids[i]);
		if (current_certainty > max_certainty) max_certainty = current_certainty;
	}
	float certainty_scaling = image_height / max_certainty / 4;
	float amplitude = 40;

	cv::Point start, end; float start_value, end_value, stard_certainty, end_certainty;
	for (size_t i = 0; i < beta_ids.size(); i++) {
		for (size_t t = 0; t < measured_certainties_history.size() - 1; t++) {

			/// Measured values
			start_value = measured_values_history[t](beta_ids[i]) - ground_truth_values[beta_ids[i]];
			end_value = measured_values_history[t + 1](beta_ids[i]) - ground_truth_values[beta_ids[i]];
			start = cv::Point(t * step, image_height / 3 - start_value * image_height / amplitude);
			end = cv::Point((t + 1) * step, image_height / 3 - end_value * image_height / amplitude);
			cv::line(image, start, end, colors[i], thickness, line_type);

			/// Measured certainties
			/*stard_certainty = measured_certainties_history[t](beta_ids[i]);
			end_certainty = measured_certainties_history[t + 1](beta_ids[i]);
			start = cv::Point(t * step, image_height - stard_certainty * certainty_scaling);
			end = cv::Point((t + 1) * step, image_height - end_certainty * certainty_scaling);
			cv::line(image, start, end, colors[i], thickness, line_type);*/
		}

		std::ostringstream stringstream; cv::Point text_origin; cv::Size text_size;
		stringstream << "beta " << beta_ids[i] << ", error = " << measured_values_history[measured_values_history.size() - 1](beta_ids[i]) - ground_truth_values[beta_ids[i]];
		text_size = cv::getTextSize(stringstream.str(), font_face, font_scale, font_thickness, &baseline);
		text_origin = cv::Point(15, (i + 1) * (text_size.height + 10));
		putText(image, stringstream.str(), text_origin, font_face, font_scale, colors[i], font_thickness, line_type);
	}

	cv::line(image, cv::Point(0, image_height / 3), cv::Point(image_width, image_height / 3), dark_grey, thickness, line_type);
	cv::line(image, cv::Point(0, image_height / 3 + 5.0f * image_height / amplitude), cv::Point(image_width, image_height / 3 + 5.0f * image_height / amplitude), light_grey, thickness, line_type);
	cv::line(image, cv::Point(0, image_height / 3 - 5.0f * image_height / amplitude), cv::Point(image_width, image_height / 3 - 5.0f * image_height / amplitude), light_grey, thickness, line_type);

	cv::moveWindow("history", 272, 375);
	cv::imshow("history", image);
}

void KalmanFilter::track(LinearSystem& system, const std::vector<Scalar>& beta) {

	for (size_t i = 0; i < num_betas; i++) measured_values(i) = beta[i];

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> LHS;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1>  rhs;

	if (settings->kalman_filter_type == EXTENDED) {
		rhs = estimated_hessian * (estimated_values - measured_values);
		LHS = estimated_hessian;
	}
	if (settings->kalman_filter_type == HYBRID) {
		//rhs = estimated_values - measured_values;
		//LHS = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(num_betas, num_betas);
		rhs = estimated_hessian * (estimated_values - measured_values);
		LHS = estimated_hessian;
	}
	
	system.lhs.block(num_thetas, num_thetas, num_betas, num_betas).noalias() += settings->weight * LHS.transpose() * LHS;
	system.rhs.segment(num_thetas, num_betas).noalias() += settings->weight * LHS.transpose() * rhs;
}
