#pragma once
#include "tracker/Energy/ShapeSpace.h"
#include <vector>
#include <fstream> 

void energy::ShapeSpace::init(Model * model, PoseSpace * E_pose, std::string logs_path) {
	beta_template = model->beta_template;
	beta_latent = std::vector<float>(num_betas_latent, 1.0f);
	this->enable_pose_prior = E_pose->settings->enable_joint_pca || E_pose->settings->enable_split_pca;
	this->model = model;
	this->E_pose = E_pose;
	this->logs_path = logs_path;

	////
	settings->num_additional_constraints = 17;
	settings->weight_additional_constraints = 7;

	settings->num_semantic_limits = model->semantic_limits.size();
	settings->weight_semantic_limits = 3000;

	settings->num_shape_dofs_blockers = 5 + 10 + 1;
	settings->weight_shape_dofs_blockers = 3000;

	shape_dofs_blockers_initialization_indicator = std::vector<bool>(settings->num_shape_dofs_blockers, false);
	shape_dofs_blockers_values = std::vector<float>(5 + 10 * 2 + 1, 0.0f);
	shape_dofs_blockers_beta_indices = { /// Edit KalmanFilter::set_measured_hessian()
		(int)model->shape_dofs_name_to_id_map["thumb_base_x"],
		(int)model->shape_dofs_name_to_id_map["index_base_y"],
		(int)model->shape_dofs_name_to_id_map["middle_base_y"],
		(int)model->shape_dofs_name_to_id_map["ring_base_y"],
		(int)model->shape_dofs_name_to_id_map["pinky_base_y"],

		(int)model->shape_dofs_name_to_id_map["thumb_bottom_length"], (int)model->shape_dofs_name_to_id_map["thumb_middle_length"],
		(int)model->shape_dofs_name_to_id_map["thumb_middle_length"], (int)model->shape_dofs_name_to_id_map["thumb_top_length"],

		(int)model->shape_dofs_name_to_id_map["index_bottom_length"], (int)model->shape_dofs_name_to_id_map["index_middle_length"],
		(int)model->shape_dofs_name_to_id_map["index_middle_length"], (int)model->shape_dofs_name_to_id_map["index_top_length"],

		(int)model->shape_dofs_name_to_id_map["middle_bottom_length"], (int)model->shape_dofs_name_to_id_map["middle_middle_length"],
		(int)model->shape_dofs_name_to_id_map["middle_middle_length"], (int)model->shape_dofs_name_to_id_map["middle_top_length"],

		(int)model->shape_dofs_name_to_id_map["ring_bottom_length"], (int)model->shape_dofs_name_to_id_map["ring_middle_length"],
		(int)model->shape_dofs_name_to_id_map["ring_middle_length"], (int)model->shape_dofs_name_to_id_map["ring_top_length"],

		(int)model->shape_dofs_name_to_id_map["pinky_bottom_length"], (int)model->shape_dofs_name_to_id_map["pinky_middle_length"],
		(int)model->shape_dofs_name_to_id_map["pinky_middle_length"], (int)model->shape_dofs_name_to_id_map["pinky_top_length"],
	};

	shape_dofs_blockers_thresholds = {
		0.0f,
		0.0f, 0.0f, 0.0f, 0.0f, // not used;
		-0.3f, -0.7f, // thumb bottom and middle
		0.5f, 0.5f, // index bottom and middle	
		0.5f, 0.5f, // middle bottom and middle	
		0.5f, 0.5f, // ring bottom and middle	
		0.5f, 0.5f // pinky bottom and middle
	};

	shape_dofs_blockers_theta_indices = {
		10, // thumb base flexion
		-1, -1, -1, -1, // not used
		11, 12, // thumb bottom and middle
		15, 16, // index bottom and middle		
		19, 20, // middle bottom and middle	
		23, 24, // ring bottom and middle	
		27, 28, // pinky bottom and middle	
	};

	/// For passing information to kalman
	shape_dofs_blockers_conditions = std::vector<bool>(settings->num_shape_dofs_blockers - 1, false);
	shape_dofs_blockers_beta_sets = {
		/*std::vector<int>{(int)model->shape_dofs_name_to_id_map["thumb_base_x"], (int)model->shape_dofs_name_to_id_map["thumb_bottom_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["index_base_y"], (int)model->shape_dofs_name_to_id_map["index_bottom_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["middle_base_y"], (int)model->shape_dofs_name_to_id_map["middle_bottom_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["ring_base_y"], (int)model->shape_dofs_name_to_id_map["ring_bottom_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["pinky_base_y"], (int)model->shape_dofs_name_to_id_map["pinky_bottom_length"]},*/

		std::vector<int>{(int)model->shape_dofs_name_to_id_map["thumb_base_x"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["index_base_y"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["middle_base_y"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["ring_base_y"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["pinky_base_y"]},

		std::vector<int>{(int)model->shape_dofs_name_to_id_map["thumb_bottom_length"], (int)model->shape_dofs_name_to_id_map["thumb_middle_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["thumb_middle_length"], (int)model->shape_dofs_name_to_id_map["thumb_top_length"], (int)model->shape_dofs_name_to_id_map["thumb_additional_y"]},

		std::vector<int>{(int)model->shape_dofs_name_to_id_map["index_bottom_length"], (int)model->shape_dofs_name_to_id_map["index_middle_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["index_middle_length"], (int)model->shape_dofs_name_to_id_map["index_top_length"]},

		std::vector<int>{(int)model->shape_dofs_name_to_id_map["middle_bottom_length"], (int)model->shape_dofs_name_to_id_map["middle_middle_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["middle_middle_length"], (int)model->shape_dofs_name_to_id_map["middle_top_length"]},

		std::vector<int>{(int)model->shape_dofs_name_to_id_map["ring_bottom_length"], (int)model->shape_dofs_name_to_id_map["ring_middle_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["ring_middle_length"], (int)model->shape_dofs_name_to_id_map["ring_top_length"]},

		std::vector<int>{(int)model->shape_dofs_name_to_id_map["pinky_bottom_length"], (int)model->shape_dofs_name_to_id_map["pinky_middle_length"]},
		std::vector<int>{(int)model->shape_dofs_name_to_id_map["pinky_middle_length"], (int)model->shape_dofs_name_to_id_map["pinky_top_length"]},
	};

}

void energy::ShapeSpace::update_beta_template(const std::vector<float> & new_beta_template) {
	beta_template.resize(model->num_betas);
	std::copy(new_beta_template.begin(), new_beta_template.begin() + model->num_betas, beta_template.begin());
}

void energy::ShapeSpace::update_beta_latent(const std::vector<float> & delta_beta_latent) {
	for (size_t i = 0; i < num_betas_latent; i++) {
		beta_latent[i] += delta_beta_latent[i];
	}
}

void energy::ShapeSpace::update_beta_latent(const VectorN & solution) {
	if (_settings.enable_shape_prior == false) return;

	for (size_t i = 0; i < num_betas_latent; i++) {
		beta_latent[i] += solution[i + model->num_parameters + num_thetas_latent];
	}
}

void energy::ShapeSpace::print_beta_latent() {
	for (size_t i = 0; i < num_betas_latent; i++) {
		cout << beta_latent[i] << " ";
	}
	cout << endl;
}

std::vector<float> energy::ShapeSpace::get_beta_latent() {
	return beta_latent;
}

Eigen::Matrix<double, Eigen::Dynamic, 1> energy::ShapeSpace::compute_objective_double(const std::vector<double> & beta_double, int beta_index, const std::vector<double> & beta_latent_double, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & F) {
	size_t num_betas = model->num_betas;
	size_t num_others = settings->num_additional_constraints + settings->num_semantic_limits + settings->num_shape_dofs_blockers;
	int start_radii = model->calibration_type_to_num_betas_map[FINGERS_AND_PALM];

	Eigen::Matrix<double, Eigen::Dynamic, 1> F_double = Eigen::Matrix<double, Eigen::Dynamic, 1>::Zero(num_betas + num_others, 1);

	/// Shape priors
	for (size_t i = 0; i < num_betas; i++) {
		BetaType beta_type = model->shape_dofs[i].type;

		if (model->calibration_stage == D1 || model->calibration_stage == DEFAULT)
			F_double(i) = +sqrt(settings->weight_uniform) * (beta_double[i] - beta_latent_double[0] * beta_template[i]);
		else
			F_double(i) = +sqrt(settings->weight_uniform) * (beta_double[i] - beta_latent[0] * beta_template[i]);

		if (beta_type == BetaType::FINGER_BASE_X) {
			if (model->calibration_stage != DN)
				F_double(i) += sqrt(settings->weight_palm_width) * (beta_double[i] - beta_latent_double[1] * beta_template[i]);
			else
				F_double(i) += sqrt(settings->weight_palm_width) * (beta_double[i] - beta_latent[1] * beta_template[i]);
		}
		if (beta_type == BetaType::FINGER_BASE_Y) {
			if (model->calibration_stage != DN)
				F_double(i) += sqrt(settings->weight_palm_height) * (beta_double[i] - beta_latent_double[2] * beta_template[i]);
			else
				F_double(i) += sqrt(settings->weight_palm_height) * (beta_double[i] - beta_latent[2] * beta_template[i]);
		}
		if (beta_type == BetaType::RADIUS && ((i != start_radii + 0) && (i != start_radii + 4) && (i != start_radii + 8) && (i != start_radii + 12) && (i != start_radii + 16) && i < start_radii + 20)) {
			if (model->calibration_stage != DN)
				F_double(i) += sqrt(settings->weight_radii) * (beta_double[i] - beta_latent_double[3] * beta_template[i]);
			else
				F_double(i) += sqrt(settings->weight_radii) * (beta_double[i] - beta_latent[3] * beta_template[i]);
		}
		if (beta_type == BetaType::RADIUS && ((i == start_radii + 0) || (i == start_radii + 4) || (i == start_radii + 8) || (i == start_radii + 12) || (i == start_radii + 16) || (i >= start_radii + 20))) {
			if (model->calibration_stage != DN)
				F_double(i) += sqrt(settings->weight_radii) * (beta_double[i] - beta_latent_double[4] * beta_template[i]);
			else
				F_double(i) += sqrt(settings->weight_radii) * (beta_double[i] - beta_latent[4] * beta_template[i]);
		}
	}

	/// Additional constraints
	if (settings->enable_additional_constraints) {
		F_double(num_betas + 0) = settings->weight_additional_constraints * (beta_latent_double[4] - (double)beta_latent[3]);
		//F_double(num_betas + 1) = 10 * settings->weight_additional_constraints * (beta_latent_double[3] - (double)beta_latent[1]);
	}

	/// Semantic limits
	if (settings->enable_semantic_limits) {
		for (size_t i = 0; i < model->semantic_limits.size(); i++) {
			double smaller_double = 0; double bigger_double = 0;
			for (size_t j = 0; j < model->semantic_limits[i].smaller.size(); j++) {
				if (std::get<0>(model->semantic_limits[i].smaller[j]) >= 0) {
					if (std::get<2>(model->semantic_limits[i].smaller[j]) == true) {
						smaller_double += beta_double[std::get<0>(model->semantic_limits[i].smaller[j])] * std::get<1>(model->semantic_limits[i].smaller[j]);
					}
					else {
						smaller_double += (double)model->beta[std::get<0>(model->semantic_limits[i].smaller[j])] * std::get<1>(model->semantic_limits[i].smaller[j]);
					}
				}
				else {
					smaller_double += (double)std::get<1>(model->semantic_limits[i].smaller[j]);
				}
			}
			for (size_t j = 0; j < model->semantic_limits[i].bigger.size(); j++) {
				if (std::get<0>(model->semantic_limits[i].bigger[j]) >= 0) {
					if (std::get<2>(model->semantic_limits[i].bigger[j]) == true) {
						bigger_double += beta_double[std::get<0>(model->semantic_limits[i].bigger[j])] * std::get<1>(model->semantic_limits[i].bigger[j]);
					}
					else {
						bigger_double += (double)model->beta[std::get<0>(model->semantic_limits[i].bigger[j])] * std::get<1>(model->semantic_limits[i].bigger[j]);
					}
				}
				else {
					bigger_double += (double)std::get<1>(model->semantic_limits[i].bigger[j]);
				}
			}

			if (F(num_betas + settings->num_additional_constraints + i) > 0) {
				//if (smaller_double > bigger_double) {
				F_double(num_betas + settings->num_additional_constraints + i) = settings->weight_semantic_limits * (smaller_double - bigger_double);

				/*bool changing_current_beta = false;
				for (size_t j = 0; j < model->semantic_limits[i].smaller.size(); j++) {
					if (model->semantic_limits[i].smaller[j] == beta_index) changing_current_beta = true;
				}
				for (size_t j = 0; j < model->semantic_limits[i].bigger.size(); j++) {
					if (model->semantic_limits[i].bigger[j] == beta_index) changing_current_beta = true;
				}
				if (changing_current_beta) {
					cout << "numerical[" << beta_index << "] = " << smaller_double - bigger_double << endl;
				}*/
			}
		}
	}

	/// Shape dofs blockers
	if (settings->enable_shape_dofs_blockers) {
		for (size_t constraint_index = 0; constraint_index < 5; constraint_index++) {
			if (constraint_index == 6) continue;

			if ((constraint_index == 0 && model->theta[shape_dofs_blockers_theta_indices[constraint_index]] <= shape_dofs_blockers_thresholds[constraint_index]) ||
				(constraint_index > 0 && E_pose->fingers_bending_latent_variable < -1.1)) {
				shape_dofs_blockers_values[constraint_index] = beta_double[shape_dofs_blockers_beta_indices[constraint_index]];
				shape_dofs_blockers_initialization_indicator[constraint_index] = true;
			}
			else {
				if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
					F_double(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (beta_double[shape_dofs_blockers_beta_indices[constraint_index]] - shape_dofs_blockers_values[constraint_index]);
				}
				else {
					F_double(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (beta_double[shape_dofs_blockers_beta_indices[constraint_index]] - beta_latent[0] * beta_template[shape_dofs_blockers_beta_indices[constraint_index]]);
				}
			}
		}

		for (size_t constraint_index = 5; constraint_index < 5 + 10; constraint_index++) {
			size_t i1 = 2 * (constraint_index - 5) + 5;
			size_t i2 = 2 * (constraint_index - 5) + 5 + 1;
			if ((constraint_index >= 7 && model->theta[shape_dofs_blockers_theta_indices[constraint_index]] >= shape_dofs_blockers_thresholds[constraint_index]) ||
				(constraint_index < 7 && model->theta[shape_dofs_blockers_theta_indices[constraint_index]] <= shape_dofs_blockers_thresholds[constraint_index])) {
				/// Shape dofs are not blocked
			}
			else {
				if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
					F_double(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (
							beta_double[shape_dofs_blockers_beta_indices[i1]] -
							beta_double[shape_dofs_blockers_beta_indices[i2]] * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2]
							);
				}
				else {
					F_double(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (
							beta_double[shape_dofs_blockers_beta_indices[i1]] -
							beta_double[shape_dofs_blockers_beta_indices[i2]] * beta_template[shape_dofs_blockers_beta_indices[i1]] / beta_template[shape_dofs_blockers_beta_indices[i2]]
							);
				}
			}
		}
		///
		size_t constraint_index = 6;
		size_t i1 = 2 * (constraint_index - 5) + 5;
		size_t i2 = 2 * (constraint_index - 5) + 5 + 1;
		size_t beta_i3 = model->shape_dofs_name_to_id_map["thumb_additional_y"];
		if (model->theta[shape_dofs_blockers_theta_indices[constraint_index]] <= shape_dofs_blockers_thresholds[constraint_index]) {
			/// Shape dofs are not blocked
		}
		else {
			shape_dofs_blockers_conditions[constraint_index] = true;
			if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
				F_double(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
					settings->weight_shape_dofs_blockers * (
						beta_double[shape_dofs_blockers_beta_indices[i1]] -
						(beta_double[shape_dofs_blockers_beta_indices[i2]] + beta_double[beta_i3]) * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2]
						);
			}
			else {
				float template_top_length = beta_template[shape_dofs_blockers_beta_indices[i2]] + beta_template[beta_i3];
				F_double(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
					settings->weight_shape_dofs_blockers * (
						beta_double[shape_dofs_blockers_beta_indices[i1]] -
						(beta_double[shape_dofs_blockers_beta_indices[i2]] + beta_double[beta_i3]) * beta_template[shape_dofs_blockers_beta_indices[i1]] / template_top_length
						);				
			}
		}
		///
	}

	return F_double;
}

void energy::ShapeSpace::verify_jacobian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & F, const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & J) {
	double epsilon = 5 * 1e-9;
	size_t num_betas = model->num_betas;
	size_t num_others = settings->num_additional_constraints + settings->num_semantic_limits + settings->num_shape_dofs_blockers;

	std::vector<double> beta_double = std::vector<double>(model->beta.begin(), model->beta.end());
	std::vector<double> beta_latent_double = std::vector<double>(beta_latent.begin(), beta_latent.end());

	Eigen::Matrix<double, Eigen::Dynamic, 1> F_double = compute_objective_double(beta_double, -1, beta_latent_double, F);
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_double = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_betas + num_others, num_betas + num_betas_latent);

	for (size_t i = 0; i < beta_double.size(); i++) {
		beta_double[i] += epsilon;
		Eigen::Matrix<double, Eigen::Dynamic, 1> F_plus = compute_objective_double(beta_double, i, beta_latent_double, F);
		beta_double[i] -= 2 * epsilon;
		Eigen::Matrix<double, Eigen::Dynamic, 1> F_minus = compute_objective_double(beta_double, i, beta_latent_double, F);
		beta_double[i] += epsilon;
		J_double.col(i) = (F_plus - F_minus) / 2 / epsilon;

	}

	for (size_t i = 0; i < beta_latent_double.size(); i++) {
		beta_latent_double[i] += epsilon;
		Eigen::Matrix<double, Eigen::Dynamic, 1> F_plus = compute_objective_double(beta_double, num_betas + i, beta_latent_double, F);
		beta_latent_double[i] -= 2 * epsilon;
		Eigen::Matrix<double, Eigen::Dynamic, 1> F_minus = compute_objective_double(beta_double, num_betas + i, beta_latent_double, F);
		beta_latent_double[i] += epsilon;
		J_double.col(i + num_betas) = (F_plus - F_minus) / 2 / epsilon;
	}

	double F_difference = 0;
	for (size_t i = 0; i < F.size(); i++) {
		double difference = abs(F(i) - F_double(i));
		if (difference > F_difference)
			F_difference = difference;
	}
	cout << "F_difference = " << F_difference << ", ";
	double J_difference = 0;
	for (size_t i = 0; i < J.rows(); i++) {
		for (size_t j = 0; j < J.cols(); j++) {
			double difference = abs(J(i, j) - J_double(i, j));
			if (difference > J_difference)
				J_difference = difference;
		}
	}
	cout << "J_difference = " << J_difference << endl;

	if (F_difference > 0.5 || J_difference > 0.5) {
		cout << "F_DIFFERENCE = " << F_difference << ", J_DIFFERENCE = " << J_difference << endl;
		std::ofstream J_file(logs_path + "J.txt"); if (J_file.is_open()) J_file << J;
		std::ofstream F_file(logs_path + "F.txt"); if (F_file.is_open()) F_file << F;
		std::ofstream J_double_file(logs_path + "J-double.txt"); if (J_double_file.is_open()) J_double_file << J_double;
		std::ofstream F_double_file(logs_path + "F-double.txt"); if (F_file.is_open()) F_double_file << F_double;
	}
}

void energy::ShapeSpace::track(LinearSystem & system, std::vector<float> beta, int iter) {
	if (!settings->enable_shape_prior || model->calibration_type == NONE) return;

	size_t num_betas = model->num_betas;

	size_t num_others = settings->num_additional_constraints + settings->num_semantic_limits + settings->num_shape_dofs_blockers;

	int start_radii = model->calibration_type_to_num_betas_map[FINGERS_AND_PALM];

	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> F = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_betas + num_others, 1);
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_betas + num_others, num_betas + num_betas_latent);

	/// Shape priors
	for (size_t i = 0; i < num_betas; i++) {
		BetaType beta_type = model->shape_dofs[i].type;

		F(i) = +sqrt(settings->weight_uniform) * (beta[i] - beta_latent[0] * beta_template[i]);
		J(i, i) = +sqrt(settings->weight_uniform);
		if (model->calibration_stage == D1 || model->calibration_stage == DEFAULT) {
			J(i, num_betas + 0) = +sqrt(settings->weight_uniform) *(-beta_template[i]);
		}

		if (beta_type == BetaType::FINGER_BASE_X) {
			F(i) += sqrt(settings->weight_palm_width) * (beta[i] - beta_latent[1] * beta_template[i]);
			J(i, i) += sqrt(settings->weight_palm_width);
			if (model->calibration_stage != DN) {
				J(i, num_betas + 1) += sqrt(settings->weight_palm_width) * (-beta_template[i]);
			}
		}
		if (beta_type == BetaType::FINGER_BASE_Y) {
			F(i) += sqrt(settings->weight_palm_height) * (beta[i] - beta_latent[2] * beta_template[i]);
			J(i, i) += sqrt(settings->weight_palm_height);
			if (model->calibration_stage != DN) {
				J(i, num_betas + 2) += sqrt(settings->weight_palm_height) * (-beta_template[i]);
			}
		}
		if (beta_type == BetaType::RADIUS) {
			F(i) += sqrt(settings->weight_radii) * (beta[i] - beta_latent[3] * beta_template[i]);
			J(i, i) += sqrt(settings->weight_radii);
			if (model->calibration_stage != DN) {
				J(i, num_betas + 3) += sqrt(settings->weight_radii) * (-beta_template[i]);
			}
		}
		/*if (beta_type == BetaType::RADIUS && ((i != start_radii + 0) && (i != start_radii + 4) && (i != start_radii + 8) && (i != start_radii + 12) && (i != start_radii + 16) && (i < start_radii + 20))) {
			F(i) += sqrt(settings->weight_radii) * (beta[i] - beta_latent[3] * beta_template[i]);
			J(i, i) += sqrt(settings->weight_radii);
			if (model->calibration_stage != DN) {
				J(i, num_betas + 3) += sqrt(settings->weight_radii) * (-beta_template[i]);
			}			
		}
		if (beta_type == BetaType::RADIUS && ((i == start_radii + 0) || (i == start_radii + 4) || (i == start_radii + 8) || (i == start_radii + 12) || (i == start_radii + 16) || (i >= start_radii + 20))) {
			F(i) += sqrt(settings->weight_radii) * (beta[i] - beta_latent[4] * beta_template[i]);
			J(i, i) += sqrt(settings->weight_radii);
			if (model->calibration_stage != DN) {
				J(i, num_betas + 4) += sqrt(settings->weight_radii) * (-beta_template[i]);
			}
		}*/

	}

	/// Additional constraints
	if (settings->enable_additional_constraints) {

		F(num_betas + 0) = settings->weight_additional_constraints * 1; J(num_betas + 0, 1) = settings->weight_additional_constraints;

		F(num_betas + 1) = settings->weight_additional_constraints * 1;	J(num_betas + 1, 4) = settings->weight_additional_constraints;
		F(num_betas + 2) = settings->weight_additional_constraints * 1; J(num_betas + 2, 5) = settings->weight_additional_constraints;
		F(num_betas + 3) = settings->weight_additional_constraints * 1; J(num_betas + 3, 18) = -settings->weight_additional_constraints;

		//F(num_betas + 4) = settings->weight_additional_constraints * 0.5; J(num_betas + 4, 6) = -settings->weight_additional_constraints;
		F(num_betas + 5) = settings->weight_additional_constraints * 1;	J(num_betas + 5, 7) = settings->weight_additional_constraints;
		F(num_betas + 6) = settings->weight_additional_constraints * 1; J(num_betas + 6, 8) = settings->weight_additional_constraints;
		//F(num_betas + 7) = settings->weight_additional_constraints * 1; J(num_betas + 7, 22) = settings->weight_additional_constraints;

		//F(num_betas + 8) = settings->weight_additional_constraints * 1; J(num_betas + 8, 9) = -settings->weight_additional_constraints;
		F(num_betas + 9) = settings->weight_additional_constraints * 1;	J(num_betas + 9, 10) = settings->weight_additional_constraints;
		F(num_betas + 10) = settings->weight_additional_constraints * 1; J(num_betas + 10, 11) = settings->weight_additional_constraints;
		//F(num_betas + 11) = settings->weight_additional_constraints * 1; J(num_betas + 11, 25) = settings->weight_additional_constraints;

		//F(num_betas + 12) = settings->weight_additional_constraints * 3; J(num_betas + 12, 12) = -settings->weight_additional_constraints;
		F(num_betas + 13) = settings->weight_additional_constraints * 1; J(num_betas + 13, 13) = settings->weight_additional_constraints;
		F(num_betas + 14) = settings->weight_additional_constraints * 1; J(num_betas + 14, 14) = settings->weight_additional_constraints;
		//F(num_betas + 15) = settings->weight_additional_constraints * 1; J(num_betas + 15, 28) = settings->weight_additional_constraints;

		F(num_betas + 16) = settings->weight_additional_constraints * 4; J(num_betas + 16, 43) = settings->weight_additional_constraints * 4;
	}

	/// Semantic limits
	if (settings->enable_semantic_limits) {
		for (size_t i = 0; i < model->semantic_limits.size(); i++) {
			double smaller = 0; double bigger = 0;
			for (size_t j = 0; j < model->semantic_limits[i].smaller.size(); j++) {
				if (std::get<0>(model->semantic_limits[i].smaller[j]) >= 0) {
					smaller += (double)beta[std::get<0>(model->semantic_limits[i].smaller[j])] * std::get<1>(model->semantic_limits[i].smaller[j]);
				}
				else {
					smaller += (double)std::get<1>(model->semantic_limits[i].smaller[j]);
				}
			}
			for (size_t j = 0; j < model->semantic_limits[i].bigger.size(); j++) {
				if (std::get<0>(model->semantic_limits[i].bigger[j]) >= 0) {
					bigger += (double)beta[std::get<0>(model->semantic_limits[i].bigger[j])] * std::get<1>(model->semantic_limits[i].bigger[j]);
				}
				else {
					bigger += (double)std::get<1>(model->semantic_limits[i].bigger[j]);
				}
			}
			//if (i == model->semantic_limits.size() - 1) cout << "smaller = " << smaller << ", bigger = " << bigger << endl;
			if (smaller > bigger) {
				F(num_betas + settings->num_additional_constraints + i) = settings->weight_semantic_limits * (smaller - bigger);
				for (size_t j = 0; j < model->semantic_limits[i].smaller.size(); j++) {
					if (std::get<0>(model->semantic_limits[i].smaller[j]) < 0) continue;
					BetaType beta_type = model->shape_dofs[std::get<0>(model->semantic_limits[i].smaller[j])].type;
					if (std::get<2>(model->semantic_limits[i].smaller[j]) == false) continue;
					J(num_betas + settings->num_additional_constraints + i, std::get<0>(model->semantic_limits[i].smaller[j])) += settings->weight_semantic_limits * std::get<1>(model->semantic_limits[i].smaller[j]);
				}
				for (size_t j = 0; j < model->semantic_limits[i].bigger.size(); j++) {
					if (std::get<0>(model->semantic_limits[i].bigger[j]) < 0) continue;
					BetaType beta_type = model->shape_dofs[std::get<0>(model->semantic_limits[i].bigger[j])].type;
					if (std::get<2>(model->semantic_limits[i].bigger[j]) == false) continue;
					J(num_betas + settings->num_additional_constraints + i, std::get<0>(model->semantic_limits[i].bigger[j])) += -settings->weight_semantic_limits * std::get<1>(model->semantic_limits[i].bigger[j]);
				}
			}
		}
	}

	/// Shape dofs blockers
	if (settings->enable_shape_dofs_blockers) {
		for (size_t constraint_index = 0; constraint_index < 5; constraint_index++) {

			if ((constraint_index == 0 && model->theta[shape_dofs_blockers_theta_indices[constraint_index]] <= shape_dofs_blockers_thresholds[constraint_index]) ||
				(constraint_index > 0 && E_pose->fingers_bending_latent_variable < -1.1)) {
				shape_dofs_blockers_conditions[constraint_index] = false;
				shape_dofs_blockers_values[constraint_index] = beta[shape_dofs_blockers_beta_indices[constraint_index]];
				shape_dofs_blockers_initialization_indicator[constraint_index] = true;
			}
			else {
				shape_dofs_blockers_conditions[constraint_index] = true;
				if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
					F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (beta[shape_dofs_blockers_beta_indices[constraint_index]] - shape_dofs_blockers_values[constraint_index]);
				}
				else {
					F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (beta[shape_dofs_blockers_beta_indices[constraint_index]] - beta_latent[0] * beta_template[shape_dofs_blockers_beta_indices[constraint_index]]);
				}
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[constraint_index]) += settings->weight_shape_dofs_blockers;
			}
		}

		for (size_t constraint_index = 5; constraint_index < 5 + 10; constraint_index++) {
			if (constraint_index == 6) continue;

			size_t i1 = 2 * (constraint_index - 5) + 5;
			size_t i2 = 2 * (constraint_index - 5) + 5 + 1;

			if ((constraint_index >= 7 && model->theta[shape_dofs_blockers_theta_indices[constraint_index]] >= shape_dofs_blockers_thresholds[constraint_index]) ||
				(constraint_index < 7 && model->theta[shape_dofs_blockers_theta_indices[constraint_index]] <= shape_dofs_blockers_thresholds[constraint_index])) {

				shape_dofs_blockers_conditions[constraint_index] = false;
				shape_dofs_blockers_values[i1] = beta[shape_dofs_blockers_beta_indices[i1]];
				shape_dofs_blockers_values[i2] = beta[shape_dofs_blockers_beta_indices[i2]];
				shape_dofs_blockers_initialization_indicator[constraint_index] = true;
			}
			else {
				shape_dofs_blockers_conditions[constraint_index] = true;
				if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
					F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (
							beta[shape_dofs_blockers_beta_indices[i1]] -
							beta[shape_dofs_blockers_beta_indices[i2]] * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2]
							);
					J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i1]) +=
						settings->weight_shape_dofs_blockers;
					J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i2]) +=
						-settings->weight_shape_dofs_blockers * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2];
				}
				else {
					F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
						settings->weight_shape_dofs_blockers * (
							beta[shape_dofs_blockers_beta_indices[i1]] -
							beta[shape_dofs_blockers_beta_indices[i2]] * beta_template[shape_dofs_blockers_beta_indices[i1]] / beta_template[shape_dofs_blockers_beta_indices[i2]]
							);
					J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i1]) +=
						settings->weight_shape_dofs_blockers;
					J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i2]) +=
						-settings->weight_shape_dofs_blockers * beta_template[shape_dofs_blockers_beta_indices[i1]] / beta_template[shape_dofs_blockers_beta_indices[i2]];
				}
			}
		}
		///
		/*size_t constraint_index = 6;
		size_t i1 = 2 * (constraint_index - 5) + 5;
		size_t i2 = 2 * (constraint_index - 5) + 5 + 1;
		size_t beta_i3 = model->shape_dofs_name_to_id_map["thumb_additional_y"];
		if (model->theta[shape_dofs_blockers_theta_indices[constraint_index]] <= shape_dofs_blockers_thresholds[constraint_index]) {
			shape_dofs_blockers_conditions[constraint_index] = false;
			shape_dofs_blockers_values[i1] = beta[shape_dofs_blockers_beta_indices[i1]];
			shape_dofs_blockers_values[i2] = beta[shape_dofs_blockers_beta_indices[i2]] + beta[beta_i3];
			shape_dofs_blockers_initialization_indicator[constraint_index] = true;
		}
		else {
			shape_dofs_blockers_conditions[constraint_index] = true;
			if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
				F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
					settings->weight_shape_dofs_blockers * (
						beta[shape_dofs_blockers_beta_indices[i1]] -
						(beta[shape_dofs_blockers_beta_indices[i2]] + beta[beta_i3]) * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2]
						);
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i1]) +=
					settings->weight_shape_dofs_blockers;
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i2]) +=
					-settings->weight_shape_dofs_blockers * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2];
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, beta_i3) +=
					-settings->weight_shape_dofs_blockers * shape_dofs_blockers_values[i1] / shape_dofs_blockers_values[i2];
			}
			else {
				float template_top_length = beta_template[shape_dofs_blockers_beta_indices[i2]] + beta_template[beta_i3];
				F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
					settings->weight_shape_dofs_blockers * (
						beta[shape_dofs_blockers_beta_indices[i1]] -
						(beta[shape_dofs_blockers_beta_indices[i2]] + beta[beta_i3]) * beta_template[shape_dofs_blockers_beta_indices[i1]] / template_top_length
						);
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i1]) +=
					settings->weight_shape_dofs_blockers;
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, shape_dofs_blockers_beta_indices[i2]) +=
					-settings->weight_shape_dofs_blockers * beta_template[shape_dofs_blockers_beta_indices[i1]] / template_top_length;
				J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, beta_i3) +=
					-settings->weight_shape_dofs_blockers * beta_template[shape_dofs_blockers_beta_indices[i1]] / template_top_length;
			}
		}*/
		///

		///
		/*constraint_index = 15;
		if (!(model->theta[10] > 0.25 && model->theta[12] < -1.1)) {
			shape_dofs_blockers_conditions[constraint_index] = false;
			shape_dofs_blockers_values[shape_dofs_blockers_values.size() - 1] = beta[model->shape_dofs_name_to_id_map["thumb_top_length"]] + beta[model->shape_dofs_name_to_id_map["thumb_additional_y"]];
			shape_dofs_blockers_initialization_indicator[constraint_index] = true;
		}
		else {
			//cout << "blocking" << endl;
			shape_dofs_blockers_conditions[constraint_index] = true;
			if (shape_dofs_blockers_initialization_indicator[constraint_index]) {
				F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
					settings->weight_shape_dofs_blockers * (beta[model->shape_dofs_name_to_id_map["thumb_top_length"]] + 
						beta[model->shape_dofs_name_to_id_map["thumb_additional_y"]] - shape_dofs_blockers_values[shape_dofs_blockers_values.size() - 1]);
			}
			else {
				F(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index) =
					settings->weight_shape_dofs_blockers * (beta[model->shape_dofs_name_to_id_map["thumb_top_length"]] + beta[model->shape_dofs_name_to_id_map["thumb_additional_y"]] - 
						beta_latent[0] * (beta_template[model->shape_dofs_name_to_id_map["thumb_top_length"]] + beta_template[model->shape_dofs_name_to_id_map["thumb_additional_y"]]));
			}
			J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, model->shape_dofs_name_to_id_map["thumb_top_length"]) +=
				settings->weight_shape_dofs_blockers;
			J(num_betas + settings->num_additional_constraints + settings->num_semantic_limits + constraint_index, model->shape_dofs_name_to_id_map["thumb_additional_y"]) +=
				settings->weight_shape_dofs_blockers;
		}*/
	}

	//verify_jacobian(F, J);

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JT = J.transpose();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> LHS = JT * J;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1>  rhs = (-JT * F);

	/*std::ofstream J_file(logs_path + "J-" + std::to_string(iter) + ".txt"); if (J_file.is_open()) J_file << J;
	std::ofstream F_file(logs_path + "F-" + std::to_string(iter) + ".txt"); if (F_file.is_open()) F_file << F;
	std::ofstream JtJ_file(logs_path + "JtJ-" + std::to_string(iter) + ".txt"); if (JtJ_file.is_open()) JtJ_file << LHS;
	std::ofstream JtF_file(logs_path + "JtF-" + std::to_string(iter) + ".txt"); if (JtF_file.is_open()) JtF_file << rhs;*/

	LinearSystem system_posespace(model->num_parameters + num_thetas_latent + num_betas_latent);
	if (enable_pose_prior) {
		system_posespace.lhs.block(0, 0, model->num_parameters + num_thetas_latent, model->num_parameters + num_thetas_latent) = system.lhs;
		system_posespace.rhs.segment(0, model->num_parameters + num_thetas_latent) = system.rhs;
	}
	else {
		system_posespace.lhs.block(0, 0, model->num_parameters, model->num_parameters) = system.lhs;
		system_posespace.rhs.segment(0, model->num_parameters) = system.rhs;
	}
	system = system_posespace;

	system.lhs.block(num_thetas, num_thetas, num_betas, num_betas) += LHS.block(0, 0, num_betas, num_betas);

	system.lhs.block(model->num_parameters + num_thetas_latent, num_thetas, num_betas_latent, num_betas) += LHS.block(num_betas, 0, num_betas_latent, num_betas);
	system.lhs.block(num_thetas, model->num_parameters + num_thetas_latent, num_betas, num_betas_latent) += LHS.block(0, num_betas, num_betas, num_betas_latent);

	system.lhs.block(model->num_parameters + num_thetas_latent, model->num_parameters + num_thetas_latent, num_betas_latent, num_betas_latent) +=
		LHS.block(num_betas, num_betas, num_betas_latent, num_betas_latent);

	system.rhs.segment(num_thetas, num_betas) += rhs.segment(0, num_betas);
	system.rhs.segment(model->num_parameters + num_thetas_latent, num_betas_latent) += rhs.segment(num_betas, num_betas_latent);

	/// > Damping
	/*for (size_t i = 0; i < num_betas_latent; i++) {
		system.lhs(model->num_parameters + num_thetas_latent + i, model->num_parameters + num_thetas_latent + i) += 1000000000;
	}*/
}


