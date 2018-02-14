#include "tracker/HModel/JacobianVerifier.h"
#include "tracker/HModel/Model.h"
#include "util/opencv_wrapper.h"
#include "cudax/cuda_glm.h"
#include "glm/gtx/string_cast.hpp"
#include <glm/gtc/type_ptr.hpp>
#include <iomanip> 
#include <fstream>


JacobianVerifier::JacobianVerifier(bool verify_jacobian, Model * model, std::string logs_path) {
	this->verify_jacobian = verify_jacobian;
	this->logs_path = logs_path;

	if (verify_jacobian) {
		this->model = model;
		correspondences_data_points = new float[upper_bound_num_sensor_points];
		correspondences_model_points = new float[upper_bound_num_sensor_points];
		correspondences_model_indices = new int[upper_bound_num_sensor_points];
		correspondences_block_indices = new int[upper_bound_num_sensor_points / 3];
		jacobian_data = new float[upper_bound_num_sensor_points * model->num_parameters];
		correspondences_silhouette_data_points = new float[3 * upper_bound_num_rendered_outline_points];
		correspondences_silhouette_model_points = new float[3 * upper_bound_num_rendered_outline_points];
		correspondences_silhouette_indices = new int[3 * upper_bound_num_rendered_outline_points];
		jacobian_silhouette = new float[3 * upper_bound_num_rendered_outline_points * model->num_parameters];
	}
}

JacobianVerifier::~JacobianVerifier() {
	if (verify_jacobian) {
		delete[] correspondences_data_points;
		delete[] correspondences_model_points;
		delete[] correspondences_model_indices;
		delete[] correspondences_block_indices;
		delete[] jacobian_data;
		delete[] correspondences_silhouette_data_points;
		delete[] correspondences_silhouette_model_points;
		delete[] correspondences_silhouette_indices;
		delete[] jacobian_silhouette;
	}
}

void JacobianVerifier::write_corresponences_to_file(int id) {

	std::ofstream output_file;

	output_file.open(logs_path + "correspondences_data_points-" + std::to_string(id) + ".txt");
	for (size_t i = 0; i < num_sensor_points * 3; i++) {
		output_file << correspondences_data_points[i] << " ";
		if (i % 3 == 2) output_file << endl;
	}
	output_file.close();

	output_file.open(logs_path + "correspondences_model_points-" + std::to_string(id) + ".txt");
	for (size_t i = 0; i < num_sensor_points * 3; i++) {
		output_file << correspondences_model_points[i] << " ";
		if (i % 3 == 2) output_file << endl;
	}
	output_file.close();

	/*output_file.open(data_path + "correspondences_silhouette_model_points-" + std::to_string(id) + ".txt");
	for (size_t i = 0; i < num_rendered_points * 3; i++) {
	output_file << correspondences_silhouette_model_points[i] << " ";
	}
	output_file.close();*/

	/*output_file.open(data_path + "jacobian-" + std::to_string(id) + ".txt");
	for (size_t i = 0; i < num_sensor_points * model->num_parameters; i++) {
		output_file << jacobian_data[i] << " ";
	}
	output_file.close();*/

}

void JacobianVerifier::jacobian_theta_double(int pose_unit_id, const glm::dvec3 & pos, const std::vector<glm::dmat4> theta_infos_globals) {

	for (int l = 0; l < KINEMATIC_MAX_LENGTH; l++) {
		int theta_info_id = model->host_pointer_kinematic_chains[pose_unit_id].data[l];
		if (theta_info_id == -1) break;
		const ThetaInfo & theta_info = model->theta_infos[theta_info_id];
		float * axis_pointer = model->theta_infos[theta_info_id].axis;
		glm::dvec3 axis = glm::dvec3(axis_pointer[0], axis_pointer[1], axis_pointer[2]);

		glm::dmat4 global = theta_infos_globals[theta_info_id];
		/*glm::dmat4 global = glm::dmat4();
		for (size_t u = 0; u < 4; u++) {
		for (size_t v = 0; v < 4; v++) {
		global[u][v] = dofs_globals_double[joint_info_id](v, u);
		}
		}*/

		glm::dvec3 dq;
		switch (theta_info.type) {
		case 1: {
			dq = glm::dmat3(global) * axis;
			break;
		}
		case 0: {
			glm::dvec3 t(global[3][0], global[3][1], global[3][2]);
			glm::dvec3 u = glm::dmat3(global) * axis;
			dq = glm::cross(u, pos - t);
			break;
		}
		}
		jacobian_analytical.push_back(dq);
	}
}

void JacobianVerifier::jacobian_beta_double(const int shape_unit_id, const glm::dvec3 & q, const glm::dvec3 & s, glm::ivec3 & index, std::vector<glm::dmat4> beta_infos_globals, std::ostream & ostream_id, int beta_id_to_print) {

	if (shape_unit_id == -1) return;
	if (shape_unit_id == 26) {
		for (size_t d = 0; d < 3; d++) {
			if (index[d] == 38 && (index[0] == 33 || index[1] == 33 || index[2] == 33)) index[d] = 24;
			if (index[d] == 38 && (index[0] == 24 || index[1] == 24 || index[2] == 24)) index[d] = 33;
		}
	}


	for (int l = 0; l < SHAPE_MAX_LENGTH; l++) {
		int beta_info_id = model->host_pointer_shape_chains[shape_unit_id].data[l];
		if (beta_info_id == -1) break;
		const BetaInfo& beta_info = model->beta_infos[beta_info_id];
		BetaType beta_type = (BetaType)beta_info.type;
		float * axis_pointer = model->beta_infos[beta_info_id].axis;
		glm::dvec3 axis = glm::dvec3(axis_pointer[0], axis_pointer[1], axis_pointer[2]);
		glm::dmat4 global = beta_infos_globals[beta_info_id];

		glm::dvec3 t(global[3][0], global[3][1], global[3][2]);
		glm::dvec3 v = glm::dmat3(global) * axis;

		glm::dvec4 parametrization = glm::dvec4(0);
		if (index[1] == RAND_MAX)
			parametrization[0] = 1.0;
		else if (index[2] == RAND_MAX)
			compute_segment_coordinates(s, centers_double[index[0]], centers_double[index[1]], parametrization[0], parametrization[1]);
		else
			compute_triangle_coordinates(s, centers_double[index[0]], centers_double[index[1]], centers_double[index[2]], parametrization[0], parametrization[1], parametrization[2]);

		glm::dvec3 dq;
		if (beta_type == PHALANGE_LENGTH || beta_type == TOP_PHALANGE_LENGTH) {
			double scaling_factor = 1.0f;
			if (index[0] == beta_info.center_id && index[1] == beta_info.top_center_id) scaling_factor = 1 - parametrization[0];
			if (index[1] == beta_info.center_id && index[0] == beta_info.top_center_id) scaling_factor = 1 - parametrization[1];
			if (index[0] == beta_info.center_id && index[1] == RAND_MAX) scaling_factor = 0.0;
			if (index[0] == beta_info.top_center_id && index[1] == RAND_MAX) scaling_factor = 1.0;

			if (shape_unit_id == 20 || shape_unit_id == 21 || shape_unit_id == 22 || shape_unit_id == 23 || shape_unit_id == 24 || shape_unit_id == 25) { // membranes	
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) {
						double fraction = 0.0;
						if (beta_info.attachment_center_id == 31) fraction = model->membranes_fractional_length[0];
						if (beta_info.attachment_center_id == 30) fraction = model->membranes_fractional_length[1];
						if (beta_info.attachment_center_id == 29) fraction = model->membranes_fractional_length[2];
						if (beta_info.attachment_center_id == 28) fraction = model->membranes_fractional_length[3];
						scaling_factor = fraction * parametrization[d];						
					}
				}
			}
			if (shape_unit_id == 26) { // thumb membrane				
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) scaling_factor = parametrization[d];
				}
			}
			dq = scaling_factor * v;

			if (verbose && beta_info_id == beta_id_to_print) { 
				ostream_id << "beta_type = PHALANGE_LENGTH || TOP_PHALANGE_LENGTH" << endl;
				ostream_id << "index[0] = " << index[0] << ", index[1] = " << index[1] << endl;
				ostream_id << "beta_info.center_id = " << beta_info.center_id << ", beta_info.top_center_id = " << beta_info.top_center_id << ", beta_info.attachment_center_id = " << beta_info.attachment_center_id << endl;
				if (index[0] == beta_info.center_id && index[1] == beta_info.top_center_id) ostream_id << "scaling_factor = 1 - parametrization[0] = " << scaling_factor << endl;
				if (index[1] == beta_info.center_id && index[0] == beta_info.top_center_id) ostream_id << "scaling_factor = 1 - parametrization[1] = " << scaling_factor << endl;
				if (index[0] == beta_info.center_id && index[1] == RAND_MAX) ostream_id << "scaling_factor = " << scaling_factor << endl;
				if (index[0] == beta_info.top_center_id && index[1] == RAND_MAX) ostream_id << "scaling_factor = " << scaling_factor << endl;
				
				ostream_id << endl;
			}
		}
		if (beta_type == FINGER_BASE_X || beta_type == FINGER_BASE_Y || beta_type == FINGER_BASE_Z) {
			double scaling_factor = 1.0;
			if (shape_unit_id == 20 || shape_unit_id == 21 || shape_unit_id == 22 || shape_unit_id == 23 || shape_unit_id == 24 || shape_unit_id == 25) { // membranes				
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) {
						scaling_factor = parametrization[d];
					}
				}				
			}
			if (shape_unit_id == 26) { // thumb membrane				
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) scaling_factor = 1 - parametrization[d];
				}
			}
			dq = scaling_factor * v;

			if (verbose && beta_info_id == beta_id_to_print) {
				ostream_id << "beta_type = FINGER_BASE" << endl;
				ostream_id << "beta_info.center_id = " << beta_info.center_id << endl;
				ostream_id << "index = (" << index[0] << ", " << index[1] << ", " << index[2] << ")" << endl;
				ostream_id << "parametrization = (" << parametrization[0] << ", " << parametrization[1] << ", " << parametrization[2] << ")" << endl;
				ostream_id << "scaling_factor = " << scaling_factor << endl;
				ostream_id << "v = " << glm::to_string(v) << endl;
				ostream_id << "dq = " << glm::to_string(dq) << endl;
			}
		}
		if (beta_type == PALM_CENTER_X || beta_type == PALM_CENTER_Y) {	
			bool print = false;
			double independent_scaling_factor = 0;
			double dependent_scaling_factor = 0;
			
			for (size_t d = 0; d < 3; d++) {
				if (index[d] == beta_info.center_id) independent_scaling_factor = parametrization[d];

				if (beta_info.center_id == 20 || beta_info.center_id == 23) { // centers 21 and 22 linearly depend on centers 20 and 23
					if (index[d] == 21 || index[d] == 22) {						
						double palm_width = glm::length(centers_double[20] - centers_double[23]);
						double width_ratio = 1 - glm::length(centers_double[index[d]] - centers_double[beta_info.center_id]) / palm_width;
						dependent_scaling_factor += parametrization[d] * width_ratio;						
					}
				}
			}

			dq = (independent_scaling_factor + dependent_scaling_factor) * v;

			if (shape_unit_id == 27) {
				glm::dvec4 get_i = glm::dvec4(3); for (size_t d = 0; d < 3; d++) get_i[index[d] - 34] = d;
				if (beta_type == PALM_CENTER_X && beta_info.top_center_id == 35)
					dq = parametrization[get_i[36 - 34]] * v - parametrization[get_i[35 - 34]] * v - parametrization[get_i[37 - 34]] * v;
				if (beta_type == PALM_CENTER_X && beta_info.top_center_id == 34) {
					dq = parametrization[get_i[34 - 34]] * v;
				}
				if (beta_type == PALM_CENTER_Y)
					dq = - parametrization[get_i[36 - 34]] * v - parametrization[get_i[37 - 34]] * v;
			}

			if (verbose && beta_info_id == beta_id_to_print) {
				ostream_id << "beta_type = PALM_CENTER" << endl;
				ostream_id << "index = " << index[0] << ", index[1] = " << index[1] << ", index[2] = " << index[2] << endl;
				ostream_id << "parametrization = (" << parametrization[0] << ", " << parametrization[1] << ", " << parametrization[2] << ")" << endl;
			}
		}
		if (beta_type == RADIUS) {
			glm::dvec3 u = q - s;
			u = u / glm::length(u);
			double scaling_factor = 0;
			for (size_t d = 0; d < 3; d++) {
				if (index[d] == beta_info.center_id) scaling_factor = parametrization[d];
			}
			dq = scaling_factor * u;

			if (shape_unit_id == 27) {
				double second_scaling_factor = 0;
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.top_center_id) second_scaling_factor = parametrization[d];
				}
				dq = (scaling_factor + second_scaling_factor) * u;
			}

			if (verbose && beta_info_id == beta_id_to_print) {
				ostream_id << "beta_type = RADIUS" << endl;
				ostream_id << "scaling_factor = " << scaling_factor << endl;
			}
		}
		if (beta_type == FINGER_TOP_Y || beta_type == FINGER_TOP_Z) {
			double scaling_factor = 0;
			if (index[0] == beta_info.center_id && index[1] == beta_info.top_center_id) scaling_factor = 1 - parametrization[0];
			if (index[1] == beta_info.center_id && index[0] == beta_info.top_center_id) scaling_factor = 1 - parametrization[1];
			if (index[0] == beta_info.center_id && index[1] == RAND_MAX) scaling_factor = 0.0;
			if (index[0] == beta_info.top_center_id && index[1] == RAND_MAX) scaling_factor = 1.0;
			dq = scaling_factor * v;

			if (verbose && beta_info_id == beta_id_to_print) {
				ostream_id << "beta_type = FINGER_TOP" << endl;
				ostream_id << "beta_info.center_id = " << beta_info.center_id << endl;
				ostream_id << "index = (" << index[0] << ", " << index[1] << ", " << index[2] << ")" << endl;
				ostream_id << "parametrization = (" << parametrization[0] << ", " << parametrization[1] << ", " << parametrization[2] << ")" << endl;
				ostream_id << "mat = " << endl << glm::to_string(glm::dmat3(global)) << endl;
				ostream_id << "axis = " << glm::to_string(axis) << endl;
				ostream_id << "scaling_factor = " << scaling_factor << endl;
				ostream_id << "v = " << glm::to_string(v) << endl;
			}
		}
		jacobian_analytical.push_back(dq);
	}
}

glm::dvec3 JacobianVerifier::function_htrack_parametrization(const size_t block_id, const glm::ivec3 & index, const size_t parameter_id, const double k_alpha, const double k_beta, const double k_gamma, const glm::dvec3 & offset) {
	update_beta_double();
	if (parameter_id < num_thetas)
		update_theta_double(true);
	else
		update_theta_double(false);
	update_centers_double();
	convert_theta_infos_to_double();
	convert_beta_infos_to_double();

	size_t phalange_id = model->blockid_to_phalangeid_map[block_id];
	glm::dmat3 global = glm::dmat3(convert_eigen_matrix_to_glm_matrix(globals_double[phalange_id]));
	glm::dvec3 q, s; double r;
	if (index[1] == RAND_MAX) {
		s = k_alpha * centers_double[index[0]];
		r = k_alpha * radii_double[index[0]];
	}
	else if (index[2] == RAND_MAX) {
		s = k_alpha * centers_double[index[0]] + k_beta * centers_double[index[1]];
		r = k_alpha * radii_double[index[0]] + k_beta * radii_double[index[1]];
	}
	else {
		s = k_alpha * centers_double[index[0]] + k_beta * centers_double[index[1]] + k_gamma * centers_double[index[2]];
		r = k_alpha * radii_double[index[0]] + k_beta * radii_double[index[1]] + k_gamma * radii_double[index[2]];
	}
	q = s + r * global * offset;
	return q;
}

void JacobianVerifier::verify_one_parameter(const size_t block_id, const glm::ivec3 & index, const size_t parameter_id, const double k_alpha, const double k_beta, const double k_gamma, const glm::dvec3 & offset) {
	/// > compute function plus
	double epsilon = 1e-10;

	if (parameter_id < num_thetas) {
		std::fill(theta_double.begin(), theta_double.end(), 0);
		theta_double[parameter_id] += epsilon;
	}
	else beta_double[parameter_id - num_thetas] += epsilon;
	glm::dvec3 q_plus = function_htrack_parametrization(block_id, index, parameter_id, k_alpha, k_beta, k_gamma, offset);

	/// > compute function minus
	if (parameter_id < num_thetas) {
		std::fill(theta_double.begin(), theta_double.end(), 0);
		theta_double[parameter_id] -= epsilon;
	}
	else beta_double[parameter_id - num_thetas] -= 2 * epsilon;
	glm::dvec3 q_minus = function_htrack_parametrization(block_id, index, parameter_id, k_alpha, k_beta, k_gamma, offset);
	glm::dvec3 dq = (q_plus - q_minus) / 2.0 / epsilon;

	jacobian_numerical.push_back(dq);
}

void JacobianVerifier::compute_parametrization(const glm::ivec3 index, const size_t block_id, const glm::dvec3 & model_point, glm::dvec3 & q, glm::dvec3 & s, double & k_alpha, double & k_beta, double & k_gamma, glm::dvec3 & offset, std::ostream & ostream_id) {

	/// > compute skeletan projection
	glm::ivec3 new_index;
	if (index[1] == RAND_MAX) {
		s = centers_double[index[0]];
		q = model_point;
		new_index = index;
	}
	else if (index[2] == RAND_MAX) {
		glm::dvec3 c1 = centers_double[index[0]]; glm::dvec3 c2 = centers_double[index[1]];
		double r1 = radii_double[index[0]]; double r2 = radii_double[index[1]];
		projection_convsegment_double(model_point, c1, c2, r1, r2, index[0], index[1], q, s, new_index);
	}
	else {
		glm::dvec3 c1 = centers_double[index[0]]; glm::dvec3 c2 = centers_double[index[1]]; glm::dvec3 c3 = centers_double[index[2]];
		double r1 = radii_double[index[0]]; double r2 = radii_double[index[1]]; double r3 = radii_double[index[2]];
		projection_convtriangle_double(model_point, c1, c2, c3, r1, r2, r3, index[0], index[1], index[2],
			tangent_points_double[block_id].v1, tangent_points_double[block_id].v2, tangent_points_double[block_id].v3, tangent_points_double[block_id].n,
			tangent_points_double[block_id].u1, tangent_points_double[block_id].u2, tangent_points_double[block_id].u3, tangent_points_double[block_id].m, model->camera_ray,
			q, s, new_index);
	}	

	if (index[1] == RAND_MAX)
		k_alpha = 1.0;
	else if (index[2] == RAND_MAX)
		compute_segment_coordinates(s, centers_double[index[0]], centers_double[index[1]], k_alpha, k_beta);
	else
		compute_triangle_coordinates(s, centers_double[index[0]], centers_double[index[1]], centers_double[index[2]], k_alpha, k_beta, k_gamma);

	size_t phalange_id = model->blockid_to_phalangeid_map[block_id];
	glm::dmat3 global = glm::dmat3(convert_eigen_matrix_to_glm_matrix(globals_double[phalange_id]));
	glm::dmat3 global_inv = glm::inverse(global);
	offset = global_inv * (q - s);
	double r = glm::length(offset);
	offset = offset / r;
	 
	/// > Verify parametrization
	glm::dvec3 q_new, s_new; double r_new;
	if (index[1] == RAND_MAX) {
		s_new = k_alpha * centers_double[index[0]];
		r_new = k_alpha * radii_double[index[0]];
	}
	else if (index[2] == RAND_MAX) {
		s_new = k_alpha * centers_double[index[0]] + k_beta * centers_double[index[1]];
		r_new = k_alpha * radii_double[index[0]] + k_beta * radii_double[index[1]];
	}
	else {
		s_new = k_alpha * centers_double[index[0]] + k_beta * centers_double[index[1]] + k_gamma * centers_double[index[2]];
		r_new = k_alpha * radii_double[index[0]] + k_beta * radii_double[index[1]] + k_gamma * radii_double[index[2]];
	}

	q_new = s_new + r_new * global * offset;

	if (verbose) {

		ostream_id << "original and recomputed index" << endl;
		ostream_id << glm::to_string(index) << endl;
		ostream_id << glm::to_string(new_index) << endl << endl;

		ostream_id << "original and recomputed model point" << endl;
		ostream_id << glm::to_string(model_point) << endl;
		ostream_id << glm::to_string(q) << endl;
		ostream_id << glm::to_string(q_new) << endl << endl;

		ostream_id << "original and recomputed skeletal projection" << endl;
		ostream_id << glm::to_string(s) << endl;
		ostream_id << glm::to_string(s_new) << endl << endl;

		ostream_id << "original and recomputed radius" << endl;
		ostream_id << r << endl;
		ostream_id << r_new << endl << endl;

		ostream_id << "centers" << endl;
		ostream_id << glm::to_string(centers_double[index[0]]) << endl;
		if (index[1] != RAND_MAX) ostream_id << glm::to_string(centers_double[index[1]]) << endl;
		if (index[2] != RAND_MAX) ostream_id << glm::to_string(centers_double[index[2]]) << endl;

		glm::dvec3 parametrization = glm::dvec3(k_alpha, k_beta, k_gamma);
		ostream_id << endl << "parametrization" << endl;
		ostream_id << glm::to_string(parametrization) << endl;
	}
}

void JacobianVerifier::verify_all_parameters(const glm::dvec3 & model_point, const size_t phalange_id, const size_t shape_unit_id, const std::vector<size_t> & phalange_ids, const std::vector<size_t> & shape_unit_ids, const size_t block_id, glm::ivec3 & index) {
	
	/// > compute parametrization 
	double k_alpha = 0, k_beta = 0, k_gamma = 0; glm::dvec3 s, q; glm::dvec3 offset; compute_parametrization(index, block_id, model_point, q, s, k_alpha, k_beta, k_gamma, offset);
	if (verbose) cout << "( " << k_alpha << ", " << k_beta << ", " << k_gamma << ") " << endl << endl;

	for (size_t i = 0; i < locals_double.size(); i++) current_locals_double[i] = locals_double[i];
	std::vector<double> theta_double_initial = theta_double;

	for (size_t i = 0; i < phalange_ids.size(); i++) {
		verify_one_parameter(block_id, index, phalange_ids[i], k_alpha, k_beta, k_gamma, offset);
	}
	jacobian_theta_double(phalange_id, model_point, theta_infos_globals);

	theta_double = theta_double_initial;
	std::vector<double> beta_double_initial = beta_double;
	for (size_t i = 0; i < shape_unit_ids.size(); i++) {
		beta_double = beta_double_initial;
		verify_one_parameter(block_id, index, shape_unit_ids[i] + num_thetas, k_alpha, k_beta, k_gamma, offset);
	}
	jacobian_beta_double(shape_unit_id, model_point, s, index, beta_infos_globals, std::cout, 45);

	/*cout << "FLOAT JACOBIAN" << endl;
	//Matrix_3xN analytical_float = model->jacobian_beta(shape_unit_id, model_point, s, index);
	Matrix_3xN analytical_float = model->jacobian_theta_new(phalange_id, model_point, s, index);
	cout << "after" << endl;
	for (size_t i = 0; i < analytical_float.cols(); i++) {
		Vector3 j_column = analytical_float.col(i);
		if (j_column.norm() > 0) {
			cout << i << ": (" << j_column[0] << ", " << j_column[1] << ", " << j_column[2] << ")" << endl;
		}
	}*/
}

void JacobianVerifier::main_verify_jacobian_for_one_point() {
	//system("cls");
	verbose = true;
	size_t block_id = 28;
	glm::ivec3 index = model->blocks[block_id];

	std::vector<size_t> pose_unit_ids;
	size_t pose_unit_id = model->blockid_to_pose_unit_id_map[block_id];
	for (int l = 0; l < KINEMATIC_MAX_LENGTH; l++) {
		int current_pose_unit_id = model->host_pointer_kinematic_chains[pose_unit_id].data[l];
		if (current_pose_unit_id == -1) break;
		pose_unit_ids.push_back(current_pose_unit_id);
	}

	std::vector<size_t> shape_unit_ids;
	int shape_unit_id = model->blockid_to_shape_unit_id_map[block_id];	

	if (shape_unit_id == -1) {
		cout << "shape unit is unspecified for this point" << endl;
		return;
	}
	for (int l = 0; l < SHAPE_MAX_LENGTH; l++) {
		int current_shape_unit_id = model->host_pointer_shape_chains[shape_unit_id].data[l];
		if (current_shape_unit_id == -1) break;
		shape_unit_ids.push_back(current_shape_unit_id);
	}

	convert_to_doubles();

	/// > verify jacobian data
	{
		jacobian_analytical.clear();
		jacobian_numerical.clear();
		jacobian_data_gpu.clear();

		glm::dvec3 model_point; bool is_found = false;
		for (size_t i = 0; i < num_sensor_points; i++) {
			glm::ivec3 current_index = glm::make_vec3(correspondences_model_indices + 3 * i);
			int current_block_index = correspondences_block_indices[i];
			if (index[0] == current_index[0] && index[1] == current_index[1] && index[2] == current_index[2]) {
				cout << "current_index = " << current_index[0] << " " << current_index[1] << " " << current_index[2] << endl;
				cout << "current_block_index = " << current_block_index << endl;
				model_point = glm::make_vec3(correspondences_model_points + 3 * i);
				if (glm::length(model_point) < 1e-9) {
					cout << "discarded data point" << endl;
					continue;
				}
				for (size_t j = 0; j < model->num_parameters; j++) {
					glm::dvec3 col = glm::make_vec3(jacobian_data + 3 * i * model->num_parameters + 3 * j);
					jacobian_data_gpu.push_back(col);
				}
				is_found = true;
				break;
			}
		}
		if (!is_found) { cout << "no correspondence like that" << endl; return; }

		verify_all_parameters(model_point, pose_unit_id, shape_unit_id, pose_unit_ids, shape_unit_ids, block_id, index);

		for (size_t i = 0; i < jacobian_analytical.size(); i++) {
			size_t k;
			if (i <pose_unit_ids.size()) {
				k = pose_unit_ids[i];
				cout << "theta " << k << ":" << endl;
			}
			else {
				k = num_thetas + shape_unit_ids[i - pose_unit_ids.size()];
				cout << "beta " << k - num_thetas << ":" << endl;
			}
			
			std::cout << "n = " << std::setprecision(4) << jacobian_numerical[i][0] << " " << jacobian_numerical[i][1] << " " << jacobian_numerical[i][2] << endl;
			std::cout << "a = " << std::setprecision(4) << jacobian_analytical[i][0] << " " << jacobian_analytical[i][1] << " " << jacobian_analytical[i][2] << endl;			
			std::cout << "g = " << std::setprecision(4) << jacobian_data_gpu[k][0] << " " << jacobian_data_gpu[k][1] << " " << jacobian_data_gpu[k][2] << endl;
			cout << endl;
		}
	}
}

void JacobianVerifier::verify_again_helper(int sensor_point_id, int parameter_id, std::ostream & ostream_id) {
	verbose = true;
	glm::ivec3 index = glm::make_vec3(correspondences_model_indices + 3 * sensor_point_id);
	glm::dvec3 model_point = glm::make_vec3(correspondences_model_points + 3 * sensor_point_id);
	int block_id = correspondences_block_indices[sensor_point_id];

	ostream_id << "sensor_point_id = " << sensor_point_id << endl;
	ostream_id << "block_id = " << block_id << endl << endl;

	double k_alpha = 0, k_beta = 0, k_gamma = 0; glm::dvec3 s, q; glm::dvec3 offset;
	compute_parametrization(index, block_id, model_point, q, s, k_alpha, k_beta, k_gamma, offset, ostream_id);

	if (parameter_id < num_thetas) {

	}
	if (parameter_id >= num_thetas) {
		int shape_unit_id = model->blockid_to_shape_unit_id_map[block_id];
		jacobian_beta_double(shape_unit_id, model_point, s, index, beta_infos_globals, ostream_id, parameter_id - num_thetas);
	}
}

void JacobianVerifier::main_verify_jacobian_for_all_points() {
	verbose = false;
	system("cls");
	
	convert_to_doubles();
	bool verify_jacobian_theta = false;
	bool verify_jacobian_beta = true;

	/// > compute parametrization
	std::vector<glm::dvec3> parametrizations = std::vector<glm::dvec3>(num_sensor_points, glm::dvec3(0, 0, 0));
	std::vector<glm::dvec3> offsets;
	std::vector<glm::dvec3> skeletal_projections; 	
	std::vector<std::vector<glm::dvec3>> jacobians_theta_numerical;
	std::vector<std::vector<glm::dvec3>> jacobians_beta_numerical;

	cout << "num_sensor_points = " << num_sensor_points << endl;

	for (size_t i = 0; i < num_sensor_points; i++ ) {
		glm::ivec3 index = glm::make_vec3(correspondences_model_indices + 3 * i);
		glm::dvec3 model_point = glm::make_vec3(correspondences_model_points + 3 * i);
		int block_id = correspondences_block_indices[i];
		double k_alpha = 0, k_beta = 0, k_gamma = 0; glm::dvec3 s, q; glm::dvec3 offset; 
		compute_parametrization(index, block_id, model_point, q, s, k_alpha, k_beta, k_gamma, offset);
		parametrizations[i] = glm::vec3(k_alpha, k_beta, k_gamma);

		if (abs(k_alpha) > std::numeric_limits<float>::max() || isnan(k_alpha) || abs(k_beta) > std::numeric_limits<float>::max() || isnan(k_beta) || abs(k_gamma) > std::numeric_limits<float>::max() || isnan(k_gamma)) {
			cout << "sensor_point_id = " << i << endl;
			cout << "block_id = " << block_id << endl;
			cout << "model_point = " << model_point[0] << " " << model_point[1] << " " << model_point[2] << endl;
			cout << "index = " << index[0] << " " << index[1] << " " << index[2] << endl;
			cout << "skeletal_projection = " << s[0] << " " << s[1] << " " << s[2] << endl;
			cout << "parametrization = " << k_alpha << " " << k_beta << " " << k_gamma << endl;
			return;
		}
		offsets.push_back(offset);
		skeletal_projections.push_back(s);
	}

	double epsilon = 1e-10;
	std::vector<double> theta_double_initial = theta_double;
	if (verify_jacobian_theta) {
		std::vector<glm::dvec3> jacobian_theta_numerical = std::vector<glm::dvec3>(num_thetas, glm::dvec3(0, 0, 0));
		jacobians_theta_numerical = std::vector<std::vector<glm::dvec3>>(num_sensor_points, jacobian_theta_numerical);
		for (size_t i = 0; i < locals_double.size(); i++) current_locals_double[i] = locals_double[i];

		for (size_t theta_id = 0; theta_id < num_thetas; theta_id++) {
			cout << "theta_id = " << theta_id << endl;
			std::vector<glm::dvec3> model_points_plus = std::vector<glm::dvec3>(num_sensor_points, glm::dvec3(0, 0, 0));

			/// > compute q_plus
			std::fill(theta_double.begin(), theta_double.end(), 0);
			theta_double[theta_id] += epsilon;
			for (size_t i = 0; i < num_sensor_points; i++) {
				glm::ivec3 index = glm::make_vec3(correspondences_model_indices + 3 * i);
				int block_id = correspondences_block_indices[i];
				glm::dvec3 q_plus = function_htrack_parametrization(block_id, index, theta_id, parametrizations[i][0], parametrizations[i][1], parametrizations[i][2], offsets[i]);
				
				if (abs(q_plus[0]) > std::numeric_limits<float>::max() || isnan(q_plus[0]) ||
					abs(q_plus[1]) > std::numeric_limits<float>::max() || isnan(q_plus[1]) ||
					abs(q_plus[2]) > std::numeric_limits<float>::max() || isnan(q_plus[2])) {
					verify_again_helper(i, theta_id);	return;
				}
				
				model_points_plus[i] = q_plus;
			}
			/// compute q_minus
			std::fill(theta_double.begin(), theta_double.end(), 0);
			theta_double[theta_id] -= epsilon;
			for (size_t i = 0; i < num_sensor_points; i++) {
				glm::ivec3 index = glm::make_vec3(correspondences_model_indices + 3 * i);
				int block_id = correspondences_block_indices[i];
				glm::dvec3 q_minus = function_htrack_parametrization(block_id, index, theta_id, parametrizations[i][0], parametrizations[i][1], parametrizations[i][2], offsets[i]);
				
				if (abs(q_minus[0]) > std::numeric_limits<float>::max() || isnan(q_minus[0]) ||
					abs(q_minus[1]) > std::numeric_limits<float>::max() || isnan(q_minus[1]) ||
					abs(q_minus[2]) > std::numeric_limits<float>::max() || isnan(q_minus[2])) {
					verify_again_helper(i, theta_id); return;
				}

				glm::dvec3 dq = (model_points_plus[i] - q_minus) / 2.0 / epsilon;
				jacobians_theta_numerical[i][theta_id] = dq;
			}
		}
	}

	if (verify_jacobian_beta) {
		std::vector<glm::dvec3> jacobian_beta_numerical = std::vector<glm::dvec3>(model->num_betas, glm::dvec3(0, 0, 0));
		jacobians_beta_numerical = std::vector<std::vector<glm::dvec3>>(num_sensor_points, jacobian_beta_numerical);

		theta_double = theta_double_initial;
		std::vector<double> beta_double_initial = beta_double;

		for (size_t beta_id = 0; beta_id < model->num_betas; beta_id++) {
			beta_double = beta_double_initial;
			cout << "beta_id = " << beta_id << endl;
			std::vector<glm::dvec3> model_points_plus = std::vector<glm::dvec3>(num_sensor_points, glm::dvec3(0, 0, 0));

			/// > compute q_plus
			beta_double[beta_id] += epsilon;
			for (size_t i = 0; i < num_sensor_points; i++) {
				glm::ivec3 index = glm::make_vec3(correspondences_model_indices + 3 * i);
				int block_id = correspondences_block_indices[i];
				int shape_unit_id = model->blockid_to_shape_unit_id_map[block_id];
				if (shape_unit_id == -1) continue;
				glm::dvec3 q_plus = function_htrack_parametrization(block_id, index, num_thetas + beta_id, parametrizations[i][0], parametrizations[i][1], parametrizations[i][2], offsets[i]);				

				if (abs(q_plus[0]) > std::numeric_limits<float>::max() || isnan(q_plus[0]) ||
					abs(q_plus[1]) > std::numeric_limits<float>::max() || isnan(q_plus[1]) ||
					abs(q_plus[2]) > std::numeric_limits<float>::max() || isnan(q_plus[2])) {
					verify_again_helper(i, num_thetas + beta_id);	return;
				}
				model_points_plus[i] = q_plus;
			}
			/// compute q_minus
			beta_double[beta_id] -= 2 * epsilon;
			for (size_t i = 0; i < num_sensor_points; i++) {
				glm::ivec3 index = glm::make_vec3(correspondences_model_indices + 3 * i);
				int block_id = correspondences_block_indices[i];
				int shape_unit_id = model->blockid_to_shape_unit_id_map[block_id];
				if (shape_unit_id == -1) continue;
				glm::dvec3 q_minus = function_htrack_parametrization(block_id, index, num_thetas + beta_id, parametrizations[i][0], parametrizations[i][1], parametrizations[i][2], offsets[i]);				

				if (abs(q_minus[0]) > std::numeric_limits<float>::max() || isnan(q_minus[0]) ||
					abs(q_minus[1]) > std::numeric_limits<float>::max() || isnan(q_minus[1]) ||
					abs(q_minus[2]) > std::numeric_limits<float>::max() || isnan(q_minus[2])) {
					verify_again_helper(i, num_thetas + beta_id); return;
				}

				glm::dvec3 dq = (model_points_plus[i] - q_minus) / 2.0 / epsilon;
				jacobians_beta_numerical[i][beta_id] = dq;
			}
		}
	}

	/// > compare numerical and analytical jacobians for all points	
	std::ostream * ostream_id = & std::cout;
	bool write_to_screen = false;
	bool write_to_file = true;
	if (write_to_file) ostream_id = new std::ofstream(logs_path + "jacobian_verifier.txt");
	int block_id_to_print = 6; int beta_id_to_print = 25; float error_tolerance = 0.1;

	int max_difference_sensor_point_id = -1;
	double max_difference_value = 0;
	int max_difference_parameter_id = -1;
	glm::dvec3 max_diffrence_dq_analyt;
	for (size_t sensor_point_id = 0; sensor_point_id < num_sensor_points; sensor_point_id++) {		
		glm::dvec3 model_point = glm::make_vec3(correspondences_model_points + 3 * sensor_point_id);
		int block_id = correspondences_block_indices[sensor_point_id];
		int shape_unit_id = model->blockid_to_shape_unit_id_map[block_id];
		if (shape_unit_id == -1) continue; //TODO: FIX THIS
		if (block_id == 14) continue; //TODO: FIX THIS

		//if (sensor_point_id % 100 != 0) continue;

		if (verify_jacobian_beta) {
			jacobian_analytical.clear();
			jacobian_beta_double(shape_unit_id, model_point, skeletal_projections[sensor_point_id], model->blocks[block_id], beta_infos_globals);

			for (int l = 0; l < SHAPE_MAX_LENGTH; l++) {
				int beta_id = model->host_pointer_shape_chains[shape_unit_id].data[l];
				if (beta_id == -1) break;	

				glm::dvec3 dq_num = jacobians_beta_numerical[sensor_point_id][beta_id];	
				glm::dvec3 dq_gpu = glm::make_vec3(jacobian_data + 3 * sensor_point_id * model->num_parameters + 3 * (num_thetas +  beta_id));	
				glm::dvec3 dq_analyt = jacobian_analytical[l];

				if ((write_to_file || write_to_screen) && 
					(glm::length(dq_gpu - dq_num) > error_tolerance || glm::length(dq_gpu - dq_analyt) > error_tolerance)) {

					if(write_to_file) cout << "writing to file" << endl;

					*ostream_id << endl << endl << endl;
					*ostream_id << "SENSOR_POINT_ID = " << sensor_point_id << endl;
					*ostream_id << "beta_id = " << beta_id << endl;
					*ostream_id << std::setprecision(4) << dq_num[0] << " " << dq_num[1] << " " << dq_num[2] << " - numerical" << endl; 
					*ostream_id << std::setprecision(4) << dq_analyt[0] << " " << dq_analyt[1] << " " << dq_analyt[2] << " - analytical" << endl;
					*ostream_id << std::setprecision(4) << dq_gpu[0] << " " << dq_gpu[1] << " " << dq_gpu[2] << " - gpu" << endl;
					*ostream_id << endl;

					verify_again_helper(sensor_point_id, num_thetas + beta_id, *ostream_id);
				}

				double difference_value = glm::length(dq_gpu - dq_num);
				if (difference_value > max_difference_value) {
					glm::dvec3 data_point = glm::make_vec3(correspondences_data_points + 3 * sensor_point_id);
					if (glm::length(data_point - model_point) >= 1e-5) {
						max_difference_value = difference_value;
						max_difference_sensor_point_id = sensor_point_id;
						max_difference_parameter_id = num_thetas + beta_id;
						max_diffrence_dq_analyt = dq_analyt;
					}
				}
			}
		}
		if (verify_jacobian_theta) {
			jacobian_analytical.clear();
			size_t pose_unit_id = model->blockid_to_pose_unit_id_map[block_id];
			jacobian_theta_double(pose_unit_id, model_point, theta_infos_globals);

			for (int l = 0; l < KINEMATIC_MAX_LENGTH; l++) {
				int theta_id = model->host_pointer_kinematic_chains[pose_unit_id].data[l];
				if (theta_id == -1) break;

				glm::dvec3 dq_num = jacobians_theta_numerical[sensor_point_id][theta_id];
				glm::dvec3 dq_gpu = glm::make_vec3(jacobian_data + 3 * sensor_point_id * model->num_parameters + 3 * theta_id);

				if (write_to_screen) {
					cout << "theta_id = " << theta_id << endl;
					std::cout << std::setprecision(4) << dq_num[0] << " " << dq_num[1] << " " << dq_num[2] << endl;
					std::cout << std::setprecision(4) << jacobian_analytical[l][0] << " " << jacobian_analytical[l][1] << " " << jacobian_analytical[l][2] << endl;
					std::cout << std::setprecision(4) << dq_gpu[0] << " " << dq_gpu[1] << " " << dq_gpu[2] << endl;
					cout << endl;
				}

				double difference_value = glm::length(dq_gpu - dq_num);
				if (difference_value > max_difference_value) {
					max_difference_value = difference_value;
					max_difference_sensor_point_id = sensor_point_id;
					max_difference_parameter_id = theta_id;
					max_diffrence_dq_analyt = jacobian_analytical[l];
				}
			}
		}
	}

	/// > show max difference
	glm::dvec3 model_point = glm::make_vec3(correspondences_model_points + 3 * max_difference_sensor_point_id);
	cout << endl << "max_difference_value = " << max_difference_value << endl;

	glm::dvec3  max_diffrence_dq_gpu = glm::make_vec3(jacobian_data + 3 * max_difference_sensor_point_id * model->num_parameters + 3 * (max_difference_parameter_id));
	glm::dvec3 max_diffrence_dq_num;
	if (max_difference_parameter_id < num_thetas) 
		max_diffrence_dq_num = jacobians_theta_numerical[max_difference_sensor_point_id][max_difference_parameter_id];			
	else 
		max_diffrence_dq_num = jacobians_beta_numerical[max_difference_sensor_point_id][max_difference_parameter_id - num_thetas];		
	

	std::cout << std::setprecision(4) << glm::to_string(max_diffrence_dq_num) << " - numerical" << endl;
	std::cout << std::setprecision(4) << glm::to_string(max_diffrence_dq_analyt) << " - analytical" << endl;
	std::cout << std::setprecision(4) << glm::to_string(max_diffrence_dq_gpu) << " - gpu" << endl;

	glm::dvec3 data_point = glm::make_vec3(correspondences_data_points + 3 * max_difference_sensor_point_id);
	std::cout << std::setprecision(4) << endl << "data_point: " << endl << glm::to_string(data_point) << endl;

	if (max_difference_parameter_id < num_thetas)
		cout << endl << "theta = " << max_difference_parameter_id << endl;
	if (max_difference_parameter_id >= num_thetas && max_difference_parameter_id < num_thetas + model->calibration_type_to_num_betas_map[FINGERS_AND_PALM])
		cout << endl << "beta = " << max_difference_parameter_id - num_thetas << ", beta_type = length or center" << endl;
	if (max_difference_parameter_id >= num_thetas + model->calibration_type_to_num_betas_map[FINGERS_AND_PALM])
		cout << endl << "beta = " << max_difference_parameter_id - num_thetas - model->calibration_type_to_num_betas_map[FINGERS_AND_PALM] << ", beta_type = radius" << endl;

	verify_again_helper(max_difference_sensor_point_id, max_difference_parameter_id);

	cout << endl << "max_difference_value = " << max_difference_value << endl;
}


