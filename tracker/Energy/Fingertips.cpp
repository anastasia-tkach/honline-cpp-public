#pragma once

#include "tracker/Energy/Fingertips.h"
#include "tracker/Data/DataFrame.h"
#include "glm/gtx/string_cast.hpp"

//#include "tracker/Detection/FindFingers.h"
//#include "tracker/Detection/DetectionStream.h"
//#include "tracker/HModel/FingertipsDetector.h"

void Fingertips::init(Model * model, Camera * camera) {
	this->model = model;
	this->camera = camera;

	data_fingertips_history = std::vector<std::queue<glm::vec3>>(num_fingers, std::queue<glm::vec3>());
	data_fingertips_sum = std::vector<glm::vec3>(num_fingers, glm::vec3(0));
	last_update_frame_id = std::vector<int>(num_fingers, -1);

	//DetectionStream temp_detection = DetectionStream(model);
	//FindFingers find_fingers = FindFingers(worker, &temp_detection);
	//fingertips_detector = FingertipsDetector(&find_fingers);
}

void Fingertips::moving_window_average(int current_frame_id) {

	//for (size_t i = 0; i < num_fingers; i++) std::cout << i << ": " << glm::to_string(ordered_data_fingertips[i]) << endl;

	//std::vector<bool> detected_fingertips = std::vector<bool>(5, false);	
	averaged_data_fingertips = std::vector<glm::vec3>(num_fingers, glm::vec3(0));

	for (size_t i = 0; i < num_fingers; i++) {

		if (glm::length(ordered_data_fingertips[i]) > std::numeric_limits<float>::epsilon() && last_frame_id != current_frame_id) {
			glm::vec3 offset = ordered_data_fingertips[i];

			/// Big motion: discard everything
			glm::vec3 averaged_data_fingertip = data_fingertips_sum[i] / (float)data_fingertips_history[i].size();
			if (!data_fingertips_history[i].empty() && glm::length(offset - averaged_data_fingertip) > 4.0f) {
				data_fingertips_history[i] = std::queue<glm::vec3>();
				data_fingertips_sum[i] = glm::vec3(0);
			}

			/// Push new offset if new frame or replace last one if new iteration
			data_fingertips_history[i].push(offset);
			data_fingertips_sum[i] += offset;
			last_update_frame_id[i] = current_frame_id;

			if (data_fingertips_history[i].size() > settings->history_size) {
				data_fingertips_sum[i] -= data_fingertips_history[i].front();
				data_fingertips_history[i].pop();
			}
		}

		if (i == 0) cout << "i = 0, " << "last_update_frame_id[i]  = " << last_update_frame_id[i] << ", current_frame_id = " << current_frame_id << endl;
		if (!data_fingertips_history[i].empty() && current_frame_id - last_update_frame_id[i] < 3) {
			averaged_data_fingertips[i] = data_fingertips_sum[i] / (float)data_fingertips_history[i].size();
		}
	}

	//for (size_t i = 0; i < num_fingers; i++) std::cout << i << ": " << glm::to_string(ordered_data_fingertips[i]) << ", " << glm::to_string(averaged_data_fingertips[i]) << endl;

	ordered_data_fingertips = averaged_data_fingertips;

	if (last_frame_id != current_frame_id) 	last_frame_id = current_frame_id;
}

void Fingertips::track(LinearSystem & system, DataFrame & current_frame) {
	if (settings->enable_fingertips_prior == false) return; // || model->calibration_type == NONE) return;

	data_fingertips.clear();
	for (size_t i = 0; i < data_fingertips_2d.size(); i++) {
		unsigned short depth_value = current_frame.depth.at<unsigned short>(data_fingertips_2d[i][0], data_fingertips_2d[i][1]);
		if (depth_value == 0) continue;
		Vector3 data_point = camera->unproject(data_fingertips_2d[i][1], camera->height() - 1 - data_fingertips_2d[i][0], depth_value);
		data_fingertips.push_back(glm::vec3(data_point[0], data_point[1], data_point[2]));
	}

	ordered_data_fingertips = std::vector < glm::vec3>(num_fingers, glm::vec3(0));
	if (data_fingertips.empty()) {
		return;
	}
	model_fingertips = std::vector<glm::vec3>(num_fingers, glm::vec3(0));
	block_indices = std::vector<int>(num_fingers, -1);
	axis_projections = std::vector<glm::vec3>(num_fingers, glm::vec3(0));
	indices = std::vector<glm::ivec3>(num_fingers, glm::ivec3(0));

	/// Assing data fingertips to model fingertips		

	for (size_t i = 0; i < data_fingertips.size(); i++) {
		float min_distance = std::numeric_limits<float>::max();
		for (size_t j = 0; j < settings->fingertips_centers_indices.size(); j++) {
			float distance = abs(glm::length(data_fingertips[i] - model->centers[settings->fingertips_centers_indices[j]]) - model->radii[settings->fingertips_centers_indices[j]]);
			if (distance < min_distance) {
				min_distance = distance;
				glm::vec3 finger_tip_direction = (data_fingertips[i] - model->centers[settings->fingertips_centers_indices[j]]) / glm::length(data_fingertips[i] - model->centers[settings->fingertips_centers_indices[j]]);

				{ /// Check that data point does not project to the lower part of fingertip circle
					glm::vec3 finger_segment_direction = (model->centers[settings->fingertips_centers_indices[j]] - model->centers[settings->fingermiddles_centers_indices[j]]);
					if (glm::dot(finger_tip_direction, finger_segment_direction) < 0) {
						continue;
					}
				}

				model_fingertips[i] = model->centers[settings->fingertips_centers_indices[j]] + model->radii[settings->fingertips_centers_indices[j]] * finger_tip_direction;

				block_indices[i] = settings->fingertips_block_indices[j];
				axis_projections[i] = model->centers[settings->fingertips_centers_indices[j]];
				indices[i] = glm::ivec3(settings->fingertips_centers_indices[j], RAND_MAX, RAND_MAX);
			}
		}
	}

	ordered_model_fingertips = std::vector<glm::vec3>(num_fingers, glm::vec3(0));
	std::vector<int> ordered_block_indices = std::vector<int>(num_fingers, -1);
	std::vector<glm::vec3> ordered_axis_projections = std::vector<glm::vec3>(num_fingers, glm::vec3(0));
	std::vector<glm::ivec3> ordered_indices = std::vector<glm::ivec3>(num_fingers, glm::ivec3(0));

	std::vector<int> min_distance_indices = std::vector<int>(num_fingers, -1);
	for (size_t i = 0; i < block_indices.size(); i++) {
		for (size_t j = 0; j < num_fingers; j++) {
			if (block_indices[i] == settings->fingertips_block_indices[j]) {

				if (glm::length(ordered_data_fingertips[j]) < std::numeric_limits<float>::epsilon()) { // if this is empty
					ordered_data_fingertips[j] = data_fingertips[i];
					ordered_model_fingertips[j] = model_fingertips[i];
					ordered_block_indices[j] = block_indices[i];
					ordered_axis_projections[j] = axis_projections[i];
					ordered_indices[j] = indices[i];
					min_distance_indices[j] = i;
				}
				else {
					float previous_distance = glm::length(ordered_data_fingertips[j] - ordered_model_fingertips[j]);
					float new_distance = glm::length(data_fingertips[i] - model_fingertips[i]);
					if (new_distance < previous_distance) {
						ordered_data_fingertips[j] = data_fingertips[i];
						ordered_model_fingertips[j] = model_fingertips[i];
						ordered_block_indices[j] = block_indices[i];
						ordered_axis_projections[j] = axis_projections[i];
						ordered_indices[j] = indices[i];
						min_distance_indices[j] = i;
					}
				}
			}
		}
	}

	/// Average data fingertips positions over tips
	//moving_window_average(current_frame.id);

	/// Build linear system
	Eigen::Matrix<Scalar, num_fingers, 1> F = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_fingers, 1);
	Eigen::Matrix<Scalar, num_fingers, Eigen::Dynamic> J = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_fingers, num_thetas + model->num_betas);
	for (size_t i = 0; i < ordered_data_fingertips.size(); i++) {

		if (glm::length(ordered_data_fingertips[i]) < std::numeric_limits<float>::epsilon()) continue;

		glm::vec3 difference = ordered_model_fingertips[i] - ordered_data_fingertips[i];
		glm::vec3 modified_difference = glm::vec3(difference[0], difference[1], 0);
		//glm::vec3 normal = difference / glm::length(difference);
		glm::vec3 normal = modified_difference / glm::length(modified_difference);

		if (glm::length(difference) > 10.0f ) continue;

		F[i] = glm::dot(difference, normal);
		size_t pose_unit_id = model->blockid_to_pose_unit_id_map[ordered_block_indices[i]];
		size_t shape_unit_id = model->blockid_to_shape_unit_id_map[ordered_block_indices[i]];

		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> J_theta = model->jacobian_theta_new(pose_unit_id, ordered_model_fingertips[i], ordered_axis_projections[i], ordered_indices[i]);
		Eigen::Matrix<Scalar, 3, Eigen::Dynamic> J_beta;
		if (model->num_betas > 0)  J_beta = model->jacobian_beta(shape_unit_id, ordered_model_fingertips[i], ordered_axis_projections[i], ordered_indices[i]);

		for (size_t j = 0; j < num_thetas; j++) {
			Vector3 j_column = J_theta.col(j);
			if (j_column.norm() > 0) {
				J(i, j) = j_column.dot(Vector3(normal[0], normal[1], normal[2]));
			}
		}
		for (size_t j = 0; j < model->num_betas; j++) {
			Vector3 j_column = J_beta.col(j);
			if (j_column.norm() > 0) {
				J(i, num_thetas + j) = j_column.dot(Vector3(normal[0], normal[1], normal[2]));
			}
		}
	}

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JT = J.transpose();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> LHS = JT * J;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1>  rhs = (-JT * F);

	if (system.has_nan()) cout << "of before the Fingertips term" << endl;

	settings->weight = 250;
	system.lhs += 250 * LHS;
	system.rhs += 250 * rhs;

	if (system.has_nan()) cout << "of the Fingertips term" << endl;
}