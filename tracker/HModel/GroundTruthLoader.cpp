#include "tracker/HModel/GroundTruthLoader.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/HModel/Model.h"
#include "tracker/Data/Camera.h"
#include "util/opencv_wrapper.h"
#include "cudax/cuda_glm.h"
#include "glm/gtx/string_cast.hpp"

#include <numeric>

Vector3 GroundTruthLoader::point_at_depth_pixel(int z, int x, int y, Camera* camera) {
	return camera->depth_to_world(x, y, z);
}

GroundTruthLoader::GroundTruthLoader(DatasetType dataset_type, bool fit_wrist_separatedly, std::string sequence_path) {
	this->dataset_type = dataset_type;
	this->fit_wrist_separatedly = fit_wrist_separatedly;
	this->sequence_path = sequence_path;
	if (dataset_type == TOMPSON) {
		this->sequence_path = "C:/Data/sensor-sequences/tompson/";
		this->num_markers = 36;
	}
	if (dataset_type == TKACH) {
		this->num_markers = 25;
	}
	if (dataset_type == SRIDHAR) {
		this->sequence_path = "C:/Data/datasets/dexter1/data/ALL/";
		this->num_markers = 6;
	}
}

void GroundTruthLoader::load_ground_truth_marker_positions() {

	int max_num_frames = 9000;
	std::string ground_truth_path = sequence_path + "marker_positions.txt";

	FILE * file_pointer = fopen((ground_truth_path).c_str(), "r");
	if (file_pointer == NULL) {
		//cout << "can't open ground truth file" << endl;
		return;
	}

	int N; fscanf(file_pointer, "%d", &N);
	for (int frame_index = 0; frame_index < N && frame_index < max_num_frames; frame_index++) {
		std::vector<float> marker_positions_entry;
		for (int i = 0; i < 3 * num_markers; i++) {
			float value; fscanf(file_pointer, "%f", &value);
			marker_positions_entry.push_back(value);
		}
		marker_positions.push_back(marker_positions_entry);
	}
	fclose(file_pointer);
}

void GroundTruthLoader::get_sensor_silhouette(const DataFrame & current_frame, Camera * camera, cv::Mat & sensor_silhouette, cv::Mat & sensor_silhouette_wrist) {

	if (marker_positions.size() <= current_frame.id) {
		cout << "no groundtruth is provided for the current frame" << endl;
		return;
	}
	sensor_silhouette = cv::Mat::zeros(current_frame.depth.rows, current_frame.depth.cols, CV_8UC1);
	if (fit_wrist_separatedly ) sensor_silhouette_wrist = cv::Mat::zeros(current_frame.depth.rows, current_frame.depth.cols, CV_8UC1);
	float depth_range = 150;
	int palm_center_index = 17; // 34
	Vector3 palm_center = Vector3(
		marker_positions[current_frame.id][3 * palm_center_index],
		marker_positions[current_frame.id][3 * palm_center_index + 1],
		marker_positions[current_frame.id][3 * palm_center_index + 2]);
	float palm_center_depth = palm_center[2];
	cv::inRange(current_frame.depth, palm_center_depth - depth_range, palm_center_depth + depth_range, sensor_silhouette);

	/// Find the range
	float min_x = std::numeric_limits<float>::max();
	float max_x = -std::numeric_limits<float>::max();
	float min_y = std::numeric_limits<float>::max();
	float max_y = -std::numeric_limits<float>::max();
	for (size_t i = 0; i < num_markers; i++) {
		if (i == 30 || i == 31 || i == 35) continue;
		float x = marker_positions[current_frame.id][3 * i];
		float y = marker_positions[current_frame.id][3 * i + 1];
		if (x < min_x) min_x = x;
		if (x > max_x) max_x = x;
		if (y < min_y) min_y = y;
		if (y > max_y) max_y = y;
	}

	/// Find the range for wrist
	float min_x_wrist = std::numeric_limits<float>::max();
	float max_x_wrist = -std::numeric_limits<float>::max();
	float min_y_wrist = std::numeric_limits<float>::max();
	float max_y_wrist = -std::numeric_limits<float>::max();
	for (size_t i = 0; i < num_markers; i++) {
		if (i != 30 && i != 31 && i != 35) continue;
		float x = marker_positions[current_frame.id][3 * i];
		float y = marker_positions[current_frame.id][3 * i + 1];
		if (x < min_x_wrist) min_x_wrist = x;
		if (x > max_x_wrist) max_x_wrist = x;
		if (y < min_y_wrist) min_y_wrist = y;
		if (y > max_y_wrist) max_y_wrist = y;
	}

	/// Only keep the pixels within the range;
	float offset = 23; float offset_wrist = 40;
	if (fit_wrist_separatedly) offset = 18;
	for (int row = 0; row < sensor_silhouette.rows; ++row) {
		for (int col = 0; col < sensor_silhouette.cols; ++col) {
			if (sensor_silhouette.at<uchar>(row, col) != 255) continue;
			int z = current_frame.depth.at<unsigned short>(row, col);
			Vector3 world = camera->depth_to_world(col, row, z);
			if (world[0] >= min_x - offset && world[0] <= max_x + offset && world[1] >= min_y - offset && world[1] <= max_y + offset)
				sensor_silhouette.at<uchar>(row, col) = 255;
			else {
				sensor_silhouette.at<uchar>(row, col) = 0;
				if (fit_wrist_separatedly) {
					if (world[0] >= min_x_wrist - offset_wrist && world[0] <= max_x_wrist + offset_wrist && world[1] >= min_y_wrist - offset_wrist && world[1] <= max_y_wrist + offset_wrist) {
						sensor_silhouette.at<uchar>(row, col) = 255;
						sensor_silhouette_wrist.at<uchar>(row, col) = 255;
					}					
				}
					
			}
		}
	}
}

std::vector<Vector3> GroundTruthLoader::get_ground_truth_marker_positions(int frame_id) {
	if (marker_positions.size() <= frame_id) {
		cout << "no groundtruth available for this frame" << endl;
		return std::vector<Vector3>();
	}
	std::vector<Vector3> marker_positions_entry;

	for (int i = 0; i < num_markers; i++) {
		marker_positions_entry.push_back(Vector3(marker_positions[frame_id][3 * i], marker_positions[frame_id][3 * i + 1], marker_positions[frame_id][3 * i + 2]));
	}

	if (dataset_type == SRIDHAR) {

		Matrix3 projection_matrix = camera->projection_matrix();
		for (int i = 0; i < num_markers; i++) {

			Vector3 p = marker_positions_entry[i];
			float depth = p[2];
			float x = p[0] / depth;
			float y = p[1] / depth;

			x = x * projection_matrix(0, 0) + projection_matrix(0, 2);
			y = y * projection_matrix(1, 1) + projection_matrix(1, 2);

			marker_positions_entry[i] = camera->inv_projection_matrix() * Vector3((camera->width() - x) * depth, y * depth, depth);
		}
	}

	return marker_positions_entry;
}

void GroundTruthLoader::write_model_marker_positions(const std::vector<Vector3> & model_marker_positions) {
	std::string marker_positions_filename = sequence_path + "marker_positions.txt";
	static ofstream marker_positions_file(marker_positions_filename);
	if (marker_positions_file.is_open()) {
		for (size_t i = 0; i < model_marker_positions.size(); i++) {
			marker_positions_file << model_marker_positions[i][0] << " " << model_marker_positions[i][1] << " " << model_marker_positions[i][2] << " ";			
		}
		marker_positions_file << endl;
	}
}

void GroundTruthLoader::write_marker_based_metrics(const std::vector<Vector3> & model_marker_positions, std::string marker_based_metrics_filename, int frame_id) {

	std::vector<Vector3> grount_truth_marker_positions = get_ground_truth_marker_positions(frame_id);

	if (!marker_based_metrics_file.is_open()) {
		marker_based_metrics_file = ofstream(marker_based_metrics_filename);
	}
	if (marker_based_metrics_file.is_open()) {
		for (size_t i = 0; i < grount_truth_marker_positions.size(); i++) {
			marker_based_metrics_file << (grount_truth_marker_positions[i] - model_marker_positions[i]).norm() << " ";
		}
		marker_based_metrics_file << endl;
	}
}

void GroundTruthLoader::get_marker_positions_tompson_finger(std::vector<std::string> centers_names, std::vector<size_t> centers_block_indices, float base_offset, bool is_thumb,
	std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices) {
	size_t num_markers = 5;
	size_t num_phalanges = 3;
	size_t num_centers = 4;

	auto project_point_on_segment = [](const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2) {
		glm::dvec3 u = c2 - c1;
		glm::dvec3 v = p - c1;
		double alpha = dot(u, v) / dot(u, u);
		if (alpha <= 0) return c1;
		if (alpha > 0 && alpha < 1) return c1 + alpha * u;
		if (alpha >= 1) return c2;
	};

	glm::vec3 membrane_center = model->centers[model->centers_name_to_id_map[centers_names[0]]];
	glm::vec3 base_center = model->centers[model->centers_name_to_id_map[centers_names[1]]];
	glm::vec3 bottom_center = model->centers[model->centers_name_to_id_map[centers_names[2]]];
	glm::vec3 middle_center = model->centers[model->centers_name_to_id_map[centers_names[3]]];
	glm::vec3 top_center = model->centers[model->centers_name_to_id_map[centers_names[4]]];
	glm::vec3 membrane_projection = project_point_on_segment(membrane_center, base_center, bottom_center);

	top_center += model->radii[model->centers_name_to_id_map[centers_names[4]]] * (top_center - middle_center) / glm::length(top_center - middle_center);
	base_center += base_offset * (bottom_center - base_center) / glm::length(bottom_center - base_center);


	std::vector<glm::vec3> centers;
	if (is_thumb) centers = { base_center, bottom_center, middle_center, top_center };
	else centers = { membrane_projection, bottom_center, middle_center, top_center };

	std::vector<float> phalange_lengths = std::vector<float>(num_phalanges, 0);
	std::vector<float> center_offsets = std::vector<float>(num_centers, 0);
	std::vector<glm::vec3> phalange_directions = std::vector<glm::vec3>(num_phalanges, glm::vec3());
	for (size_t i = 0; i < centers.size() - 1; i++) {
		phalange_lengths[i] = glm::length(centers[i + 1] - centers[i]);
		phalange_directions[i] = (centers[i + 1] - centers[i]) / phalange_lengths[i];
		center_offsets[i + 1] = center_offsets[i] + phalange_lengths[i];
	}

	float total_length = std::accumulate(phalange_lengths.begin(), phalange_lengths.end(), 0);
	float markers_initial_offset = total_length / 6;
	float markers_interval = (total_length - markers_initial_offset) / (num_markers - 1);

	for (size_t i = 0; i < num_markers; i++) {
		float marker_offset = markers_initial_offset + markers_interval * i;
		for (size_t j = 0; j < center_offsets.size() - 1; j++) {
			if ((marker_offset > center_offsets[j] && marker_offset < center_offsets[j + 1]) || abs(marker_offset - center_offsets[j]) < 1e-7) {
				float marker_offset_remainder = marker_offset - center_offsets[j];
				glm::vec3 marker_position = centers[j] + marker_offset_remainder * phalange_directions[j];
				marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
				marker_block_indices.push_back(centers_block_indices[j]);
				break;
			}
		}
	}
	std::reverse(marker_positions.begin(), marker_positions.end());
	std::reverse(marker_block_indices.begin(), marker_block_indices.end());

	marker_positions.push_back(Vector3(base_center[0], base_center[1], base_center[2]));
	marker_block_indices.push_back(centers_block_indices[0]);

}

void GroundTruthLoader::get_marker_positions_tompson(std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices) {

	std::vector<Vector3> pinky_marker_positions; std::vector<size_t> pinky_marker_block_indices;
	get_marker_positions_tompson_finger({ "pinky_membrane", "pinky_base", "pinky_bottom", "pinky_middle", "pinky_top" }, { 2, 1, 0 }, 12, false, pinky_marker_positions, pinky_marker_block_indices);
	std::vector<Vector3> ring_marker_positions; std::vector<size_t> ring_marker_block_indices;
	get_marker_positions_tompson_finger({ "ring_membrane", "ring_base", "ring_bottom", "ring_middle", "ring_top" }, { 5, 4, 3 }, 25, false, ring_marker_positions, ring_marker_block_indices);
	std::vector<Vector3> middle_marker_positions; std::vector<size_t> middle_marker_block_indices;
	get_marker_positions_tompson_finger({ "middle_membrane", "middle_base", "middle_bottom", "middle_middle", "middle_top" }, { 8, 7, 6 }, 20, false, middle_marker_positions, middle_marker_block_indices);
	std::vector<Vector3> index_marker_positions; std::vector<size_t> index_marker_block_indices;
	get_marker_positions_tompson_finger({ "index_membrane", "index_base", "index_bottom", "index_middle", "index_top" }, { 11, 10, 9 }, 5, false, index_marker_positions, index_marker_block_indices);
	std::vector<Vector3> thumb_marker_positions; std::vector<size_t> thumb_marker_block_indices;
	get_marker_positions_tompson_finger({ "thumb_membrane", "thumb_base", "thumb_bottom", "thumb_middle", "thumb_additional" }, { 14, 13, 27 }, 0, true, thumb_marker_positions, thumb_marker_block_indices);

	marker_positions.insert(std::end(marker_positions), std::begin(pinky_marker_positions), std::end(pinky_marker_positions));
	marker_positions.insert(std::end(marker_positions), std::begin(ring_marker_positions), std::end(ring_marker_positions));
	marker_positions.insert(std::end(marker_positions), std::begin(middle_marker_positions), std::end(middle_marker_positions));
	marker_positions.insert(std::end(marker_positions), std::begin(index_marker_positions), std::end(index_marker_positions));
	marker_positions.insert(std::end(marker_positions), std::begin(thumb_marker_positions), std::end(thumb_marker_positions));

	marker_block_indices.insert(std::end(marker_block_indices), std::begin(pinky_marker_block_indices), std::end(pinky_marker_block_indices));
	marker_block_indices.insert(std::end(marker_block_indices), std::begin(ring_marker_block_indices), std::end(ring_marker_block_indices));
	marker_block_indices.insert(std::end(marker_block_indices), std::begin(middle_marker_block_indices), std::end(middle_marker_block_indices));
	marker_block_indices.insert(std::end(marker_block_indices), std::begin(index_marker_block_indices), std::end(index_marker_block_indices));
	marker_block_indices.insert(std::end(marker_block_indices), std::begin(thumb_marker_block_indices), std::end(thumb_marker_block_indices));

	auto convert_eigen_matrix_to_glm_matrix = [](const Mat3f & eigen_matrix) {
		glm::mat3 glm_matrix = glm::mat3();
		for (size_t u = 0; u < 3; u++) {
			for (size_t v = 0; v < 3; v++) {
				glm_matrix[u][v] = eigen_matrix(v, u);
			}
		}
		return glm_matrix;
	};

	glm::vec3 marker_position;
	glm::mat3 global = convert_eigen_matrix_to_glm_matrix(model->phalanges[0].global.block(0, 0, 3, 3));

	/// Marker 30 - palm right
	marker_position = model->centers[model->centers_name_to_id_map["palm_right"]] + global * model->radii[model->centers_name_to_id_map["palm_right"]] * glm::vec3(0, -1, 0.5);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(16);

	/// Marker 31 - palm left
	marker_position = model->centers[model->centers_name_to_id_map["palm_left"]] + global * model->radii[model->centers_name_to_id_map["palm_left"]] * glm::vec3(0, -1, 0.5);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(18);

	/// Marker 32 - palm middle
	marker_position = model->centers[model->centers_name_to_id_map["palm_middle"]] + global * model->radii[model->centers_name_to_id_map["palm_middle"]] * glm::vec3(0, 0, 0.5);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(17);

	/// Marker 33 - palm ring
	marker_position = model->centers[model->centers_name_to_id_map["palm_ring"]] + global * model->radii[model->centers_name_to_id_map["palm_ring"]] * glm::vec3(0, 0, 0.5);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(16);

	/// Marker 34 - palm center
	marker_position = 0.29f * model->centers[model->centers_name_to_id_map["palm_middle"]] + 0.25f * model->centers[model->centers_name_to_id_map["palm_ring"]] + 0.46f * model->centers[model->centers_name_to_id_map["palm_back"]] +
		global * model->radii[model->centers_name_to_id_map["palm_back"]] * glm::vec3(0, 0, 0.8);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(17);

	/// Marker 35 - wrist center
	marker_position = model->centers[model->centers_name_to_id_map["palm_back"]] + global * model->radii[model->centers_name_to_id_map["palm_back"]] * glm::vec3(0, -1.0, 0.5);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(17);
}

void GroundTruthLoader::get_marker_positions_sridhar(std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices) {

	auto convert_eigen_matrix_to_glm_matrix = [](const Mat3f & eigen_matrix) {
		glm::mat3 glm_matrix = glm::mat3();
		for (size_t u = 0; u < 3; u++) {
			for (size_t v = 0; v < 3; v++) {
				glm_matrix[u][v] = eigen_matrix(v, u);
			}
		}
		return glm_matrix;
	};

	glm::vec3 marker_position;
	glm::mat3 global = convert_eigen_matrix_to_glm_matrix(model->phalanges[0].global.block(0, 0, 3, 3));

	marker_position = model->centers[model->centers_name_to_id_map["thumb_additional"]];
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(27);

	marker_position = model->centers[model->centers_name_to_id_map["index_top"]];
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(9);

	marker_position = model->centers[model->centers_name_to_id_map["middle_top"]];
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(6);

	marker_position = model->centers[model->centers_name_to_id_map["ring_top"]];
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(3);

	marker_position = model->centers[model->centers_name_to_id_map["pinky_top"]];
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(0);

	marker_position = 0.5f * model->centers[model->centers_name_to_id_map["middle_base"]] + 0.5f * model->centers[model->centers_name_to_id_map["ring_base"]] +
		global * model->radii[model->centers_name_to_id_map["palm_middle"]] * glm::vec3(0, 0, -1);
	marker_positions.push_back(Vector3(marker_position[0], marker_position[1], marker_position[2]));
	marker_block_indices.push_back(17);
}

void GroundTruthLoader::get_marker_positions(std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices) {
	if (dataset_type == TKACH || dataset_type == TOMPSON) {
		get_marker_positions_tompson(marker_positions, marker_block_indices);
	}
	if (dataset_type == SRIDHAR) {
		get_marker_positions_sridhar(marker_positions, marker_block_indices);
	}
}

void GroundTruthLoader::track(LinearSystem & system, DataFrame & current_frame) {

	if (enable_ground_truth_reinit == false) return;
	//cout << "running reinitialization" << endl;

	std::vector<Vector3> model_marker_positions; std::vector<size_t> model_marker_block_indices;
	get_marker_positions(model_marker_positions, model_marker_block_indices);

	std::vector<Vector3> ground_truth_marker_positions;
	ground_truth_marker_positions = get_ground_truth_marker_positions(current_frame.id);

	active_model_points.clear(); active_data_points.clear();

	/// Build linear system
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> F = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(model_marker_positions.size(), 1);
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> J = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(model_marker_positions.size(), num_thetas + model->num_betas);
	for (size_t i = 0; i < model_marker_positions.size(); i++) {	

		if ( (dataset_type == TOMPSON  && i == 0 || i == 16 || i == 12 || i == 18 || i == 24 || i == 30 || i == 31 || i == 34) ||
			dataset_type == SRIDHAR) {

			glm::vec3 model_point = glm::vec3(model_marker_positions[i][0], model_marker_positions[i][1], model_marker_positions[i][2]);
			glm::vec3 data_point = glm::vec3(ground_truth_marker_positions[i][0], ground_truth_marker_positions[i][1], ground_truth_marker_positions[i][2]);
			glm::vec3 difference = model_point - data_point;
			glm::vec3 normal = difference / glm::length(difference);

			float weight = 1.0f;

			float d = glm::length(difference);
			float w = 1.0f / glm::sqrt(d + 1e-3);
			if (d > 1e-3) weight *= w * 3.5f;

			//if (glm::length(difference) > 50.0f) continue;
			/*cout << i << endl;
			cout << "model_point = " << glm::to_string(model_point) << endl;
			cout << "data_point = " << glm::to_string(data_point) << endl;
			cout << "block_index = " << model_marker_block_indices[i] << endl;
			cout << "block = " << glm::to_string(model->blocks[model_marker_block_indices[i]]) << endl;
			cout << endl;*/

			///
			active_model_points.push_back(model_marker_positions[i]);
			active_data_points.push_back(ground_truth_marker_positions[i]);
			///

			F[i] = weight * glm::dot(difference, normal);
			size_t pose_unit_id = model->blockid_to_pose_unit_id_map[model_marker_block_indices[i]];
			size_t shape_unit_id = model->blockid_to_shape_unit_id_map[model_marker_block_indices[i]];

			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> J_theta = model->jacobian_theta_new(pose_unit_id, model_point, model_point, model->blocks[model_marker_block_indices[i]]);
			Eigen::Matrix<Scalar, 3, Eigen::Dynamic> J_beta;
			//if (model->num_betas > 0)  J_beta = model->jacobian_beta(shape_unit_id, model_point, model_point, model->blocks[model_marker_block_indices[i]]);

			for (size_t j = 0; j < num_thetas; j++) {
				Vector3 j_column = J_theta.col(j);
				if (j_column.norm() > 0) {
					J(i, j) = weight * j_column.dot(Vector3(normal[0], normal[1], normal[2]));
				}
			}
			/*for (size_t j = 0; j < model->num_betas; j++) {
				Vector3 j_column = J_beta.col(j);
				if (j_column.norm() > 0) {
					J(i, num_thetas + j) = weight * j_column.dot(Vector3(normal[0], normal[1], normal[2]));
				}
			}*/
		}
	}

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> JT = J.transpose();
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> LHS = JT * J;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1>  rhs = (-JT * F);

	if (system.has_nan()) cout << "of before the GroundTruth term" << endl;

	system.lhs += 500 * LHS;
	system.rhs += 500 * rhs;

	if (system.has_nan()) cout << "of the GroundTruth term" << endl;
}
