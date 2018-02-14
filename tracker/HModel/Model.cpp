#include "Model.h"
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include <numeric>
#include <Eigen/Geometry>
#include "DataLoader.h"
#include "ModelSemantics.h";
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"
#include "tracker/Data/Camera.h"

#include "glm/gtx/string_cast.hpp"
#include <random>

Model::Model() :outline_finder(this), serializer(this), semantics(this) {

}

Model::~Model() {
	if (host_pointer_centers) delete[] host_pointer_centers;
	if (host_pointer_radii) delete[] host_pointer_radii;
	if (host_pointer_blocks) delete[] host_pointer_blocks;
	if (host_pointer_tangent_points) delete[] host_pointer_tangent_points;
	if (host_pointer_outline) delete[] host_pointer_outline;
	if (host_pointer_blockid_to_pose_unit_id_map) delete[] host_pointer_blockid_to_pose_unit_id_map;
	if (host_pointer_blockid_to_shape_unit_id_map) delete[] host_pointer_blockid_to_shape_unit_id_map;
	if (rendered_pixels) delete[] rendered_pixels;
	if (rendered_points) delete[] rendered_points;
	if (rendered_normals) delete[] rendered_normals;
	if (rendered_block_ids) delete[] rendered_block_ids;
}

void Model::init(std::string data_path, std::string sequence_path, std::string sequence_name, std::string calibrated_model_path, CalibrationType calibration_type, CalibrationStage calibration_stage,
	bool load_calibrated_model, bool enable_shape_prior, bool enable_shape_dofs_blockers, bool fit_wrist_separately, bool show_initialization_calls) {
	this->calibration_type = calibration_type;
	this->enable_shape_prior = enable_shape_prior;
	this->enable_shape_dofs_blockers = enable_shape_dofs_blockers;
	this->fit_wrist_separately = fit_wrist_separately;
	this->show_initialization_calls = show_initialization_calls;

	semantics.setup_calibration_type_to_num_betas_map();
	semantics.setup_calibration_type_to_beta_template_map();
	num_betas = calibration_type_to_num_betas_map[calibration_type];
	if (show_initialization_calls) cout << "num_betas = " << num_betas << endl;
	num_parameters = num_thetas + num_betas;

	centers = std::vector<glm::vec3>(num_centers, glm::vec3());
	radii = std::vector<float>(num_centers, 0);
	blocks = std::vector<glm::ivec3>(num_blocks, glm::ivec3());
	tangent_points = std::vector<Tangent>(num_blocks, Tangent());

	theta = std::vector<float>(num_thetas, 0);
	beta = std::vector<float>(num_betas, 0);
	beta_latent = std::vector<float>(num_betas_latent, 1);
	beta_certainty = std::vector<float>(num_betas, 0);
	phalanges = std::vector<Phalange>(num_phalanges + 1, Phalange());
	pose_dofs = std::vector<PoseDof>(num_thetas, PoseDof());
	shape_dofs = std::vector<ShapeDof>(num_betas, ShapeDof());

	host_pointer_centers = NULL;
	host_pointer_radii = NULL;
	host_pointer_blocks = NULL;
	host_pointer_tangent_points = NULL;
	host_pointer_outline = NULL;
	host_pointer_blockid_to_pose_unit_id_map = NULL;
	host_pointer_blockid_to_shape_unit_id_map = NULL;

	camera_ray = glm::vec3(0, 0, 1);

	rendered_pixels = new int[upper_bound_num_rendered_outline_points];
	rendered_points = new float[3 * upper_bound_num_rendered_outline_points];
	rendered_normals = new float[2 * upper_bound_num_rendered_outline_points];
	rendered_block_ids = new int[upper_bound_num_rendered_outline_points];

	this->data_path = data_path;
	this->sequence_path = sequence_path;
	this->sequence_name = sequence_name;

	if (show_initialization_calls) cout << "run theta setups" << endl;
	semantics.setup_blockid_to_pose_unit_id_map();
	semantics.setup_centers_name_to_id_map();
	semantics.setup_attachments();
	semantics.setup_jointid_to_centerid_map();

	semantics.setup_outline();
	semantics.setup_phalanges();

	semantics.setup_blockid_to_phalangeid_map();
	semantics.setup_jointid_to_phalangeid_map();
	semantics.setup_pose_dofs();
	semantics.setup_thetas_limits();
	semantics.setup_pose_units_and_kinematic_chains();

	if (show_initialization_calls) cout << "run beta setups" << endl;
	semantics.setup_blockid_to_shape_unit_id_map();
	semantics.setup_shape_dofs();
	semantics.setup_betas_limits();
	semantics.setup_shape_units_and_shape_chains();

	if (show_initialization_calls) cout << "load the model" << endl;
	
	string model_path = ""; 
	if (load_calibrated_model) model_path = calibrated_model_path;
	load_model_from_file(load_calibrated_model, model_path);

	update_parameters(std::vector<float>(num_thetas, 0));
	initialize_offsets();

	/// > Initialize templates
	beta_template = calibration_type_to_beta_template_map[calibration_type];

	phalanges_template = phalanges;
	for (size_t i = 0; i < num_phalanges; i++) {
		if (phalanges[i].attachments.empty()) continue;
		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			Vec3f c = Eigen::Vector3f(centers_template[phalanges[i].attachments[j]][0], centers_template[phalanges[i].attachments[j]][1], centers_template[phalanges[i].attachments[j]][2]);
			Mat3d inverse = (phalanges[i].global.block(0, 0, 3, 3).cast<double>()).inverse();
			Vec3d offset = inverse *  (c - phalanges[i].global.block(0, 3, 3, 1)).cast<double>();
			phalanges_template[i].offsets[j] = offset;
		}
	}

	semantics.setup_semantic_limits();

	if (show_initialization_calls) cout << "finished initializing model" << endl;
	/*for (size_t i = 0; i < num_phalanges; i++) {
	Phalange phalange = phalanges[i];
	cout << "id = " << i << endl;
	cout << "init_local: " << endl << phalange.init_local << endl;
	cout << endl << endl;
	}*/
	//print_model();
}

void Model::change_calibration_type(CalibrationType new_calibration_type) {
	CalibrationType old_calibration_type = this->calibration_type;

	this->calibration_type = new_calibration_type;
	num_betas = calibration_type_to_num_betas_map[new_calibration_type];
	num_parameters = num_thetas + num_betas;

	shape_dofs = std::vector<ShapeDof>(num_betas, ShapeDof());

	host_pointer_blockid_to_shape_unit_id_map = NULL;

	semantics.setup_blockid_to_shape_unit_id_map();
	semantics.setup_shape_dofs();
	semantics.setup_betas_limits();
	semantics.setup_semantic_limits();
	semantics.setup_shape_units_and_shape_chains();

	beta.resize(num_betas);
	if (num_betas > 0) {
		beta = get_beta();
		update_beta(beta);
	}
	update_theta(get_theta());
	update_centers();
	compute_outline();
}

void Model::initialize_offsets(bool rewrite_thumb) {
	bool print = false;
	for (size_t i = 0; i < num_phalanges; i++) {
		if (print) cout << endl << i << endl;

		if (phalanges[i].attachments.empty()) continue;
		phalanges[i].offsets.resize(phalanges[i].attachments.size());
		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			Vec3f c = Eigen::Vector3f(centers[phalanges[i].attachments[j]][0], centers[phalanges[i].attachments[j]][1], centers[phalanges[i].attachments[j]][2]);
			Mat3d inverse = (phalanges[i].global.block(0, 0, 3, 3).cast<double>()).inverse();
			Vec3d offset = inverse *  (c - phalanges[i].global.block(0, 3, 3, 1)).cast<double>();
			phalanges[i].offsets[j] = offset;

			if (calibration_type == FINGERS_AND_PALM || calibration_type == FULL) { // put all palm centers to a plane
				if (i == 0) phalanges[0].offsets[j][2] = 0;
				if (i == 4 || i == 7 || i == 10 || i == 13) phalanges[i].init_local(2, 3) -= 3;
			}

			if (print) {
				cout << "attachment_id = " << phalanges[i].attachments[j] << endl;
				cout << " center = " << centers[phalanges[i].center_id][0] << ", " << centers[phalanges[i].center_id][1] << ", " << centers[phalanges[i].center_id][2] << endl;
				cout << "R = " << phalanges[i].global.block(0, 0, 3, 3) << endl;
				cout << " offset = " << phalanges[i].offsets[j][0] << ", " << phalanges[i].offsets[j][1] << ", " << phalanges[i].offsets[j][2] << endl;
			}
		}
	}

	// replace by something more elegant
	Vec3d palm_index_offset = phalanges[0].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_index"]]];
	Vec3d palm_middle_offset = phalanges[0].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_middle"]]];
	Vec3d palm_ring_offset = phalanges[0].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_ring"]]];
	Vec3d palm_pinky_offset = phalanges[0].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_pinky"]]];
	float palm_width = (palm_index_offset - palm_pinky_offset).norm();
	palm_middle_offset_ratio = (palm_index_offset - palm_middle_offset).norm() / palm_width;
	palm_ring_offset_ratio = (palm_index_offset - palm_ring_offset).norm() / palm_width;

	if (rewrite_thumb) {
		phalanges[phalanges_name_to_id_map["HandThumb2"]].offsets[0] = Eigen::Vector3d(0.0, 12.0, 0.0);
		phalanges[phalanges_name_to_id_map["HandThumb3"]].offsets[0][0] = 0;
		phalanges[phalanges_name_to_id_map["HandThumb3"]].offsets[1][0] = 0;
	}
}

void Model::reindex() {
	for (size_t i = 0; i < blocks.size(); i++) {
		if (blocks[i][2] == RAND_MAX) {
			if (radii[blocks[i][0]] < radii[blocks[i][1]]) {
				std::swap(blocks[i][0], blocks[i][1]);
			}
		}
		else {
			if (radii[blocks[i][0]] <= radii[blocks[i][1]] && radii[blocks[i][1]] <= radii[blocks[i][2]]) {
				blocks[i] = glm::ivec3(blocks[i][2], blocks[i][1], blocks[i][0]);
			}
			if (radii[blocks[i][0]] <= radii[blocks[i][2]] && radii[blocks[i][2]] <= radii[blocks[i][1]]) {
				blocks[i] = glm::ivec3(blocks[i][1], blocks[i][2], blocks[i][0]);
			}
			if (radii[blocks[i][1]] <= radii[blocks[i][0]] && radii[blocks[i][0]] <= radii[blocks[i][2]]) {
				blocks[i] = glm::ivec3(blocks[i][2], blocks[i][0], blocks[i][1]);
			}
			if (radii[blocks[i][1]] <= radii[blocks[i][2]] && radii[blocks[i][2]] <= radii[blocks[i][0]]) {
				blocks[i] = glm::ivec3(blocks[i][0], blocks[i][2], blocks[i][1]);
			}
			if (radii[blocks[i][2]] <= radii[blocks[i][0]] && radii[blocks[i][0]] <= radii[blocks[i][1]]) {
				blocks[i] = glm::ivec3(blocks[i][1], blocks[i][0], blocks[i][2]);
			}
		}
	}
}

void Model::compute_outline() {
	outline_finder.find_outline();
	//outline_finder.write_outline();
	//outline_finder.compute_projections_outline(centers, radii, data_points, camera_ray);
}

void Model::compute_tangent_point(const glm::vec3 & camera_ray, const glm::vec3 & c1, const glm::vec3 & c2, const glm::vec3 & c3, float r1, float r2, float r3,
	glm::vec3 & v1, glm::vec3 & v2, glm::vec3 & v3, glm::vec3 & u1, glm::vec3 & u2, glm::vec3 & u3, glm::vec3 & n, glm::vec3 & m) {

	/*std::cout << "c1 = (" << c1[0] << ", " << c1[1] << ", " << c1[2] << ")" << std::endl;
	std::cout << "c2 = (" << c2[0] << ", " << c2[1] << ", " << c2[2] << ")" << std::endl;
	std::cout << "c3 = (" << c3[0] << ", " << c3[1] << ", " << c3[2] << ")" << std::endl;
	std::cout << "r1 = " << r1 << std::endl;
	std::cout << "r2 = " << r2 << std::endl;
	std::cout << "r3 = " << r3 << std::endl;*/

	float epsilon = 1e-2;
	if (r1 - r2 < epsilon && r1 - r3 < epsilon) {
		n = cross(c1 - c2, c1 - c3); n = n / length(n);
		if (dot(camera_ray, n) < 0) {
			m = -n;
		}
		else {
			m = n;
			n = -m;
		}
		v1 = c1 + r1 * n;
		v2 = c2 + r2 * n;
		v3 = c3 + r3 * n;
		u1 = c1 + r1 * m;
		u2 = c2 + r2 * m;
		u3 = c3 + r3 * m;
		/*std::cout << "n = (" << n[0] << ", " << n[1] << ", " << n[2] << ")" << std::endl;
		std::cout << "v1 = (" << v1[0] << ", " << v1[1] << ", " << v1[2] << ")" << std::endl;
		std::cout << "v2 = (" << v2[0] << ", " << v2[1] << ", " << v2[2] << ")" << std::endl;
		std::cout << "v3 = (" << v3[0] << ", " << v3[1] << ", " << v3[2] << ")" << std::endl;
		std::cout << std::endl << std::endl;*/
		return;
	}

	glm::vec3 z12 = c1 + (c2 - c1) * r1 / (r1 - r2);
	glm::vec3 z13 = c1 + (c3 - c1) * r1 / (r1 - r3);

	glm::vec3 l = (z12 - z13) / length(z12 - z13);
	float projection = dot(c1 - z12, l);
	glm::vec3 z = z12 + projection * l;

	float eta = length(c1 - z);
	float sin_beta = r1 / eta;
	float nu = sqrt(eta * eta - r1 * r1);
	float cos_beta = nu / eta;

	glm::vec3 f = (c1 - z) / eta;
	glm::vec3 h = cross(l, f);
	normalize(h);

	glm::vec3 g;

	g = sin_beta * h + cos_beta * f;
	v1 = z + nu * g;
	n = (v1 - c1) / length(v1 - c1);
	v2 = c2 + r2 * n;
	v3 = c3 + r3 * n;

	g = -sin_beta * h + cos_beta * f;
	u1 = z + nu * g;
	m = (u1 - c1) / length(u1 - c1);
	u2 = c2 + r2 * m;
	u3 = c3 + r3 * m;

	if (dot(camera_ray, n) > 0) {
		std::swap(v1, u1);
		std::swap(v2, u2);
		std::swap(v3, u3);
		std::swap(n, m);
	}
}

void Model::compute_tangent_points() {
	//cout << "centers.size() = " << centers.size() << endl;
	//cout << "radii.size() = " << radii.size() << endl;
	//cout << "blocks.size() = " << blocks.size() << endl;	
	for (size_t i = 0; i < blocks.size(); i++) {
		if (blocks[i][2] > centers.size()) continue;
		//cout << "c1 = " << centers[blocks[i][0]][0] << endl;
		//cout << "c2 = " << centers[blocks[i][1]][0] << endl;
		//cout << "c3 = " << centers[blocks[i][2]][0] << endl;
		compute_tangent_point(camera_ray, centers[blocks[i][0]], centers[blocks[i][1]], centers[blocks[i][2]],
			radii[blocks[i][0]], radii[blocks[i][1]], radii[blocks[i][2]],
			tangent_points[i].v1, tangent_points[i].v2, tangent_points[i].v3,
			tangent_points[i].u1, tangent_points[i].u2, tangent_points[i].u3,
			tangent_points[i].n, tangent_points[i].m);
		//std::cout << i << ": (" << tangent_points[i].v1[0] << ", " << tangent_points[i].v1[1] << ", " << tangent_points[i].v1[2] << " ); " << std::endl;
	}
}

glm::dvec3 project_point_on_segment_copy(const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2);

void Model::print_model() {
	std::cout << "CENTERS" << std::endl;
	for (size_t i = 0; i < centers.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			std::cout << centers[i][j] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "RADII" << std::endl;
	for (size_t i = 0; i < radii.size(); i++) {
		std::cout << radii[i] << std::endl;
	}
	std::cout << "BLOCKS" << std::endl;
	for (size_t i = 0; i < blocks.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			if (blocks[i][j] < centers.size())
				std::cout << blocks[i][j] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "TANGENT POINTS" << std::endl;
	for (size_t i = 0; i < tangent_points.size(); i++) {
		if (blocks[i][2] > centers.size()) continue;
		std::cout << "block " << i << std::endl;
		std::cout << "	v1: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].v1[j] << " "; std::cout << std::endl;
		std::cout << "	v2: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].v2[j] << " "; std::cout << std::endl;
		std::cout << "	v3: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].v3[j] << " "; std::cout << std::endl;
		std::cout << "	u1: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].u1[j] << " "; std::cout << std::endl;
		std::cout << "	u2: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].u2[j] << " "; std::cout << std::endl;
		std::cout << "	u3: "; for (size_t j = 0; j < d; j++) std::cout << tangent_points[i].u3[j] << " "; std::cout << std::endl;
	}
}

void Model::render_outline() {
	std::vector<std::pair<Vector3, Vector3>> segments;
	std::vector<std::pair<Vector3, Vector3>> arc_endpoints;
	std::vector<Vector3> arc_centers;
	std::vector<float> arc_radii;
	for (size_t i = 0; i < outline_finder.outline3D.size(); i++) {
		glm::vec3 s = outline_finder.outline3D[i].start;
		glm::vec3 e = outline_finder.outline3D[i].end;
		if (outline_finder.outline3D[i].indices[1] != RAND_MAX) {
			segments.push_back(std::pair<Vector3, Vector3>(Vector3(s[0], s[1], s[2]), Vector3(e[0], e[1], e[2])));
		}
		else {
			glm::vec3 c = centers[outline_finder.outline3D[i].indices[0]];
			arc_centers.push_back(Vector3(c[0], c[1], c[2]));
			arc_radii.push_back(radii[outline_finder.outline3D[i].indices[0]]);
			arc_endpoints.push_back(std::pair<Vector3, Vector3>(Vector3(s[0], s[1], s[2]), Vector3(e[0], e[1], e[2])));
		}
	}
	DebugRenderer::instance().add_segments(segments, Vector3(0.75, 0.2, 0.45));
	DebugRenderer::instance().add_arcs(arc_endpoints, arc_centers, arc_radii, Vector3(0.75, 0.2, 0.45));
}

int Model::compute_outline_energy_data_helper(int row, int col, Vector3 p, int block, int num_rendered_points, Vector2 n_image, bool display,
	cv::Mat & image, const cv::Mat & sensor_silhouette, Camera * camera, energy::fitting::Settings * const settings, bool is_segment_point) {

	row = camera->height() - row - 1;
	if (col < 1 || col >= camera->width() || row < 1 || row >= camera->height()) return num_rendered_points;

	if (settings->fit2D_silhouette2outline_enable && sensor_silhouette.at<uchar>(row, col) == 255) return num_rendered_points;

	rendered_pixels[num_rendered_points] = row * camera->width() + col;
	rendered_points[3 * num_rendered_points] = p[0];
	rendered_points[3 * num_rendered_points + 1] = p[1];
	rendered_points[3 * num_rendered_points + 2] = p[2];
	if (settings->fit2D_outline_enable) { // TODO: remove rendered_normals
		rendered_normals[2 * num_rendered_points + 0] = n_image[0];
		rendered_normals[2 * num_rendered_points + 1] = n_image[1];
	}
	rendered_block_ids[num_rendered_points] = block;

	if (settings->fit2D_outline_enable && is_segment_point) {
		rendered_block_ids[num_rendered_points] += 1;
		rendered_block_ids[num_rendered_points] = -rendered_block_ids[num_rendered_points];
	}

	num_rendered_points++;

	if (display) {
		image.at<cv::Vec3b>(cv::Point(2 * col - 1, 2 * row - 1)) = cv::Vec3b(200, 200, 0);
		image.at<cv::Vec3b>(cv::Point(2 * col - 1, 2 * row)) = cv::Vec3b(200, 200, 0);
		image.at<cv::Vec3b>(cv::Point(2 * col, 2 * row - 1)) = cv::Vec3b(200, 200, 0);
		image.at<cv::Vec3b>(cv::Point(2 * col, 2 * row)) = cv::Vec3b(200, 200, 0);
	}

	return num_rendered_points;
}

void Model::compute_outline_energy_data(const cv::Mat & sensor_silhouette, Camera * camera, energy::fitting::Settings * const settings) {
	bool display = false;
	cv::Mat image;
	if (display) {
		cv::Scalar color = cv::Scalar(100, 0, 230);
		cv::Mat in[] = { sensor_silhouette, sensor_silhouette, sensor_silhouette };
		cv::merge(in, 3, image);
		cv::resize(image, image, cv::Size(640, 480));
	}

	num_rendered_points = 0;
	for (size_t i = 0; i < outline_finder.outline3D.size(); i++) {
		glm::vec3 s_glm = outline_finder.outline3D[i].start;
		glm::vec3 e_glm = outline_finder.outline3D[i].end;
		Vector3 s = Vector3(s_glm[0], s_glm[1], s_glm[2]);
		Vector3 e = Vector3(e_glm[0], e_glm[1], e_glm[2]);
		int block = outline_finder.outline3D[i].block;
		if (outline_finder.outline3D[i].indices[1] != RAND_MAX) { // segment

			//if (settings->fit2D_outline_enable) continue; 

			Vector2 s_image = camera->world_to_image(s);
			Vector2 e_image = camera->world_to_image(e);

			float x1 = s_image[0]; float y1 = s_image[1];
			float x2 = e_image[0]; float y2 = e_image[1];
			float X1 = s[0]; float Y1 = s[1]; float Z1 = s[2];
			float X2 = e[0]; float Y2 = e[1]; float Z2 = e[2];
			int num_samples = 1.2 * (e_image - s_image).norm();
			for (int n = 0; n < num_samples; n++) {
				float t = (float)n / (num_samples - 1);
				float x = x1 + t * (x2 - x1);
				float y = y1 + t * (y2 - y1);
				Vector3 p = Vector3(X1 + t * (X2 - X1), Y1 + t * (Y2 - Y1), Z1 + t * (Z2 - Z1));
				int col = (int)x;
				int row = (int)y;
				Vector2 direction = (e_image - s_image) / (e_image - s_image).norm();
				Vector2 n_image = Vector2(direction[1], -direction[0]);
				num_rendered_points = compute_outline_energy_data_helper(row, col, p, block, num_rendered_points, n_image, display, image, sensor_silhouette, camera, settings, true /*is_segment_point*/);
			}
		}
		else { // arc of circle
			glm::vec3 c_glm = centers[outline_finder.outline3D[i].indices[0]];
			float r = radii[outline_finder.outline3D[i].indices[0]];
			Vector3 c = Vector3(c_glm[0], c_glm[1], c_glm[2]);
			Vector2 c_image = camera->world_to_image(c);
			Vector2 s_image = camera->world_to_image(s);
			float r_image = (c_image - s_image).norm();
			Vector3 v1 = s - c;
			Vector3 v2 = e - c;

			float phi;
			Vector2 u = Vector2(1, 0); Vector2 v = Vector2(0, 1);
			Vector3 U = Vector3(1, 0, 0); Vector3 V = Vector3(0, 1, 0);

			float alpha = atan2(v1[0], v1[1]);
			float beta = atan2(v2[0], v2[1]);
			if (beta > alpha) alpha = alpha + 2 * M_PI;
			int num_samples = 1.2 * (alpha - beta) * r_image;
			for (int n = 0; n < num_samples; n++) {
				phi = alpha + n * (beta - alpha) / (num_samples - 1);
				Vector2 x = c_image + r_image * (u * sin(phi) + v * cos(phi));
				Vector3 p = c + r * (U * sin(phi) + V * cos(phi));
				int col = (int)x[0];
				int row = (int)x[1];
				Vector2 n_image = (x - c_image) / (x - c_image).norm();
				num_rendered_points = compute_outline_energy_data_helper(row, col, p, block, num_rendered_points, n_image, display, image, sensor_silhouette, camera, settings, false /*is_segment_point*/);
			}
		}
	}
	if (display) cv::imshow("sensor_silhouette", image);
	/*num_rendered_points = 0;
	for (int row = 0; row < rendered_silhouette.rows; ++row) {
	for (int col = 0; col < rendered_silhouette.cols; ++col) {
	if (rendered_silhouette.at<uchar>(row, col) != 255 && sensor_silhouette.at<uchar>(row, col) != 255) {
	rendered_indicator[num_rendered_points] = row * camera->width() + col;
	num_rendered_points++;
	}
	}
	}*/

}

void Model::write_model(std::string data_path, int frame_number) {

	cout << "data_path = " << data_path << endl;

	std::string frame_number_string = "";
	if (frame_number > 0) {
		frame_number_string = "-" + std::to_string(frame_number);
	}

	std::ofstream centers_file;
	centers_file.open(data_path + "C" + frame_number_string + ".txt");
	centers_file << centers.size() << endl;
	for (size_t i = 0; i < centers.size(); i++) {
		centers[i] -= glm::vec3(0, -50, 375);
		for (size_t j = 0; j < 3; j++) {
			centers_file << centers[i][j] << " ";
		}
		centers[i] += glm::vec3(0, -50, 375);
	}
	centers_file.close();

	std::ofstream radii_file;
	radii_file.open(data_path + "R" + frame_number_string + ".txt");
	radii_file << radii.size() << endl;
	for (size_t i = 0; i < radii.size(); i++) {
		radii_file << radii[i] << " ";
	}
	radii_file.close();

	std::ofstream transformations_file;
	transformations_file.open(data_path + "I" + frame_number_string + ".txt");
	transformations_file << num_phalanges + 1 << endl;
	for (size_t i = 0; i < num_phalanges + 1; i++) {
		for (size_t u = 0; u < 4; u++) {
			for (size_t v = 0; v < 4; v++) {
				transformations_file << phalanges[i].init_local(v, u) << " ";
			}
		}
	}
	transformations_file.close();

	std::ofstream blocks_file;
	blocks_file.open(data_path + "B" + frame_number_string + ".txt");
	blocks_file << blocks.size() << endl;
	for (size_t i = 0; i < blocks.size(); i++) {
		for (size_t j = 0; j < 3; j++) {
			blocks_file << blocks[i][j] << " ";
		}
	}
	blocks_file.close();

	if (frame_number >= 0) { // we are writing a model for matlab

		std::ofstream transformations_file;
		transformations_file.open(data_path + "I" + frame_number_string + ".txt");
		transformations_file << num_phalanges << endl;
		for (size_t i = 0; i < num_phalanges; i++) {
			for (size_t u = 0; u < 4; u++) {
				for (size_t v = 0; v < 4; v++) {
					transformations_file << phalanges[i].init_local(u, v) << " ";
				}
			}
		}
		transformations_file.close();

		std::ofstream theta_file;
		theta_file.open(data_path + "T" + frame_number_string + ".txt");
		theta_file << theta.size() << endl;
		for (size_t i = 0; i < theta.size(); i++) {
			theta_file << theta[i] << " ";
		}
		theta_file.close();

		std::ofstream beta_file;
		beta_file.open(data_path + "V" + frame_number_string + ".txt");
		beta_file << beta.size() << endl;
		for (size_t i = 0; i < beta.size(); i++) {
			beta_file << beta[i] << " ";
		}
		beta_file.close();

		std::ofstream blocks_file;
		blocks_file.open(data_path + "B" + frame_number_string + ".txt");
		blocks_file << blocks.size() << endl;
		for (size_t i = 0; i < blocks.size(); i++) {
			for (size_t j = 0; j < 3; j++) {
				blocks_file << blocks[i][j] << " ";
			}
		}
		blocks_file.close();
	}
	if (show_initialization_calls) cout << "finished writing the model" << endl;
}

void Model::load_model_from_file(bool load_calibrated_model, std::string input_path) {
	std::string model_path;

	if (input_path.length() > 0) model_path = input_path;
	else {
		if (load_calibrated_model)
			model_path = calibrated_model_path;
		else model_path = data_path + "models/anastasia/";
	}

	cout << "model_path = " << model_path << endl;

	read_model_centers(model_path, "C", centers);
	read_model_radii(model_path, "R", radii);
	read_model_blocks(model_path, "B", blocks);

	// Read initial transformations
	FILE *fp = fopen((model_path + "I.txt").c_str(), "r");
	int N; fscanf(fp, "%d", &N); 
	for (int i = 0; i < num_phalanges + 1; ++i) {
		phalanges[i].init_local = Mat4f::Zero(d + 1, d + 1);
		for (size_t u = 0; u < d + 1; u++) {
			for (size_t v = 0; v < d + 1; v++) {
				fscanf(fp, "%f", &phalanges[i].init_local(v, u));
			}
		}
	}
	fclose(fp);

	//for (size_t i = 0; i < centers.size(); i++) {
	//	centers[i] += glm::vec3(0, 0, 375);
	//}
}

Matrix_3xN Model::jacobian_theta(const Vector3 & s, size_t id) {
	Matrix_3xN J = Matrix_3xN::Zero(d, num_thetas);
	for (size_t i = 0; i < pose_units[id].kinematic_chain.size(); i++) {
		size_t dof_id = pose_units[id].kinematic_chain[i];
		size_t phalange_id = pose_dofs[dof_id].phalange_id;
		Vector3 u = pose_dofs[dof_id].axis;
		Vector3 p = phalanges[phalange_id].global.block(0, 3, 3, 1);
		Transform3f T = Transform3f(phalanges[phalange_id].global);
		Vector3 v = T * u - p; // rotated axis

		if (isnan(p(0)) || isnan(p(1)) || isnan(p(2))) {
			cout << "phalange_id = " << phalange_id << std::endl;
			std::cout << "p = " << p.transpose() << std::endl;
		}

		switch (pose_dofs[dof_id].type) {
		case TRANSLATION_AXIS:
			//J.col(dof_id) = T * u;
			J.col(dof_id) = u;
			if (isnan(u(0)) || isnan(u(1)) || isnan(u(2))) std::cout << "nan in jacobian" << std::endl;
			break;
		case ROTATION_AXIS:
			Vector3 w = v.cross(s - p);
			J.col(dof_id) = w;
			if (isnan(w(0)) || isnan(w(1)) || isnan(w(2))) std::cout << "nan in jacobian" << std::endl;
			break;
		}
		/*std::cout << endl << "i = " << i << std::endl;
		std::cout << "axis = " << u.transpose() << std::endl;
		std::cout << "p = " << p.transpose() << std::endl;
		std::cout << "s = " << s.transpose() << std::endl;
		std::cout << "T = " << endl << phalanges[phalange_id].global << std::endl;
		std::cout << "v = " << v.transpose() << std::endl;*/
	}
	return J;
}

void compute_segment_coordinates(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b, float & u, float & v) {
	float ab = glm::length(a - b);
	float ap = glm::length(a - p);
	u = 1 - ap / ab;
	v = 1 - u;
}

void compute_triangle_coordinates(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, float & u, float & v, float & w) {
	float epsilon = 1e-10;
	glm::vec3 v0 = b - a;
	glm::vec3 v1 = c - a;
	glm::vec3 v2 = p - a;
	float d00 = glm::dot(v0, v0);
	float d01 = glm::dot(v0, v1);
	float d11 = glm::dot(v1, v1);
	float d20 = glm::dot(v2, v0);
	float d21 = glm::dot(v2, v1);
	float denom = d00 * d11 - d01 * d01;
	if (abs(denom) > epsilon) {
		v = (d11 * d20 - d01 * d21) / denom;
		w = (d00 * d21 - d01 * d20) / denom;
		u = 1.0f - v - w;
	}
	else {
		if (glm::length(a - b) < epsilon) compute_segment_coordinates(p, a, c, u, w); v = 0;
		if (glm::length(a - c) < epsilon) compute_segment_coordinates(p, a, b, u, v); w = 0;
		if (glm::length(b - c) < epsilon) compute_segment_coordinates(p, a, b, u, v); w = 0;
	}
}

Matrix_3xN Model::jacobian_theta_new(const int pose_unit_id, const glm::vec3 & p, const glm::vec3 & s, const glm::ivec3 & index) {
	Matrix_3xN J = Matrix_3xN::Zero(3, num_thetas);

	for (int l = 0; l < KINEMATIC_MAX_LENGTH; l++) {
		int theta_info_id = host_pointer_kinematic_chains[pose_unit_id].data[l];
		if (theta_info_id == -1) break;
		const ThetaInfo & theta_info = theta_infos[theta_info_id];
		float * axis_pointer = theta_infos[theta_info_id].axis;
		glm::vec3 axis = glm::dvec3(axis_pointer[0], axis_pointer[1], axis_pointer[2]);

		float * global_pointer = theta_infos[theta_info_id].mat;
		glm::mat4 global = glm::mat4();
		for (size_t u = 0; u < 4; u++)
			for (size_t v = 0; v < 4; v++) global[u][v] = global_pointer[4 * u + v];

		glm::vec3 dq;
		switch (theta_info.type) {
		case 1: {
			dq = glm::dmat3(global) * axis;
			break;
		}
		case 0: {
			glm::vec3 t(global[3][0], global[3][1], global[3][2]);
			glm::vec3 u = glm::dmat3(global) * axis;
			dq = glm::cross(u, p - t);
			break;
		}
		}

		J.col(theta_info.jacobian_column) = Vector3(dq[0], dq[1], dq[2]);
	}
	return J;
}

Matrix_3xN Model::jacobian_beta(const int shape_unit_id, const glm::vec3 & q, const glm::vec3 & s, glm::ivec3 & index) {
	Matrix_3xN J = Matrix_3xN::Zero(3, num_betas);

	if (shape_unit_id == 26) {
		for (size_t d = 0; d < 3; d++) {
			if (index[d] == 38 && (index[0] == 33 || index[1] == 33 || index[2] == 33)) index[d] = 24;
			if (index[d] == 38 && (index[0] == 24 || index[1] == 24 || index[2] == 24)) index[d] = 33;
		}
	}


	for (int l = 0; l < SHAPE_MAX_LENGTH; l++) {
		int beta_info_id = host_pointer_shape_chains[shape_unit_id].data[l];
		if (beta_info_id == -1) break;
		const BetaInfo& beta_info = beta_infos[beta_info_id];
		float * axis_pointer = beta_infos[beta_info_id].axis;
		glm::vec3 axis = glm::dvec3(axis_pointer[0], axis_pointer[1], axis_pointer[2]);

		float * global_pointer = beta_infos[beta_info_id].mat;
		glm::mat4 global = glm::mat4();
		for (size_t u = 0; u < 4; u++) {
			for (size_t v = 0; v < 4; v++) {
				global[u][v] = global_pointer[4 * u + v];
			}
		}
		glm::vec3 t(global[3][0], global[3][1], global[3][2]);
		glm::vec3 v = glm::dmat3(global) * axis;

		glm::vec4 parametrization = glm::vec4(0);
		if (index[1] == RAND_MAX)
			parametrization[0] = 1.0;
		else if (index[2] == RAND_MAX)
			compute_segment_coordinates(s, centers[index[0]], centers[index[1]], parametrization[0], parametrization[1]);
		else
			compute_triangle_coordinates(s, centers[index[0]], centers[index[1]], centers[index[2]], parametrization[0], parametrization[1], parametrization[2]);

		glm::vec3 dq;
		float scaling_factor = 1.0f;
		if (beta_info.type == 0 /*TOP_PHALANGE_LENGTH*/ || beta_info.type == 1 /*PHALANGE_LENGTH*/) {

			if (index[0] == beta_info.center_id && index[1] == beta_info.top_center_id) scaling_factor = 1 - parametrization[0];
			if (index[1] == beta_info.center_id && index[0] == beta_info.top_center_id) scaling_factor = 1 - parametrization[1];
			if (index[0] == beta_info.center_id && index[1] == RAND_MAX) scaling_factor = 0.0;
			if (index[0] == beta_info.top_center_id && index[1] == RAND_MAX) scaling_factor = 1.0;

			if (shape_unit_id == 20 || shape_unit_id == 21 || shape_unit_id == 22 || shape_unit_id == 23 || shape_unit_id == 24 || shape_unit_id == 25) { // membranes	
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) {
						double fraction = 0.0;
						if (beta_info.attachment_center_id == 31) fraction = membranes_fractional_length[0];
						if (beta_info.attachment_center_id == 30) fraction = membranes_fractional_length[1];
						if (beta_info.attachment_center_id == 29) fraction = membranes_fractional_length[2];
						if (beta_info.attachment_center_id == 28) fraction = membranes_fractional_length[3];
						scaling_factor = fraction * parametrization[d];
					}
				}
			}
			if (shape_unit_id == 26) { /// thumb membrane				
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) scaling_factor = parametrization[d];
				}
			}
			dq = scaling_factor * v;

		}
		if (beta_info.type == 2 /*FINGER_BASE_X*/ || beta_info.type == 3 /*FINGER_BASE_Y*/ || beta_info.type == 4 /*FINGER_BASE_Z*/) {
			float scaling_factor = 1.0;
			if (shape_unit_id == 20 || shape_unit_id == 21 || shape_unit_id == 22 || shape_unit_id == 23 || shape_unit_id == 24 || shape_unit_id == 25) { // membranes				
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) scaling_factor = parametrization[d];
				}
			}
			if (shape_unit_id == 26) { /// thumb membrane				
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == beta_info.attachment_center_id) scaling_factor = 1 - parametrization[d];
				}
			}
			dq = scaling_factor * v;
		}
		if (beta_info.type == 5 /*PALM_CENTER_X*/ || beta_info.type == 6 /*PALM_CENTER_Y*/) {
			glm::vec3 parametrization;
			compute_triangle_coordinates(s, centers[index[0]], centers[index[1]], centers[index[2]],
				parametrization[0], parametrization[1], parametrization[2]);
			float independent_scaling_factor = 0;
			float dependent_scaling_factor = 0;
			for (size_t d = 0; d < 3; d++) {
				if (index[d] == beta_info.center_id) {
					independent_scaling_factor = parametrization[d];
				}
				if (beta_info.center_id == 20 || beta_info.center_id == 23) { // centers 21 and 22 linearly depend on centers 20 and 23
					if (index[d] == 21 || index[d] == 22) {
						float palm_width = glm::length(centers[20] - centers[23]);
						float width_ratio = 1 - glm::length(centers[index[d]] - centers[beta_info.center_id]) / palm_width;
						dependent_scaling_factor += parametrization[d] * width_ratio;
					}
				}
			}
			dq = (independent_scaling_factor + dependent_scaling_factor) * v;

			if (shape_unit_id == 27) { /// wrist
				glm::dvec4 get_i = glm::dvec4(3); for (size_t d = 0; d < 3; d++) get_i[index[d] - 34] = d;
				if (beta_info.type == 5 /*PALM_CENTER_X*/)
					dq = parametrization[get_i[34 - 34]] * v + parametrization[get_i[36 - 34]] * v - parametrization[get_i[35 - 34]] * v - parametrization[get_i[37 - 34]] * v;
				if (beta_info.type == 6 /*PALM_CENTER_Y*/)
					dq = -parametrization[get_i[36 - 34]] * v - parametrization[get_i[37 - 34]] * v;
			}

		}
		if (beta_info.type == 7 /*RADIUS*/) {
			glm::vec3 u = q - s;
			if (glm::length(u) == 0) continue;
			u = u / glm::length(u);
			float scaling_factor = 0;
			for (size_t d = 0; d < 3; d++) {
				if (index[d] == beta_info.center_id)
					scaling_factor = parametrization[d];
			}
			dq = scaling_factor * u;
		}

		if (beta_info.type == 8 /*FINGER_TOP_Y*/ || beta_info.type == 9 /*FINGER_TOP_Z*/) {
			float scaling_factor = 0;
			if (index[0] == beta_info.center_id && index[1] == beta_info.top_center_id) scaling_factor = 1 - parametrization[0];
			if (index[1] == beta_info.center_id && index[0] == beta_info.top_center_id) scaling_factor = 1 - parametrization[1];
			if (index[0] == beta_info.center_id && index[1] == RAND_MAX) scaling_factor = 0.0;
			if (index[0] == beta_info.top_center_id && index[1] == RAND_MAX) scaling_factor = 1.0;
			dq = scaling_factor * v;
		}

		J.col(beta_info.jacobian_column - num_thetas) = Vector3(dq[0], dq[1], dq[2]);
	}
	return J;
}

void Model::update(Phalange & phalange) {
	if (phalange.parent_id >= 0)
		phalange.global = phalanges[phalange.parent_id].global * phalange.local;
	else  phalange.global = phalange.local;
	for (size_t i = 0; i < phalange.children_ids.size(); i++)  update(phalanges[phalange.children_ids[i]]);

	// check if isnan
	/*bool has_nan = false;
	for (size_t i = 0; i < 4; i++) {
	for (size_t j = 0; j < 4; j++) {
	if (isnan(phalange.global(i, j)))
	has_nan = true;
	break;
	}
	}
	if (has_nan) std::cerr << "nan in global transformation" << std::endl;*/
}

void Model::transform_joints(const std::vector<float> & theta) {
	//cout << "transform_joints" << endl;	

	for (size_t i = 0; i < pose_dofs.size(); ++i) {
		if (pose_dofs[i].phalange_id == -1) continue;
		Phalange & phalange = phalanges[pose_dofs[i].phalange_id];
		switch (pose_dofs[i].type) {
		case TRANSLATION_AXIS: {
			Vec3f t = pose_dofs[i].axis * theta[i];
			phalange.local(0, 3) += t[0];
			phalange.local(1, 3) += t[1];
			phalange.local(2, 3) += t[2];
			update(phalange);
			break;
		}
		case ROTATION_AXIS:
			phalange.local = phalange.local * Transform3f(Eigen::AngleAxisf(theta[i], pose_dofs[i].axis)).matrix();
			update(phalange);
			break;
		}
	}
}

void Model::update_parameters(const std::vector<float> & parameters) {
	if (parameters.size() == num_thetas) {
		update_theta(parameters);
		return;
	}
	if (parameters.size() == num_betas) {
		update_beta(parameters);
		return;
	}
}

void Model::print_beta() {
	if (num_betas == 0) return;
	cout << "beta:" << endl;
	for (size_t i = 0; i < num_betas; i++) {
		cout << "[" << i << "]: " << beta[i] << ", ";
		if (i == 14 || i == 29) cout << endl; // put a new line before each type of beta
	}
	cout << endl << endl;
}

const std::vector<float>& Model::get_beta() {
	if (num_betas == 0) return std::vector<float>(0, 0);

	for (size_t i = 0; i < num_betas; i++) {
		size_t phalange_id = shape_dofs[i].phalange_id;
		BetaType beta_type = shape_dofs[i].type;

		if (beta_type == PHALANGE_LENGTH) {
			size_t child_phalange_id = phalange_id + 1;
			Eigen::Vector3d v = phalanges[child_phalange_id].init_local.block(0, 3, 3, 1).cast<double>();
			beta[i] = v.norm();
		}
		if (beta_type == TOP_PHALANGE_LENGTH) {
			Eigen::Vector3d v = phalanges[phalange_id].offsets[0];
			beta[i] = v.norm();
		}
		if (beta_type == FINGER_BASE_X) {
			beta[i] = phalanges[phalange_id].init_local(0, 3);
		}
		if (beta_type == FINGER_BASE_Y) {
			beta[i] = phalanges[phalange_id].init_local(1, 3);
		}
		if (beta_type == FINGER_BASE_Z) {
			beta[i] = phalanges[phalange_id].init_local(2, 3);
		}
		if (beta_type == PALM_CENTER_X) {
			if (phalange_id == phalanges_name_to_id_map["Hand"]) {
				size_t attachment_id = centerid_to_attachment_id_map[shape_dofs[i].center_id];
				beta[i] = phalanges[phalange_id].offsets[attachment_id][0];
			}
		}
		if (beta_type == PALM_CENTER_Y) {
			if (phalange_id == phalanges_name_to_id_map["Hand"]) {
				size_t attachment_id = centerid_to_attachment_id_map[shape_dofs[i].center_id];
				beta[i] = phalanges[phalange_id].offsets[attachment_id][1];
			}
		}
		if (beta_type == RADIUS) {
			beta[i] = radii[shape_dofs[i].center_id];
		}
		if (beta_type == FINGER_TOP_Y) {
			beta[i] = (phalanges[phalange_id].offsets[1] - phalanges[phalange_id].offsets[0])[1];
		}
		if (beta_type == FINGER_TOP_Z) {
			beta[i] = (phalanges[phalange_id].offsets[1] - phalanges[phalange_id].offsets[0])[2];
		}
		this->beta[i] = beta[i];
	}

	beta[shape_dofs_name_to_id_map["wrist_width_left"]] = phalanges[phalanges_name_to_id_map["Wrist"]].offsets[2][0];
	beta[shape_dofs_name_to_id_map["wrist_width"]] = phalanges[phalanges_name_to_id_map["Wrist"]].offsets[0][0];
	beta[shape_dofs_name_to_id_map["wrist_length"]] = abs(phalanges[phalanges_name_to_id_map["Wrist"]].offsets[0][1]);

	return beta;
}

void Model::update_beta(const VectorN & solution) {
	if (num_betas == 0) return;
	vector<float> delta_beta(solution.data() + num_thetas, solution.data() + num_thetas + num_betas);
	for (size_t i = 0; i < num_betas; i++) {
		beta[i] = beta[i] + delta_beta[i];
	}

	if (enable_shape_prior) {
		for (size_t i = 0; i < num_betas_latent; i++) {
			beta_latent[i] += solution[i + num_parameters + num_thetas_latent];
		}
	}

	update_beta(beta);
}

void Model::update_beta(const std::vector<float> & beta) {
	//if (beta.empty()) { cout << "beta is empty" << endl; return; }
	if (num_betas == 0) return;

	for (size_t i = 0; i < num_betas; i++) {
		int phalange_id = shape_dofs[i].phalange_id;
		BetaType beta_type = shape_dofs[i].type;
		if ((beta_type == PHALANGE_LENGTH || beta_type == TOP_PHALANGE_LENGTH) && beta[i] < 0)
			cout << "error, beta[" << i << "] = " << beta[i] << endl;

		if (beta_type == BetaType::PHALANGE_LENGTH) {
			size_t child_phalange_id = phalange_id + 1;
			Eigen::Vector3f v = phalanges[child_phalange_id].init_local.block(0, 3, 3, 1);
			Eigen::Vector3f u = v / v.norm();
			if (u[1] < 0)
				cout << "error, direction[" << i << "] = " << u[1] << endl;
			v = beta[i] * u;
			phalanges[child_phalange_id].init_local.block(0, 3, 3, 1) = v;
		}
		if (beta_type == TOP_PHALANGE_LENGTH) {
			Eigen::Vector3d v = phalanges[phalange_id].offsets[0];
			v = beta[i] * v / v.norm();
			phalanges[phalange_id].offsets[0] = v;
		}
		if (beta_type == FINGER_BASE_X) {
			phalanges[phalange_id].init_local(0, 3) = beta[i];
		}
		if (beta_type == FINGER_BASE_Y) {
			phalanges[phalange_id].init_local(1, 3) = beta[i];
		}
		if (beta_type == FINGER_BASE_Z) {
			phalanges[phalange_id].init_local(2, 3) = beta[i];
		}
		if (beta_type == PALM_CENTER_X) {
			if (phalange_id == phalanges_name_to_id_map["Hand"]) {
				size_t attachment_id = centerid_to_attachment_id_map[shape_dofs[i].center_id];
				phalanges[phalange_id].offsets[attachment_id][0] = beta[i];
				phalanges[phalange_id].offsets[attachment_id][2] = 0;
			}			
		}
		if (beta_type == PALM_CENTER_Y) {
			if (phalange_id == phalanges_name_to_id_map["Hand"]) {
				size_t attachment_id = centerid_to_attachment_id_map[shape_dofs[i].center_id];
				phalanges[phalange_id].offsets[attachment_id][1] = beta[i];
			}			
		}
		if (beta_type == PALM_CENTER_X || beta_type == PALM_CENTER_Y) {
			// recompute attachments palm-middle and palm-ring - replace by something more elegant
			if (phalange_id == phalanges_name_to_id_map["Hand"]) {
				Vec3d palm_index_offset = phalanges[phalange_id].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_index"]]];
				Vec3d palm_pinky_offset = phalanges[phalange_id].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_pinky"]]];
				Vec3d palm_middle_offset = palm_index_offset + palm_middle_offset_ratio * (palm_pinky_offset - palm_index_offset);
				Vec3d palm_ring_offset = palm_index_offset + palm_ring_offset_ratio * (palm_pinky_offset - palm_index_offset);
				phalanges[phalange_id].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_middle"]]] = palm_middle_offset;
				phalanges[phalange_id].offsets[centerid_to_attachment_id_map[centers_name_to_id_map["palm_ring"]]] = palm_ring_offset;
			}
		}
		if (beta_type == RADIUS) {
			radii[shape_dofs[i].center_id] = beta[i];
		}
		if (beta_type == FINGER_TOP_Y) {
			phalanges[phalange_id].offsets[1][1] = phalanges[phalange_id].offsets[0][1] + beta[i];
		}
		if (beta_type == FINGER_TOP_Z) {
			phalanges[phalange_id].offsets[1][2] = phalanges[phalange_id].offsets[0][2] + beta[i];
		}
		this->beta[i] = beta[i];
	}
	/// > Depends only on scale
	if (calibration_type == FULL) {
		for (size_t i = 0; i < radii.size(); i++) {
			if (i == centers_name_to_id_map["index_membrane"] || i == centers_name_to_id_map["middle_membrane"] || i == centers_name_to_id_map["ring_membrane"] || i == centers_name_to_id_map["pinky_membrane"] ||
				i == centers_name_to_id_map["palm_thumb"] || i == centers_name_to_id_map["thumb_membrane_left"] || i == centers_name_to_id_map["thumb_additional"]) {
				if (beta_latent[0] < 1) radii[i] = beta_latent[0] * radii_template[i];
				else radii[i] = radii_template[i];				
			}
		}
		{ /// membranes
			auto compute_membrane_offset = [=](int i, float length, float fraction, Vec3d & offset, float radius) {
				Vec3f top = phalanges[phalanges[i].children_ids[0]].init_local.block(0, 3, 3, 1);
				Vec3f membrane = phalanges[i].offsets[0].cast<float>();
				glm::dvec3 projection_glm = project_point_on_segment_copy(glm::vec3(membrane[0], membrane[1], membrane[2]), glm::vec3(0, 0, 0), glm::vec3(top[0], top[1], top[2]));
				Vec3f projection = Vec3f(projection_glm[0], projection_glm[1], projection_glm[2]);
				Vec3f radial_offset = phalanges[i].offsets[0].cast<float>() - projection;
				radial_offset = radial_offset / radial_offset.norm() * 0.6 * radius;
				projection = fraction * length * projection / projection.norm();
				offset = (projection + radial_offset).cast<double>();
			};
			compute_membrane_offset(phalanges_name_to_id_map["HandIndex1"], beta[3], membranes_fractional_length[0], phalanges[phalanges_name_to_id_map["HandIndex1"]].offsets[0], beta[shape_dofs_name_to_id_map["index_bottom_radius"]]);
			compute_membrane_offset(phalanges_name_to_id_map["HandMiddle1"], beta[6], membranes_fractional_length[1], phalanges[phalanges_name_to_id_map["HandMiddle1"]].offsets[0], beta[shape_dofs_name_to_id_map["middle_bottom_radius"]]);
			compute_membrane_offset(phalanges_name_to_id_map["HandRing1"], beta[9], membranes_fractional_length[2], phalanges[phalanges_name_to_id_map["HandRing1"]].offsets[0], beta[shape_dofs_name_to_id_map["ring_bottom_radius"]]);
			compute_membrane_offset(phalanges_name_to_id_map["HandPinky1"], beta[12], membranes_fractional_length[3], phalanges[phalanges_name_to_id_map["HandPinky1"]].offsets[0], beta[shape_dofs_name_to_id_map["pinky_bottom_radius"]]);
			//cout << phalanges[phalanges_name_to_id_map["HandMiddle1"]].offsets[0].transpose() << ", " << phalanges_template[phalanges_name_to_id_map["HandMiddle1"]].offsets[0].transpose() << endl;
		}
		{ /// thumb-fold 	
			radii[32] = 0.67 * beta[calibration_type_to_num_betas_map[FINGERS_AND_PALM] + 3];
		}
		{ /// wrist
			phalanges[phalanges_name_to_id_map["Wrist"]].offsets[0][0] = beta[40]; // wrist_bottom_left_x
			phalanges[phalanges_name_to_id_map["Wrist"]].offsets[0][1] = -beta[41]; // wrist_bottom_left_y

			phalanges[phalanges_name_to_id_map["Wrist"]].offsets[1][0] = -beta[40]; // wrist_bottom_right_x
			phalanges[phalanges_name_to_id_map["Wrist"]].offsets[1][1] = -beta[41]; // wrist_bottom_right_y

			phalanges[phalanges_name_to_id_map["Wrist"]].offsets[2][0] = beta[45]; // wrist_top_left_x
			phalanges[phalanges_name_to_id_map["Wrist"]].offsets[3][0] = -beta[40]; // wrist_top_right_x

			size_t c = calibration_type_to_num_betas_map[FINGERS_AND_PALM];
			radii[centers_name_to_id_map["wrist_top_right"]] = beta[c + 27] + 0.01;
			radii[centers_name_to_id_map["wrist_bottom_right"]] = beta[c + 28] + 0.01;
		}
	}
}

void Model::update_phalanges() {
	if (num_betas == 0) return;

	phalanges[0].length = 0; phalanges[0].radius1 = 0; phalanges[0].radius2 = 0;
	phalanges[16].length = 0; phalanges[16].radius1 = 0; phalanges[16].radius2 = 0;
	phalanges[17].length = 0; phalanges[17].radius1 = 0; phalanges[17].radius2 = 0;
	phalanges[1].radius1 = radii[centers_name_to_id_map["thumb_base"]]; phalanges[1].radius2 = radii[centers_name_to_id_map["thumb_bottom"]];
	phalanges[2].radius1 = radii[centers_name_to_id_map["thumb_bottom"]]; phalanges[2].radius2 = radii[centers_name_to_id_map["thumb_middle"]];
	phalanges[3].radius1 = radii[centers_name_to_id_map["thumb_middle"]]; phalanges[3].radius2 = radii[centers_name_to_id_map["thumb_top"]];
	phalanges[4].radius1 = radii[centers_name_to_id_map["pinky_base"]]; phalanges[4].radius2 = radii[centers_name_to_id_map["pinky_bottom"]];
	phalanges[5].radius1 = radii[centers_name_to_id_map["pinky_bottom"]]; phalanges[5].radius2 = radii[centers_name_to_id_map["pinky_middle"]];
	phalanges[6].radius1 = radii[centers_name_to_id_map["pinky_middle"]]; phalanges[6].radius2 = radii[centers_name_to_id_map["pinky_top"]];
	phalanges[7].radius1 = radii[centers_name_to_id_map["ring_base"]]; phalanges[7].radius2 = radii[centers_name_to_id_map["ring_bottom"]];
	phalanges[8].radius1 = radii[centers_name_to_id_map["ring_bottom"]]; phalanges[8].radius2 = radii[centers_name_to_id_map["ring_middle"]];
	phalanges[9].radius1 = radii[centers_name_to_id_map["ring_middle"]]; phalanges[9].radius2 = radii[centers_name_to_id_map["ring_top"]];
	phalanges[10].radius1 = radii[centers_name_to_id_map["middle_base"]]; phalanges[10].radius2 = radii[centers_name_to_id_map["middle_bottom"]];
	phalanges[11].radius1 = radii[centers_name_to_id_map["middle_bottom"]]; phalanges[11].radius2 = radii[centers_name_to_id_map["middle_middle"]];
	phalanges[12].radius1 = radii[centers_name_to_id_map["middle_middle"]]; phalanges[12].radius2 = radii[centers_name_to_id_map["middle_top"]];
	phalanges[13].radius1 = radii[centers_name_to_id_map["index_base"]]; phalanges[13].radius2 = radii[centers_name_to_id_map["index_bottom"]];
	phalanges[14].radius1 = radii[centers_name_to_id_map["index_bottom"]]; phalanges[14].radius2 = radii[centers_name_to_id_map["index_middle"]];
	phalanges[15].radius1 = radii[centers_name_to_id_map["index_middle"]]; phalanges[15].radius2 = radii[centers_name_to_id_map["index_top"]];

	for (size_t i = 0; i < num_betas; i++) {
		if (shape_dofs[i].type == PHALANGE_LENGTH || shape_dofs[i].type == TOP_PHALANGE_LENGTH) {
			phalanges[shape_dofs[i].phalange_id].length = beta[i];
		}
	}
}

const std::vector<float>& Model::get_theta() {
	return theta;
}

void Model::print_theta() {
	cout << "theta" << endl;
	for (size_t i = 0; i < num_thetas; i++) {
		cout << theta[i] << " ";
	}
	cout << endl << endl;
}

void Model::update_theta(const std::vector<float> & theta) {
	for (size_t i = 0; i < num_thetas; i++) {
		this->theta[i] = theta[i];
	}
	for (size_t i = 0; i < num_phalanges + 1; i++) {
		phalanges[i].local = phalanges[i].init_local;
	}

	//cout << "move" << endl;
	vector<float> rotateX(num_thetas, 0); // flexion
	vector<float> rotateZ(num_thetas, 0); // abduction
	vector<float> rotateY(num_thetas, 0); // twist
	vector<float> globals(num_thetas, 0); // pose

	for (size_t i = 0; i < num_thetas; ++i) {
		if (pose_dofs[i].phalange_id < num_phalanges && pose_dofs[i].type == ROTATION_AXIS) {
			if (pose_dofs[i].axis == Vec3f(1, 0, 0))
				rotateX[i] = theta[i];
			else if (pose_dofs[i].axis == Vec3f(0, 1, 0))
				rotateY[i] = theta[i];
			else if (pose_dofs[i].axis == Vec3f(0, 0, 1))
				rotateZ[i] = theta[i];
			else
				cout << "wrong axis" << endl;
		}
		else
			globals[i] = theta[i];
	}

	//transform joints separately
	transform_joints(globals); // pose	
	transform_joints(rotateX); // flexion
	transform_joints(rotateZ); // abduction
	transform_joints(rotateY); // twist
}

void Model::update_centers() {
	bool print = false;

	for (size_t i = 0; i < num_phalanges; i++) {
		Vec3f p = phalanges[i].global.block(0, 3, 3, 1);
		centers[phalanges[i].center_id] = glm::vec3(p[0], p[1], p[2]);

		if (print) {
			cout << endl << endl << endl << "i = " << i << endl;
			cout << "name = " << phalanges[i].name << endl;
			cout << "center_id = " << phalanges[i].center_id << endl;
			cout << " center = " << centers[phalanges[i].center_id][0] << ", " << centers[phalanges[i].center_id][1] << ", " << centers[phalanges[i].center_id][2] << endl;
		}
		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			Vec3d t = phalanges[i].global.block(0, 0, 3, 3).cast<double>() * phalanges[i].offsets[j];
			centers[phalanges[i].attachments[j]] = glm::vec3(p[0], p[1], p[2]) + glm::vec3(t[0], t[1], t[2]);

			if (print) {
				cout << endl << "attachment_id = " << phalanges[i].attachments[j] << endl;
				cout << "offset = " << phalanges[i].offsets[j][0] << ", " << phalanges[i].offsets[j][1] << ", " << phalanges[i].offsets[j][2] << endl;
				cout << " center = " << centers[phalanges[i].attachments[j]][0] << ", " << centers[phalanges[i].attachments[j]][1] << ", " << centers[phalanges[i].attachments[j]][2] << endl;
				cout << "t = " << t[0] << ", " << t[1] << ", " << t[2] << endl;
			}
		}
	}

	float alpha = 0.4f; float beta = 0.3f; float gamma = 0.3f;
	centers[centers_name_to_id_map["thumb_membrane_middle"]] = alpha * centers[centers_name_to_id_map["palm_thumb"]] +
		beta * centers[centers_name_to_id_map["thumb_membrane_left"]] + gamma * centers[centers_name_to_id_map["thumb_base"]];
	radii[centers_name_to_id_map["thumb_membrane_middle"]] = alpha * radii[centers_name_to_id_map["palm_thumb"]] +
		beta * radii[centers_name_to_id_map["thumb_membrane_left"]] + gamma * radii[centers_name_to_id_map["thumb_base"]];

	reindex();
	compute_tangent_points();
	if (calibration_type != NONE) update_phalanges();

}

std::vector<float> Model::get_updated_parameters(const vector<float> & theta, const vector<float> &delta_theta) {
	size_t rx = 3;
	size_t ry = 4;
	size_t rz = 5;

	std::vector<float> updated(num_thetas);
	for (size_t i = 0; i < num_thetas; ++i)
		updated[i] = theta[i] + (i < delta_theta.size() ? delta_theta[i] : 0);

	Vec3f axisX(1, 0, 0);
	Vec3f axisY(0, 1, 0);
	Vec3f axisZ(0, 0, 1);

	Transform3f rX(Eigen::Quaternionf(Eigen::AngleAxisf(delta_theta[rx], axisX)));
	Transform3f rY(Eigen::Quaternionf(Eigen::AngleAxisf(delta_theta[ry], axisY)));
	Transform3f rZ(Eigen::Quaternionf(Eigen::AngleAxisf(delta_theta[rz], axisZ)));

	Mat3f r = (rZ * rX * rY).rotation();

	r = phalanges[17].global.block(0, 0, 3, 3) * r;

	Vec3f e = r.eulerAngles(0, 1, 2);

	updated[rx] = e[0];
	updated[ry] = e[1];
	updated[rz] = e[2];

	return updated;
}

Vec3f Model::get_palm_center() {
	glm::vec3 palm_center = glm::vec3(0, 0, 0);

	palm_center += centers[centers_name_to_id_map["palm_right"]];
	palm_center += centers[centers_name_to_id_map["palm_left"]];
	palm_center += centers[centers_name_to_id_map["palm_pinky"]];
	palm_center += centers[centers_name_to_id_map["palm_ring"]];
	palm_center += centers[centers_name_to_id_map["palm_middle"]];
	palm_center += centers[centers_name_to_id_map["palm_index"]];
	palm_center = palm_center / 6.0f;

	return Vec3f(palm_center[0], palm_center[1], palm_center[2]);
}

float Model::get_palm_length() {
	float length = 0;
	glm::vec3 c = (centers[centers_name_to_id_map["ring_membrane"]] + centers[centers_name_to_id_map["middle_membrane"]]) / 2.0f;
	length = glm::length(centers[centers_name_to_id_map["palm_back"]] - c);
	return length;
}

Mat3f Model::build_rotation_matrix(Vec3f euler_angles) {
	float alpha = euler_angles(0);
	float beta = euler_angles(1);
	float gamma = euler_angles(2);
	Mat3f Rx, Ry, Rz;
	Rx << 1, 0, 0, 0, cos(alpha), -sin(alpha), 0, sin(alpha), cos(alpha);
	Ry << cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta);
	Rz << cos(gamma), -sin(gamma), 0, sin(gamma), cos(gamma), 0, 0, 0, 1;
	return Rz * Ry * Rx;
}

void Model::manually_adjust_initial_transformations() {
	Mat3f R;
	// thumb
	R = build_rotation_matrix(Vec3f(-1.45, 0.6, -1.95));
	phalanges[1].init_local.block(0, 0, 3, 3) = R;

	//R = build_rotation_matrix(Vec3f(-1.45, 0.6, -1));
	//phalanges[1].init_local.block(0, 0, 3, 3) = R;

	// index	
	//R = build_rotation_matrix(Vec3f(3.14, 0, -3.5));
	//phalanges[13].init_local.block(0, 0, 3, 3) = R;
	R = build_rotation_matrix(Vec3f(0, 0, -0.08));
	phalanges[15].init_local.block(0, 0, 3, 3) = R;

	//middle
	R = build_rotation_matrix(Vec3f(3.1067, -0.12, -3.1215));
	phalanges[10].init_local.block(0, 0, 3, 3) = R;
	// ring
	R = build_rotation_matrix(Vec3f(-3.054, 0.0, -2.9823));
	phalanges[7].init_local.block(0, 0, 3, 3) = R;

	std::vector<float> theta = std::vector<float>(num_thetas, 0);
	theta[1] = -50; theta[2] = 375; theta[9] = 0.7; theta[10] = 0.6;
	update_parameters(theta);
	update_centers();
	initialize_offsets();
}

void Model::resize_model(float uniform_scaling_factor, float width_scaling_factor, float thickness_scaling_factor) {
	Mat3d scaling_matrix = Mat3d::Identity();
	scaling_matrix(0, 0) = uniform_scaling_factor * width_scaling_factor;
	scaling_matrix(1, 1) = uniform_scaling_factor;
	scaling_matrix(2, 2) = uniform_scaling_factor;

	for (size_t i = 0; i < radii.size(); i++) {
		radii[i] *= uniform_scaling_factor * (1 + (thickness_scaling_factor - 1) * 1);
	}

	for (size_t i = 0; i < num_phalanges; i++) {
		phalanges[i].init_local.col(3).segment(0, 3) = scaling_matrix.cast <float>() * phalanges[i].init_local.col(3).segment(0, 3);
		for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
			phalanges[i].offsets[j] = scaling_matrix * phalanges[i].offsets[j];
		}
	}
}

void Model::perturb_parameters(unsigned random_seed, float uniform_scaling_mean, float prior_standard_deviation, float parameters_standard_deviation) {

	std::default_random_engine generator(random_seed);
	/*std::uniform_real_distribution<float> distribution(-prior_standard_deviation, prior_standard_deviation);
	auto uniform_real_distribution_with_offset = [&]() {
		float r = distribution(generator);
		if (r >= 0) r = 0.45 * (0.5 * prior_standard_deviation + 0.5 * r);
		if (r < 0) r = -0.5 * prior_standard_deviation + 0.5 * r;
		if (r < -0.35) r = -0.35;
		if (r > 0.35) r = 0.35;
		return r;
	};
	float uniform_scaling_factor = uniform_scaling_mean + uniform_real_distribution_with_offset();
	float width_scaling_factor = 1 + 0.5 * uniform_real_distribution_with_offset();*/

	//uniform_scaling_factor = 0.65;
	//width_scaling_factor = 1.1;

	std::normal_distribution<double> distribution(0.0, prior_standard_deviation);
	float r1 = distribution(generator); if (r1 < -0.7) r1 = -0.7; if (r1 > 0.7) r1 = 0.7;
	float r2 = distribution(generator); if (r2 < -0.7) r2 = -0.7; if (r2 > 0.7) r2 = 0.7;
	float uniform_scaling_factor =  uniform_scaling_mean + r1;
	float width_scaling_factor = 1 + r2;

	uniform_scaling_factor = 0.95;
	width_scaling_factor = 1.0;

	cout << "   random_seed = " << random_seed << ", uniform = " << uniform_scaling_factor << ", width = " << width_scaling_factor << endl;
	beta_latent[0] = uniform_scaling_factor;

	if (calibration_type == FINGERS || calibration_type == FINGERS_AND_PALM) {
		cout << "tempalte perturbation deteled" << endl;
	}
	if (calibration_type == FULL) {

		for (size_t i = 0; i < radii.size(); i++) {
			radii[i] *= 1 + (uniform_scaling_factor - 1);
			//radii[i] *= 1 + (width_scaling_factor - 1);
			//radii[i] += 0.5 * distribution(generator);
		}

		Mat3d scaling_matrix = Mat3d::Identity();
		scaling_matrix(0, 0) = uniform_scaling_factor * width_scaling_factor;
		scaling_matrix(1, 1) = uniform_scaling_factor;
		scaling_matrix(2, 2) = uniform_scaling_factor;

		for (size_t i = 0; i < num_phalanges; i++) {
			Eigen::Vector3f random_pertrubation;
			if (i == phalanges_name_to_id_map["HandThumb1"] || i == phalanges_name_to_id_map["HandIndex1"] || i == phalanges_name_to_id_map["HandMiddle1"] || i == phalanges_name_to_id_map["HandRing1"] || i == phalanges_name_to_id_map["HandPinky1"]) {
				random_pertrubation = parameters_standard_deviation * Eigen::Vector3f(distribution(generator), distribution(generator), distribution(generator));
			}
			else {
				Eigen::Vector3f direction = phalanges[i].init_local.block(0, 3, 3, 1);
				random_pertrubation = parameters_standard_deviation * distribution(generator) * direction / direction.norm();
			}
			if (i == phalanges_name_to_id_map["Hand"] || i == phalanges_name_to_id_map["Wrist"]) random_pertrubation = Eigen::Vector3f::Zero();

			phalanges[i].init_local.col(3).segment(0, 3) = scaling_matrix.cast <float>() * phalanges[i].init_local.col(3).segment(0, 3) + random_pertrubation;

			for (size_t j = 0; j < phalanges[i].attachments.size(); j++) {
				Eigen::Vector3d random_pertrubation = Eigen::Vector3d::Zero();
				if (i == phalanges_name_to_id_map["HandThumb3"] || i == phalanges_name_to_id_map["HandIndex3"] || i == phalanges_name_to_id_map["HandMiddle3"] || i == phalanges_name_to_id_map["HandRing3"] || i == phalanges_name_to_id_map["HandPinky3"]) {
					Eigen::Vector3d direction = phalanges[i].offsets[j];
					random_pertrubation = parameters_standard_deviation * distribution(generator) * direction / direction.norm();
				}
				else {
					random_pertrubation = parameters_standard_deviation * Eigen::Vector3d(distribution(generator), distribution(generator), distribution(generator));
					random_pertrubation[2] = 0;
				}

				phalanges[i].offsets[j] = scaling_matrix * phalanges[i].offsets[j] + random_pertrubation;
			}
		}
	}
	update_beta(get_beta());
	update_theta(get_theta());
	update_centers();
	compute_outline();
}

glm::dvec3 project_point_on_segment_copy(const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2) {
	glm::dvec3 u = c2 - c1;
	glm::dvec3 v = p - c1;
	double alpha = dot(u, v) / dot(u, u);
	if (alpha <= 0) return c1;
	if (alpha > 0 && alpha < 1) return c1 + alpha * u;
	if (alpha >= 1) return c2;
}

void Model::set_initial_pose() {
	// pose tracking first
	std::vector<float> theta = std::vector<float>(num_thetas, 0);
	theta[1] = -50; theta[2] = 375; //theta[9] = 0.7; theta[10] = 0.6;
	update_theta(theta); update_centers();
}