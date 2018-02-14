#pragma once
#include "ConvolutionRenderer.h"
#include <iostream>
#include <vector>
#include <cstdlib>
#include <Eigen/Dense>
#include <GL/glew.h> 
#include <OpenGP/GL/EigenOpenGLSupport3.h>
#include <QGLBuffer>
#include <QGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <iomanip>

glm::vec3 ConvolutionRenderer::world_to_window_coordinates(glm::vec3 point) {
	Eigen::Matrix4f view_projection = projection * camera.view;
	glm::mat4 MVP_glm = glm::mat4(0);
	for (size_t i = 0; i < 4; i++) {
		for (size_t j = 0; j < 4; j++) {
			MVP_glm[j][i] = projection(i, j);
		}
	}

	glm::vec4 point_gl = MVP_glm * glm::vec4(point, 1.0);
	glm::vec3 point_clip = glm::vec3(point_gl[0], point_gl[1], point_gl[2]) / point_gl[3];
	float f = camera.zFar;
	float n = camera.zNear;

	float ox = window_left + window_width / 2;
	float oy = window_bottom + window_height / 2;

	float xd = point_clip[0];
	float yd = point_clip[1];
	float zd = point_clip[2];

	glm::vec3 point_window = glm::vec3(0, 0, 0);
	point_window[0] = xd * window_width / 2 + ox;
	point_window[1] = yd * window_height / 2 + oy;
	point_window[2] = zd * (f - n) / 2 + (n + f) / 2;

	return point_window;
}

ConvolutionRenderer::ConvolutionRenderer(Model *model, bool real_color, std::string data_path, int camera_heigth, int camera_width) {
	this->data_path = data_path;
	this->model = model;
	this->real_color = real_color;
	this->camera_heigth = camera_heigth;
	this->camera_width = camera_width;

	//if (this->display_certainty) {
	center_index_to_length_certainty_map = vector<float>(model->centers.size(), 0);
	center_index_to_x_certainty_map = vector<float>(model->centers.size(), 0);
	center_index_to_y_certainty_map = vector<float>(model->centers.size(), 0);
	center_index_to_radius_certainty_map = vector<float>(model->centers.size(), 0);
	//}
}

ConvolutionRenderer::ConvolutionRenderer(Model *model, ConvolutionRenderer::SHADERMODE mode, const Eigen::Matrix4f& projection, std::string data_path, int camera_heigth, int camera_width) {
	this->camera_heigth = camera_heigth;
	this->camera_width = camera_width;
	this->data_path = data_path;
	this->model = model;
	this->init(mode);
	this->projection = projection;
}

void ConvolutionRenderer::send_vertices_to_shader(std::string vertices_name) {

	bool success = vertexbuffer.create(); assert(success);
	vertexbuffer.setUsagePattern(QGLBuffer::StaticDraw);
	success = vertexbuffer.bind(); assert(success);
	vertexbuffer.allocate(points.data(), sizeof(points[0]) * points.size());
	program.setAttributeBuffer(vertices_name.c_str(), GL_FLOAT, 0, 3);
	program.enableAttributeArray(vertices_name.c_str());
}

void ConvolutionRenderer::setup_canvas() {

	points = std::vector<Eigen::Vector3f>(4, Eigen::Vector3f::Zero());
	points[0] = Eigen::Vector3f(-1, -1, 0); points[1] = Eigen::Vector3f(1, -1, 0);
	points[2] = Eigen::Vector3f(-1, 1, 0); points[3] = Eigen::Vector3f(1, 1, 0);
	send_vertices_to_shader("position");

	/// Specify window bounds
	glUniform1f(glGetUniformLocation(program.programId(), "window_left"), window_left);
	glUniform1f(glGetUniformLocation(program.programId(), "window_bottom"), window_bottom);
	glUniform1f(glGetUniformLocation(program.programId(), "window_height"), window_height);
	glUniform1f(glGetUniformLocation(program.programId(), "window_width"), window_width);
}

void create_smooth_thumb_fold_and_realistic_thumb_top(Model * model, std::vector<glm::vec3> & new_centers, std::vector<float> & new_radii, std::vector<glm::ivec3> & new_blocks, std::vector<Tangent> & new_tangent_points) {

	auto project_point_on_segment = [](const glm::vec3 & p, const glm::vec3 & c1, const glm::vec3 & c2) {
		glm::vec3 u = c2 - c1; glm::vec3 v = p - c1;
		float alpha = dot(u, v) / dot(u, u);
		if (alpha <= 0) return c1;
		if (alpha > 0 && alpha < 1) return c1 + alpha * u;
		if (alpha >= 1) return c2;
	};

	/// Shift palm-fold center
	float thumb_fold_alpha = glm::length(new_centers[33] - new_centers[17]) / glm::length(new_centers[17] - new_centers[18]);
	float thumb_fold_beta = 1 - thumb_fold_alpha;
	//new_centers[33] = new_centers[33] + new_radii[17] * (new_centers[24] - new_centers[33]) / glm::length(new_centers[24] - new_centers[33]);

	/// Shift palm-thumb
	{

		/*if (model->calibration_type == FULL &&
		(model->beta[model->shape_dofs_name_to_id_map["index_palm_center_x"]] + model->beta[model->shape_dofs_name_to_id_map["palm_index_radius"]] >
		model->beta[model->shape_dofs_name_to_id_map["index_base_x"]] + model->beta[model->shape_dofs_name_to_id_map["index_base_radius"]])) {
		new_centers[24] = new_centers[23] + (new_radii[23] - new_radii[24]) * glm::vec3(1, 0, 0);
		}*/
		glm::vec3 projection = project_point_on_segment(new_centers[24], new_centers[15], new_centers[14]);
		new_centers[24] = projection + (new_radii[15] - new_radii[24]) * (new_centers[24] - projection) / glm::length(new_centers[24] - projection);

		glm::vec3 palm_back = model->centers[model->centers_name_to_id_map["palm_back"]];
		glm::vec3 index_base = model->centers[model->centers_name_to_id_map["index_base"]];
		glm::vec3 pinky_base = model->centers[model->centers_name_to_id_map["pinky_base"]];
		glm::vec3 palm_normal = glm::cross(palm_back - index_base, palm_back - pinky_base);
		palm_normal = palm_normal / glm::length(palm_normal);
		glm::vec3 camera_direction = glm::vec3(0, 0, 1);
		if (glm::dot(camera_direction, palm_normal) > 0.3) {
			new_centers[24] += 0.3f * new_radii[15] * glm::vec3(0, 0, 1);
			new_centers[23][0] = new_centers[15][0] + new_radii[15] - new_radii[23];
			//new_centers[23][0] = new_centers[24][0] - (new_radii[23] - 1.8 * new_radii[24]);
			new_centers[23][2] = new_centers[15][2] + 0.5 * (new_radii[15] - new_radii[23]);
		}


		size_t block_index = 24;
		model->compute_tangent_point(model->camera_ray, new_centers[new_blocks[block_index][0]], new_centers[new_blocks[block_index][1]], new_centers[new_blocks[block_index][2]],
			new_radii[new_blocks[block_index][0]], new_radii[new_blocks[block_index][1]], new_radii[new_blocks[block_index][2]], new_tangent_points[block_index].v1,
			new_tangent_points[block_index].v2, new_tangent_points[block_index].v3, new_tangent_points[block_index].u1, new_tangent_points[block_index].u2,
			new_tangent_points[block_index].u3, new_tangent_points[block_index].n, new_tangent_points[block_index].m);
		block_index = 25;
		model->compute_tangent_point(model->camera_ray, new_centers[new_blocks[block_index][0]], new_centers[new_blocks[block_index][1]], new_centers[new_blocks[block_index][2]],
			new_radii[new_blocks[block_index][0]], new_radii[new_blocks[block_index][1]], new_radii[new_blocks[block_index][2]], new_tangent_points[block_index].v1,
			new_tangent_points[block_index].v2, new_tangent_points[block_index].v3, new_tangent_points[block_index].u1, new_tangent_points[block_index].u2,
			new_tangent_points[block_index].u3, new_tangent_points[block_index].n, new_tangent_points[block_index].m);
		block_index = 19;
		model->compute_tangent_point(model->camera_ray, new_centers[new_blocks[block_index][0]], new_centers[new_blocks[block_index][1]], new_centers[new_blocks[block_index][2]],
			new_radii[new_blocks[block_index][0]], new_radii[new_blocks[block_index][1]], new_radii[new_blocks[block_index][2]], new_tangent_points[block_index].v1,
			new_tangent_points[block_index].v2, new_tangent_points[block_index].v3, new_tangent_points[block_index].u1, new_tangent_points[block_index].u2,
			new_tangent_points[block_index].u3, new_tangent_points[block_index].n, new_tangent_points[block_index].m);
	}



	/// Shift finger bases if the are infront of the palm
	bool shifting = false;
	if (shifting) {
		glm::vec3 palm_back = model->centers[model->centers_name_to_id_map["palm_back"]];
		glm::vec3 index_base = model->centers[model->centers_name_to_id_map["index_base"]];
		glm::vec3 pinky_base = model->centers[model->centers_name_to_id_map["pinky_base"]];
		glm::vec3 palm_normal = glm::cross(palm_back - index_base, palm_back - pinky_base);
		palm_normal = palm_normal / glm::length(palm_normal);
		glm::vec3 camera_direction = glm::vec3(0, 0, 1);
		if (glm::dot(camera_direction, palm_normal) > 0.6) {

			auto shift_center_behind_model_surface = [&](std::string first_center_name, std::string second_center_name, std::string shifted_center_name) {
				glm::vec3 a = new_centers[model->centers_name_to_id_map[first_center_name]] - new_radii[model->centers_name_to_id_map[first_center_name]] * glm::vec3(0, 0, 1);
				glm::vec3 b = new_centers[model->centers_name_to_id_map[second_center_name]] - new_radii[model->centers_name_to_id_map[second_center_name]] * glm::vec3(0, 0, 1);
				glm::vec3 c = new_centers[model->centers_name_to_id_map[shifted_center_name]] - new_radii[model->centers_name_to_id_map[shifted_center_name]] * glm::vec3(0, 0, 1);
				glm::vec3 p = project_point_on_segment(c, a, b);
				if (c[2] < p[2]) {
					new_centers[model->centers_name_to_id_map[shifted_center_name]] += glm::vec3(0, 0, p[2] - c[2] + 1.0);
				}
			};

			shift_center_behind_model_surface("palm_index", "palm_pinky", "index_base");
			shift_center_behind_model_surface("palm_index", "palm_pinky", "middle_base");
			shift_center_behind_model_surface("palm_index", "palm_pinky", "ring_base");
			shift_center_behind_model_surface("palm_index", "palm_pinky", "pinky_base");

			shift_center_behind_model_surface("index_base", "index_bottom", "index_membrane");
			shift_center_behind_model_surface("middle_base", "middle_bottom", "middle_membrane");
			shift_center_behind_model_surface("ring_base", "ring_bottom", "ring_membrane");
			shift_center_behind_model_surface("pinky_base", "pinky_bottom", "pinky_membrane");

			for (size_t block_index = 20; block_index < 26; block_index++) {
				model->compute_tangent_point(model->camera_ray, new_centers[new_blocks[block_index][0]], new_centers[new_blocks[block_index][1]], new_centers[new_blocks[block_index][2]],
					new_radii[new_blocks[block_index][0]], new_radii[new_blocks[block_index][1]], new_radii[new_blocks[block_index][2]], new_tangent_points[block_index].v1,
					new_tangent_points[block_index].v2, new_tangent_points[block_index].v3, new_tangent_points[block_index].u1, new_tangent_points[block_index].u2,
					new_tangent_points[block_index].u3, new_tangent_points[block_index].n, new_tangent_points[block_index].m);
			}
		}
	}

	auto process_new_block = [=](size_t block_index, glm::ivec3 block, std::vector<glm::vec3> & centers, const std::vector<float> & radii, std::vector<glm::ivec3> & blocks, std::vector<Tangent> & tangent_points) {

		if (block_index == blocks.size()) {
			blocks.push_back(glm::ivec3(0));
			tangent_points.push_back(Tangent());
		}
		if (block_index >= blocks.size()) cout << "block index is outsize of the range" << endl;

		/// reindex
		if (radii[block[0]] <= radii[block[1]] && radii[block[1]] <= radii[block[2]]) block = glm::ivec3(block[2], block[1], block[0]);
		if (radii[block[0]] <= radii[block[2]] && radii[block[2]] <= radii[block[1]]) block = glm::ivec3(block[1], block[2], block[0]);
		if (radii[block[1]] <= radii[block[0]] && radii[block[0]] <= radii[block[2]]) block = glm::ivec3(block[2], block[0], block[1]);
		if (radii[block[1]] <= radii[block[2]] && radii[block[2]] <= radii[block[0]]) block = glm::ivec3(block[0], block[2], block[1]);
		if (radii[block[2]] <= radii[block[0]] && radii[block[0]] <= radii[block[1]]) block = glm::ivec3(block[1], block[0], block[2]);

		blocks[block_index] = block;

		/// compute tangent points
		model->compute_tangent_point(model->camera_ray, centers[blocks[block_index][0]], centers[blocks[block_index][1]], centers[blocks[block_index][2]],
			radii[blocks[block_index][0]], radii[blocks[block_index][1]], radii[blocks[block_index][2]], tangent_points[block_index].v1,
			tangent_points[block_index].v2, tangent_points[block_index].v3, tangent_points[block_index].u1, tangent_points[block_index].u2,
			tangent_points[block_index].u3, tangent_points[block_index].n, tangent_points[block_index].m);
	};

	/// Smooth thumb fold
	{
		/// new thumb fold 38
		float alpha, beta, gamma; glm::vec3 new_center; float new_radius;
		alpha = 0.63f; beta = 0.18f; gamma = 0.19f;
		new_center = alpha * new_centers[24] + beta * new_centers[33] + gamma * new_centers[19];
		new_radius = alpha * new_radii[24] + beta * new_radii[33] + gamma * new_radii[19];
		new_centers[38] = new_center;
		new_radii[38] = new_radius;

		/// new thumb fold 39
		alpha = 0.42f; beta = 0.32f; gamma = 0.26f;
		new_center = alpha * new_centers[24] + beta * new_centers[33] + gamma * new_centers[19];
		new_radius = alpha * new_radii[24] + beta * new_radii[33] + gamma * new_radii[19];
		new_centers.push_back(new_center);
		new_radii.push_back(new_radius);

		/// new thumb fold 40
		alpha = 0.25f; beta = 0.55f; gamma = 0.20f;
		new_center = alpha * new_centers[24] + beta * new_centers[33] + gamma * new_centers[19];
		new_radius = alpha * new_radii[24] + beta * new_radii[33] + gamma * new_radii[19];
		new_centers.push_back(new_center);
		new_radii.push_back(new_radius);

		/// new center inside of thumb middle phalange 41
		float length = glm::length(new_centers[17] - new_centers[18]);
		alpha = (thumb_fold_alpha * length + thumb_fold_alpha * new_radii[18] + thumb_fold_beta * new_radii[17] - new_radii[33]) / length;
		beta = (thumb_fold_beta * length - thumb_fold_alpha * new_radii[18] - thumb_fold_beta * new_radii[17] + new_radii[33]) / length;
		new_center = alpha * new_centers[18] + beta * new_centers[17];
		new_radius = alpha * new_radii[18] + beta * new_radii[17];
		new_centers.push_back(new_center);
		new_radii.push_back(new_radius);

		process_new_block(14, glm::ivec3(18, 19, 40), new_centers, new_radii, new_blocks, new_tangent_points);

		process_new_block(26, glm::ivec3(19, 24, 38), new_centers, new_radii, new_blocks, new_tangent_points);
		process_new_block(30, glm::ivec3(19, 38, 39), new_centers, new_radii, new_blocks, new_tangent_points);
		process_new_block(31, glm::ivec3(19, 39, 40), new_centers, new_radii, new_blocks, new_tangent_points);
		process_new_block(32, glm::ivec3(19, 40, 33), new_centers, new_radii, new_blocks, new_tangent_points);
		process_new_block(33, glm::ivec3(18, 41, 40), new_centers, new_radii, new_blocks, new_tangent_points);
	}

	/// Realistic thumb top
	{
		size_t block_id = 27;
		Vec3f offset = model->phalanges[model->phalanges_name_to_id_map["HandThumb3"]].offsets[1].cast<float>();
		Mat4f global = model->phalanges[model->phalanges_name_to_id_map["HandThumb3"]].global;
		float new_radius = 1.1 * new_radii[32];

		/// new thumb top 32
		float offset_z = 3.2 - (model->phalanges[model->phalanges_name_to_id_map["HandThumb3"]].offsets[1][2] - model->phalanges[model->phalanges_name_to_id_map["HandThumb3"]].offsets[0][2]);
		if (offset_z < 0) offset_z = 0;

		offset = offset + Vec3f(0, -0.8, offset_z);
		Vec3f new_center = global.block(0, 0, 3, 3) * offset;
		new_centers[32] = new_centers[17] + glm::vec3(new_center[0], new_center[1], new_center[2]);
		new_radii[32] = new_radius;

		/// new center 42
		new_center = Vec3f(new_centers[17][0], new_centers[17][1], new_centers[17][2]) + global.block(0, 0, 3, 3) * (offset + Vec3f(0.7f, -1.2f, 0));
		new_centers.push_back(glm::vec3(new_center[0], new_center[1], new_center[2]));
		new_radii.push_back(new_radius);

		/// new center 43
		new_center = Vec3f(new_centers[17][0], new_centers[17][1], new_centers[17][2]) + global.block(0, 0, 3, 3) * (offset + Vec3f(-0.7f, -1.2f, 0));
		new_centers.push_back(glm::vec3(new_center[0], new_center[1], new_center[2]));
		new_radii.push_back(new_radius);

		process_new_block(34, glm::ivec3(16, 32, 43), new_centers, new_radii, new_blocks, new_tangent_points);
		process_new_block(block_id, glm::ivec3(16, 32, 42), new_centers, new_radii, new_blocks, new_tangent_points);
	}
}

void pass_centers_radii_block_to_shader(GLuint program_id, const std::vector<glm::vec3> & centers, const std::vector<float> & radii, const std::vector<glm::ivec3> & blocks, const std::vector<Tangent> & tangent_points) {
	glUniform1f(glGetUniformLocation(program_id, "num_blocks"), blocks.size());
	glUniform3fv(glGetUniformLocation(program_id, "centers"), centers.size(), (GLfloat *)centers.data());
	glUniform1fv(glGetUniformLocation(program_id, "radii"), radii.size(), (GLfloat *)radii.data());
	glUniform3iv(glGetUniformLocation(program_id, "blocks"), blocks.size(), (GLint *)blocks.data());

	std::vector<Eigen::Vector3f> tangents_v1 = std::vector<Eigen::Vector3f>(tangent_points.size(), Eigen::Vector3f());
	std::vector<Eigen::Vector3f> tangents_v2 = std::vector<Eigen::Vector3f>(tangent_points.size(), Eigen::Vector3f());
	std::vector<Eigen::Vector3f> tangents_v3 = std::vector<Eigen::Vector3f>(tangent_points.size(), Eigen::Vector3f());
	std::vector<Eigen::Vector3f> tangents_u1 = std::vector<Eigen::Vector3f>(tangent_points.size(), Eigen::Vector3f());
	std::vector<Eigen::Vector3f> tangents_u2 = std::vector<Eigen::Vector3f>(tangent_points.size(), Eigen::Vector3f());
	std::vector<Eigen::Vector3f> tangents_u3 = std::vector<Eigen::Vector3f>(tangent_points.size(), Eigen::Vector3f());

	for (size_t i = 0; i < tangent_points.size(); i++) {
		if (tangents_v1.size() <= i) tangents_v1.push_back(Eigen::Vector3f(0));
		if (tangents_v2.size() <= i) tangents_v2.push_back(Eigen::Vector3f(0));
		if (tangents_v3.size() <= i) tangents_v3.push_back(Eigen::Vector3f(0));
		if (tangents_u1.size() <= i) tangents_u1.push_back(Eigen::Vector3f(0));
		if (tangents_u2.size() <= i) tangents_u2.push_back(Eigen::Vector3f(0));
		if (tangents_u3.size() <= i) tangents_u3.push_back(Eigen::Vector3f(0));

		tangents_v1[i] = Eigen::Vector3f(tangent_points[i].v1[0], tangent_points[i].v1[1], tangent_points[i].v1[2]);
		tangents_v2[i] = Eigen::Vector3f(tangent_points[i].v2[0], tangent_points[i].v2[1], tangent_points[i].v2[2]);
		tangents_v3[i] = Eigen::Vector3f(tangent_points[i].v3[0], tangent_points[i].v3[1], tangent_points[i].v3[2]);
		tangents_u1[i] = Eigen::Vector3f(tangent_points[i].u1[0], tangent_points[i].u1[1], tangent_points[i].u1[2]);
		tangents_u2[i] = Eigen::Vector3f(tangent_points[i].u2[0], tangent_points[i].u2[1], tangent_points[i].u2[2]);
		tangents_u3[i] = Eigen::Vector3f(tangent_points[i].u3[0], tangent_points[i].u3[1], tangent_points[i].u3[2]);
	}

	glUniform3fv(glGetUniformLocation(program_id, "tangents_v1"), tangents_v1.size(), (GLfloat *)tangents_v1.data());
	glUniform3fv(glGetUniformLocation(program_id, "tangents_v2"), tangents_v2.size(), (GLfloat *)tangents_v2.data());
	glUniform3fv(glGetUniformLocation(program_id, "tangents_v3"), tangents_v3.size(), (GLfloat *)tangents_v3.data());
	glUniform3fv(glGetUniformLocation(program_id, "tangents_u1"), tangents_u1.size(), (GLfloat *)tangents_u1.data());
	glUniform3fv(glGetUniformLocation(program_id, "tangents_u2"), tangents_u2.size(), (GLfloat *)tangents_u2.data());
	glUniform3fv(glGetUniformLocation(program_id, "tangents_u3"), tangents_u3.size(), (GLfloat *)tangents_u3.data());
}

void ConvolutionRenderer::pass_model_to_shader(bool fingers_only, bool use_indicator_texture, bool wristband_found) {

	if (mode == FRAMEBUFFER) {
		glm::vec3 min_x_world = glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
		glm::vec3 min_y_world = glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
		glm::vec3 max_x_world = -glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
		glm::vec3 max_y_world = -glm::vec3(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());

		int num_centers = model->centers.size();
		if (fingers_only) num_centers = 34;
		for (size_t i = 0; i < num_centers; i++) {
			if (model->centers[i][0] - model->radii[i] < min_x_world[0]) min_x_world = model->centers[i] - model->radii[i];
			if (model->centers[i][1] - model->radii[i] < min_y_world[1]) min_y_world = model->centers[i] - model->radii[i];
			if (model->centers[i][0] + model->radii[i] > max_x_world[0]) max_x_world = model->centers[i] + model->radii[i];
			if (model->centers[i][1] + model->radii[i] > max_y_world[1]) max_y_world = model->centers[i] + model->radii[i];
		}
		glm::vec3 min_x_window = world_to_window_coordinates(min_x_world);
		glm::vec3 min_y_window = world_to_window_coordinates(min_y_world);
		glm::vec3 max_x_window = world_to_window_coordinates(max_x_world);
		glm::vec3 max_y_window = world_to_window_coordinates(max_y_world);

		glUniform1f(glGetUniformLocation(program.programId(), "min_x"), min_x_window[0]);
		glUniform1f(glGetUniformLocation(program.programId(), "min_y"), min_y_window[1]);
		glUniform1f(glGetUniformLocation(program.programId(), "max_x"), max_x_window[0]);
		glUniform1f(glGetUniformLocation(program.programId(), "max_y"), max_y_window[1]);

		glUniform1i(glGetUniformLocation(program.programId(), "fingers_only"), fingers_only);
		glUniform1i(glGetUniformLocation(program.programId(), "use_indicator_texture"), use_indicator_texture);
		/*for (size_t i = 0; i < 4; i++) {
		for (size_t j= 0; j< 4; j++) {
		cout << camera.MVP_glm[i][j] << " ";
		}
		cout << endl;
		}
		cout << endl << endl;

		cout << "min_x_world = " << min_x_world[0] << endl;
		cout << "min_y_world = " << min_y_world[1] << endl;
		cout << "max_x_world = " << max_x_world[0] << endl;
		cout << "max_y_world = " << max_y_world[1] << endl;

		cout << "min_x = " << min_x_window[0] << endl;
		cout << "min_y = " << min_y_window[1] << endl;
		cout << "max_x = " << max_x_window[0] << endl;
		cout << "max_y = " << max_y_window[1] << endl;*/
	}

	tangents_v1 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_v2 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_v3 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_u1 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_u2 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());
	tangents_u3 = std::vector<Eigen::Vector3f>(model->tangent_points.size(), Eigen::Vector3f());

	bool adjust_shape_for_display = true;

	if (adjust_shape_for_display) {
		if (mode == FRAMEBUFFER && fingers_only == false) {
			model->radii[32] = 1.2 * 0.67 * model->radii[16];
			model->radii[38] = 1.5 * model->radii_template[38];
			model->radii[33] = 1.6 * model->radii_template[33];
			//model->radii[24] = 10 * model->radii_template[24];
			pass_centers_radii_block_to_shader(program.programId(), model->centers, model->radii, model->blocks, model->tangent_points);
			model->radii[32] = 0.67 * model->radii[16];
			model->radii[38] = model->radii_template[38];
			model->radii[33] = model->radii_template[33];
			//model->radii[24] = model->radii_template[24];
		}
		if (mode == NORMAL) {
			std::vector<glm::vec3> new_centers = model->centers;
			std::vector<float> new_radii = model->radii;
			std::vector<glm::ivec3> new_blocks = model->blocks;
			std::vector<Tangent> new_tangent_points = model->tangent_points;
			create_smooth_thumb_fold_and_realistic_thumb_top(model, new_centers, new_radii, new_blocks, new_tangent_points);
			pass_centers_radii_block_to_shader(program.programId(), new_centers, new_radii, new_blocks, new_tangent_points);

			glUniform1i(glGetUniformLocation(program.programId(), "num_frames_since_calibrated"), num_frames_since_calibrated);
		}
	}
	else {
		pass_centers_radii_block_to_shader(program.programId(), model->centers, model->radii, model->blocks, model->tangent_points);
	}

	glUniform1i(glGetUniformLocation(program.programId(), "wristband_found"), wristband_found);
}

void ConvolutionRenderer::pass_beta_certainty_to_shader() {

	glUniform1i(glGetUniformLocation(program.programId(), "display_certainty"), display_estimated_certainty || display_measured_certainty);
	if (!display_estimated_certainty && !display_measured_certainty) return;
	if (display_estimated_certainty && display_measured_certainty) {
		cout << "measured and estimated certainty active - error" << endl; return;
	}

	static std::vector<string> bottom_length_names = { "index_bottom_length" , "middle_bottom_length" , "ring_bottom_length" , "pinky_bottom_length" };
	static std::vector<string> base_names = { "index_base_y" , "middle_base_y" , "ring_base_y" , "pinky_base_y" };

	std::map <int, float> shape_dof_to_max_certainty;

	if (display_estimated_certainty) {

		/// Length
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_bottom_length"]] = 10.0f; // 0
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_middle_length"]] = 10.0f; // 1
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_top_length"]] = 5.0f; // 2

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_bottom_length"]] = 30.0f; // 3
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_middle_length"]] = 10.0f; // 4
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_top_length"]] = 7.0f; // 5

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_bottom_length"]] = 30.0f; // 6
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_middle_length"]] = 10.0f; // 7
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_top_length"]] = 8.0f; // 8

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_bottom_length"]] = 30.0f; // 9
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_middle_length"]] = 10.0f; // 10
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_top_length"]] = 8.0f; // 11

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_bottom_length"]] = 25.0f; // 12
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_middle_length"]] = 8.0f; // 13 
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_top_length"]] = 5.0f; // 14 

		/// Bases
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_base_y"]] = 30.0f; // 16
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_base_y"]] = 30.0f; // 19
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_base_y"]] = 30.0f; // 22
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_base_y"]] = 30.0f; // 25
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_base_y"]] = 30.0f; // 28

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_palm_center_x"]] = 0.3f;// 30
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_palm_center_x"]] = 10.0f; //32
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_base_x"]] = 15.0f; //24
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["right_palm_center_x"]] = 3.0f; //36
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["wrist_width"]] = 30.0f; //40
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_membrane_length"]] = 4.0f; //42
	}

	if (display_measured_certainty) {
		/// Length
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_bottom_length"]] = 6.0f; // 0
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_middle_length"]] = 3.0f; // 1
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_top_length"]] = 0.5f; // 2

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_bottom_length"]] = 3.5f; // 3
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_middle_length"]] = 2.5f; // 4
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_top_length"]] = 0.3f; // 5

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_bottom_length"]] = 4.0f; // 6
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_middle_length"]] = 2.5f; // 7
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_top_length"]] = 0.5f; // 8

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_bottom_length"]] = 4.0f; // 9
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_middle_length"]] = 2.5f; // 10
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_top_length"]] = 0.5f; // 11

		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_bottom_length"]] = 2.0f; // 12
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_middle_length"]] = 1.5f; // 13 
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_top_length"]] = 0.4f; // 14 

		/// Bases
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_base_y"]] = 7.0f; // 16
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_base_y"]] = 9.0f; // 19
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["middle_base_y"]] = 9.0f; // 22
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["ring_base_y"]] = 7.0f; // 25
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_base_y"]] = 6.0f; // 28

		/// Palm
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["index_palm_center_x"]] = 0.03f; // 30
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["pinky_palm_center_x"]] = 1.5f; // 32
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_base_x"]] = 12.0f; // 24
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["right_palm_center_x"]] = 1.5f; // 36
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["wrist_width"]] = 4.0f; // 40 
		shape_dof_to_max_certainty[model->shape_dofs_name_to_id_map["thumb_membrane_length"]] = 0.02f; // 42
	}

	//for (auto it = shape_dof_to_max_certainty.begin(); it != shape_dof_to_max_certainty.end(); it++) it->second /= 5;

	std::map<size_t, float> certainties_map;
	for (size_t i = 0; i < model->num_betas; i++) {
		if (model->shape_dofs[i].type == BetaType::PHALANGE_LENGTH || model->shape_dofs[i].type == BetaType::TOP_PHALANGE_LENGTH) {
			size_t center_id = model->shape_dofs[i].center_id;
			if (i == 42) center_id = 33;

			float max_certainty = shape_dof_to_max_certainty[i];
			center_index_to_length_certainty_map[center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);

			/// If bottom phalange multiply uncertainty by base uncertainty
			for (size_t j = 0; j < bottom_length_names.size(); j++) {
				if (i == model->shape_dofs_name_to_id_map[bottom_length_names[j]]) {
					size_t base_index = model->shape_dofs_name_to_id_map[base_names[j]];
					float base_certainty = std::min(1.0f, model->beta_certainty[base_index] / shape_dof_to_max_certainty[base_index]);

					center_index_to_length_certainty_map[center_id] = base_certainty;
				}
			}
			if (model->shape_dofs[i].type == BetaType::TOP_PHALANGE_LENGTH) {
				center_index_to_length_certainty_map[model->shape_dofs[i].top_center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
			}
			certainties_map[i] = (center_index_to_length_certainty_map[center_id]);
		}
		if (model->shape_dofs[i].type == BetaType::FINGER_BASE_X || model->shape_dofs[i].type == BetaType::FINGER_BASE_Y) {
			float max_certainty = shape_dof_to_max_certainty[i];
			if (model->shape_dofs[i].type == BetaType::FINGER_BASE_X) {
				center_index_to_x_certainty_map[model->shape_dofs[i].center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
				center_index_to_x_certainty_map[model->shape_dofs[i].attachment_center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
				certainties_map[i] = (center_index_to_x_certainty_map[model->shape_dofs[i].center_id]);
			}
			if (model->shape_dofs[i].type == BetaType::FINGER_BASE_Y) {
				center_index_to_y_certainty_map[model->shape_dofs[i].center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
				center_index_to_y_certainty_map[model->shape_dofs[i].attachment_center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
				certainties_map[i] = (center_index_to_y_certainty_map[model->shape_dofs[i].center_id]);
			}
		}
		if (model->shape_dofs[i].type == BetaType::PALM_CENTER_X) {
			float max_certainty = shape_dof_to_max_certainty[i];
			center_index_to_x_certainty_map[model->shape_dofs[i].center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
			certainties_map[i] = (center_index_to_x_certainty_map[model->shape_dofs[i].center_id]);
		}
		if (model->shape_dofs[i].type == BetaType::PALM_CENTER_Y) {
			float max_certainty = shape_dof_to_max_certainty[i];
			center_index_to_y_certainty_map[model->shape_dofs[i].center_id] = std::min(1.0f, model->beta_certainty[i] / max_certainty);
			certainties_map[i] = (center_index_to_y_certainty_map[model->shape_dofs[i].center_id]);
		}
	}

	/*for (size_t i = 0; i < center_index_to_y_certainty_map.size(); i++) {
	cout << i << ":   " << std::setprecision(3) << center_index_to_length_certainty_map[i] << ", " << center_index_to_x_certainty_map[i] << ", " << center_index_to_y_certainty_map[i] << endl;
	} cout << endl << endl;*/

	float min_certainty = std::numeric_limits<float>::max();
	size_t min_index = 0;
	for (auto it : certainties_map) {
		if (it.second < min_certainty) {
			min_certainty = it.second;
			min_index = it.first;
		}
	}
	//cout << min_index << " " << min_certainty << endl;
	if (min_certainty > 0.75) {
		//if (num_frames_since_calibrated < 0) num_frames_since_calibrated = 0;
	}

	glUniform1fv(glGetUniformLocation(program.programId(), "center_index_to_length_certainty_map"), center_index_to_length_certainty_map.size(), (GLfloat *)center_index_to_length_certainty_map.data());
	glUniform1fv(glGetUniformLocation(program.programId(), "center_index_to_x_certainty_map"), center_index_to_x_certainty_map.size(), (GLfloat *)center_index_to_x_certainty_map.data());
	glUniform1fv(glGetUniformLocation(program.programId(), "center_index_to_y_certainty_map"), center_index_to_y_certainty_map.size(), (GLfloat *)center_index_to_y_certainty_map.data());
	glUniform1fv(glGetUniformLocation(program.programId(), "center_index_to_radius_certainty_map"), center_index_to_radius_certainty_map.size(), (GLfloat *)center_index_to_radius_certainty_map.data());
}

void ConvolutionRenderer::setup_texture() {
	QImage texture_image;
	if (!texture_image.load(QString::fromUtf8(data_path.c_str()) + "shaders//skin_texture.png")) std::cerr << "error loading" << std::endl;
	QImage formatted_image = QGLWidget::convertToGLFormat(texture_image);
	if (formatted_image.isNull()) std::cerr << "error formatting" << std::endl;

	const GLfloat vtexcoord[] = { 0, 0, 1, 0, 0, 1, 1, 1 };

	glGenTextures(1, &synthetic_texture_id);
	glBindTexture(GL_TEXTURE_2D, synthetic_texture_id);

	bool success = texturebuffer.create(); assert(success);
	texturebuffer.setUsagePattern(QGLBuffer::StaticDraw);
	success = texturebuffer.bind(); assert(success);
	texturebuffer.allocate(vtexcoord, sizeof(vtexcoord));
	program.setAttributeBuffer("vtexcoord", GL_FLOAT, 0, 2);
	program.enableAttributeArray("vtexcoord");

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, formatted_image.width(), formatted_image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, formatted_image.bits());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glUniform1i(glGetUniformLocation(program.programId(), "synthetic_texture"), 0);
}

void ConvolutionRenderer::setup_texture(cv::Mat & image) {

	glGenTextures(1, &real_texture_id);
	glBindTexture(GL_TEXTURE_2D, real_texture_id);

	cv::flip(image, image, 0);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image.cols, image.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, image.ptr());

	glUniform1i(glGetUniformLocation(program.programId(), "real_texture"), 2);

}

void ConvolutionRenderer::setup_silhoeutte() {
	glGenTextures(1, &silhouette_texture_id);
	glBindTexture(GL_TEXTURE_2D, silhouette_texture_id);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, model->silhouette_texture.cols, model->silhouette_texture.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, model->silhouette_texture.ptr());
	glUniform1i(glGetUniformLocation(program.programId(), "silhouette"), 1);

}

void ConvolutionRenderer::init(ConvolutionRenderer::SHADERMODE mode) {
	this->mode = mode;
	if (!vao.isCreated()) {
		bool success = vao.create();
		assert(success);
		vao.bind();
	}

	switch (mode) {
	case NORMAL:
		vertex_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_vshader.glsl";
		fragment_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_fshader.glsl";
		break;
	case FRAMEBUFFER:
		vertex_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_vshader.glsl";
		fragment_shader_name = QString::fromUtf8(data_path.c_str()) + "shaders//" + "model_FB_fshader.glsl";
		window_width = camera_width;
		window_height = camera_heigth;
		break;
	}

	bool vok = program.addShaderFromSourceFile(QGLShader::Vertex, vertex_shader_name);
	bool fok = program.addShaderFromSourceFile(QGLShader::Fragment, fragment_shader_name);
	bool lok = program.link();
	if (!(lok && vok && fok)) {
		std::cout << "shader compile error: " << std::endl;
		std::cout << "vshader: " << vertex_shader_name.toStdString() << std::endl;
		std::cout << "fshader: " << fragment_shader_name.toStdString() << std::endl;
		std::cout << "shaders log: " << program.log().toStdString() << std::endl;
		exit(EXIT_FAILURE);
	}
	bool success = program.bind();
	assert(success);

	setup_canvas();
	setup_texture();
	setup_silhoeutte();

	material.setup(program.programId());
	light.setup(program.programId());

	camera.setup(program.programId(), projection);

	program.release();
	vao.release();
}

void ConvolutionRenderer::render(bool wristband_found) {

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	vao.bind();
	program.bind();

	camera.setup(program.programId(), projection);
	if (real_color) setup_texture(model->real_color);
	pass_model_to_shader(false, false, wristband_found);

	pass_beta_certainty_to_shader();

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, synthetic_texture_id);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, silhouette_texture_id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, model->silhouette_texture.cols, model->silhouette_texture.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, model->silhouette_texture.ptr());

	if (real_color) {
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, real_texture_id);
	}

	glDrawArrays(GL_TRIANGLE_STRIP, 0, points.size());

	program.release();
	vao.release();
}

void ConvolutionRenderer::render_offscreen(bool fingers_only, bool use_indicator_texture) {
	vao.bind();
	program.bind();

	camera.setup(program.programId(), projection);
	pass_model_to_shader(fingers_only, use_indicator_texture, false);

	if (use_indicator_texture) {
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, silhouette_texture_id);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, model->outline_texture.cols, model->outline_texture.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, model->outline_texture.ptr());
	}

	glDrawArrays(GL_TRIANGLE_STRIP, 0, points.size());

	program.release();
	vao.release();
}

