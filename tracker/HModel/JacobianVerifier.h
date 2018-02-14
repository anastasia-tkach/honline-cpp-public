#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "tracker/HModel/Model.h"
#include "tracker/Worker.h"
#include "glm/gtx/string_cast.hpp"
#include "tracker/OpenGL/DebugRenderer/DebugRenderer.h"

class JacobianVerifier {

public:
	float * correspondences_data_points;
	float * correspondences_model_points;
	int * correspondences_model_indices;
	int * correspondences_block_indices;
	float * jacobian_data;
	float * correspondences_silhouette_data_points;
	float * correspondences_silhouette_model_points;
	int * correspondences_silhouette_indices;
	float * jacobian_silhouette;
	bool verify_jacobian;
	int num_sensor_points;
	int num_rendered_points;
	std::vector<double> theta_double;
	std::vector<double> beta_double;

	JacobianVerifier(bool verify_jacobian, Model * model, std::string logs_path);

	~JacobianVerifier();

	void jacobian_theta_double(int phalange_id, const glm::dvec3 & pos, const std::vector<glm::dmat4> theta_infos_globals);

	void jacobian_beta_double(const int shape_unit_id, const glm::dvec3 & q, const glm::dvec3 & s, glm::ivec3 & index, const std::vector<glm::dmat4> beta_infos_globals, std::ostream & ostream_id = std::cout, int beta_id = -1);

	glm::dvec3 function_htrack_parametrization(const size_t block_id, const glm::ivec3 & index, const size_t parameter_id, const double k_alpha, const double k_beta, const double k_gamma, const glm::dvec3 & offset);

	void verify_one_parameter(const size_t block_id, const glm::ivec3 & index, const size_t parameter_id, const double k_alpha, const double k_beta, const double k_gamma, const glm::dvec3 & offset);

	void compute_parametrization(const glm::ivec3 index, const size_t block_id, const glm::dvec3 & model_point, glm::dvec3 & q, glm::dvec3 & s, double & k_alpha, double & k_beta, double & k_gamma, glm::dvec3 & offset, std::ostream & ostream_id = std::cout);

	void verify_all_parameters(const glm::dvec3 & model_point, const size_t joint_id, const size_t shape_unit_id, const std::vector<size_t> & phalange_ids, const std::vector<size_t> & shape_unit_ids, const size_t block_id, glm::ivec3 & index);

	void verify_again_helper(int sensor_point_id, int parameter_id, std::ostream & ostream_id = std::cout);

	void main_verify_jacobian_for_one_point();

	void main_verify_jacobian_for_all_points();

	void write_corresponences_to_file(int id);


private:
	struct Tangent {
		glm::dvec3 v1; glm::dvec3 v2; glm::dvec3 v3; glm::dvec3 n;
		glm::dvec3 u1; glm::dvec3 u2; glm::dvec3 u3; glm::dvec3 m;
		Tangent() {
			v1 = glm::dvec3(0, 0, 0); v2 = glm::dvec3(0, 0, 0); v3 = glm::dvec3(0, 0, 0);
			u1 = glm::dvec3(0, 0, 0); u2 = glm::dvec3(0, 0, 0); u3 = glm::dvec3(0, 0, 0);
		}
	};
	int NUM_CENTERS = 38;

	std::vector<Mat4d> init_locals_double;
	std::vector<Mat4d> current_locals_double;
	std::vector<Mat4d> locals_double;
	std::vector<Mat4d> globals_double;
	std::vector<Vec3d> axis_double;
	std::vector<glm::dvec3> centers_double;
	std::vector<glm::dvec3> offsets_double;
	std::vector<double> radii_double;
	std::vector<glm::dmat4> theta_infos_globals;
	std::vector<glm::dmat4> beta_infos_globals;
	std::vector<Tangent> tangent_points_double;

	std::vector<glm::dvec3> jacobian_numerical;
	std::vector<glm::dvec3> jacobian_analytical;
	std::vector<glm::dvec3> jacobian_data_gpu;
	std::vector<glm::dvec3> jacobian_silhouette_gpu;

	bool verbose = true;
	std::string logs_path;

	Model * model;

	glm::dmat4 convert_eigen_matrix_to_glm_matrix(const Mat4d & eigen_matrix) {
		glm::dmat4 glm_matrix = glm::dmat4();
		for (size_t u = 0; u < 4; u++) {
			for (size_t v = 0; v < 4; v++) {
				glm_matrix[u][v] = eigen_matrix(v, u);
			}
		}
		return glm_matrix;
	}

	void compute_segment_coordinates(const glm::dvec3 & p, const glm::dvec3 & a, const glm::dvec3 & b, double & u, double & v) {
		double ab = glm::length(a - b);
		double ap = glm::length(a - p);
		u = 1 - ap / ab;
		v = 1 - u;
	}

	void compute_triangle_coordinates(const glm::dvec3 & p, const glm::dvec3 & a, const glm::dvec3 & b, const glm::dvec3 & c, double & u, double & v, double & w) {
		glm::dvec3 v0 = b - a;
		glm::dvec3 v1 = c - a;
		glm::dvec3 v2 = p - a;
		double d00 = glm::dot(v0, v0);
		double d01 = glm::dot(v0, v1);
		double d11 = glm::dot(v1, v1);
		double d20 = glm::dot(v2, v0);
		double d21 = glm::dot(v2, v1);
		double denom = d00 * d11 - d01 * d01;
		v = (d11 * d20 - d01 * d21) / denom;
		w = (d00 * d21 - d01 * d20) / denom;
		u = 1.0f - v - w;
	}

	bool is_point_in_triangle(const glm::dvec3 & p, const glm::dvec3 & a, const glm::dvec3 & b, const glm::dvec3 & c) {
		glm::dvec3 v0 = b - a;
		glm::dvec3 v1 = c - a;
		glm::dvec3 v2 = p - a;
		double d00 = dot(v0, v0);
		double d01 = dot(v0, v1);
		double d11 = dot(v1, v1);
		double d20 = dot(v2, v0);
		double d21 = dot(v2, v1);
		double denom = d00 * d11 - d01 * d01;
		double alpha = (d11 * d20 - d01 * d21) / denom;
		double beta = (d00 * d21 - d01 * d20) / denom;
		double gamma = 1.0 - alpha - beta;
		if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1) return true;
		else return false;
	}

	glm::dvec3 project_point_on_segment(const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2) {
		glm::dvec3 u = c2 - c1;
		glm::dvec3 v = p - c1;
		double alpha = dot(u, v) / dot(u, u);
		if (alpha <= 0) return c1;
		if (alpha > 0 && alpha < 1) return c1 + alpha * u;
		if (alpha >= 1) return c2;
	}

	int index_size(glm::ivec3 index) {
		if (index[1] > NUM_CENTERS) return 1;
		if (index[2] > NUM_CENTERS) return 2;
		return 3;
	}

	void projection_convsegment_double(const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2, double r1, double r2, int index1, int index2,
		glm::dvec3 & q, glm::dvec3 & s, glm::ivec3 & index) {

		glm::dvec3 x = c2 - c1;
		double delta_r = r1 - r2;
		double length_x = length(x);
		double length_x2 = length_x * length_x;

		double alpha = dot(x, p - c1) / length_x2;
		glm::dvec3 t = c1 + alpha * x;
		double omega = sqrt(length_x2 - delta_r * delta_r);
		double beta = length(p - t) * delta_r / omega;
		s = t - beta * x / length_x;

		double gamma = delta_r * length(c2 - t + beta * x / length_x) / length_x;
		q = s + (p - s) / length(p - s) * (gamma + r2);
		index = glm::ivec3(index1, index2, RAND_MAX);
	}

	void projection_convtriangle_double(const glm::dvec3 & p, const glm::dvec3 & c1, const glm::dvec3 & c2, const glm::dvec3 & c3,
		double r1, double r2, double r3, int index1, int index2, int index3,
		const glm::dvec3 & v1, const glm::dvec3 & v2, const glm::dvec3 & v3, const glm::dvec3 & n,
		const glm::dvec3 & u1, const glm::dvec3 & u2, const glm::dvec3 & u3, const glm::dvec3 & m, const glm::dvec3 & camera_ray,
		glm::dvec3 & q, glm::dvec3 & s, glm::ivec3 & index) {

		glm::dvec3 l = normalize(cross(c2 - c1, c3 - c1));
		index = glm::ivec3(index1, index2, index3);

		glm::dvec3 s1, s2, q1, q2;
		double cos_alpha, distance;
		bool f1 = false; bool f2 = false;

		if (glm::dot(n, camera_ray) < 0) {
			if (dot(l, n) < 0) l = -l;
			cos_alpha = dot(l, n);
			distance = dot(p - c1, l) / cos_alpha;
			s1 = p - n * distance;
			if (is_point_in_triangle(s1, c1, c2, c3)) {
				f1 = true;
				distance = dot(p - v1, n);
				q1 = p - n * distance;
			}
		}
		if (glm::dot(m, camera_ray) < 0) {
			if (dot(l, m) < 0) l = -l;
			cos_alpha = dot(l, m);
			distance = dot(p - c1, l) / cos_alpha;
			s2 = p - m * distance;
			if (is_point_in_triangle(s2, c1, c2, c3)) {
				f2 = true;
				distance = dot(p - u1, m);
				q2 = p - m * distance;
			}
		}
		if (f1 && f2) {
			if (length(p - q1) < length(p - q2)) { q = q1; s = s1; }
			else { q = q2; s = s2; } return;
		}
		if (f1 && !f2) { q = q1; s = s1; return; }
		if (f2 && !f1) { q = q2; s = s2; return; }
		if (!f1 && !f2) {
			cout << "both trianges are back-facing" << endl;
		}

		glm::dvec3 q12; glm::dvec3 s12; glm::ivec3 index12;
		projection_convsegment_double(p, c1, c2, r1, r2, index1, index2, q12, s12, index12);
		glm::dvec3 q13; glm::dvec3 s13; glm::ivec3 index13;
		projection_convsegment_double(p, c1, c3, r1, r3, index1, index3, q13, s13, index13);
		glm::dvec3 q23; glm::dvec3 s23; glm::ivec3 index23;
		projection_convsegment_double(p, c2, c3, r2, r3, index2, index3, q23, s23, index23);

		double d12 = glm::sign(glm::length(p - s12) - glm::length(q12 - s12)) * glm::length(p - q12);
		double d13 = glm::sign(glm::length(p - s13) - glm::length(q13 - s13)) * glm::length(p - q13);
		double d23 = glm::sign(glm::length(p - s23) - glm::length(q23 - s23)) * glm::length(p - q23);

		// supress sphere projections corresponding to non - existing surface
		if (index_size(index12) == 1 &&
			(index_size(index23) == 2 || index12[0] != index23[0]) &&
			(index_size(index13) == 2 || index12[0] != index13[0]))
			d12 = RAND_MAX;
		if (index_size(index13) == 1 &&
			(index_size(index12) == 2 || index13[0] != index12[0]) &&
			(index_size(index23) == 2 || index13[0] != index23[0]))
			d13 = RAND_MAX;
		if (index_size(index23) == 1 &&
			(index_size(index12) == 2 || index23[0] != index12[0]) &&
			(index_size(index13) == 2 || index23[0] != index13[0]))
			d23 = RAND_MAX;

		if (d12 <= d13 && d12 <= d23) {
			s = s12; q = q12; index = index12;
		}
		if (d13 <= d12 && d13 <= d23) {
			s = s13; q = q13; index = index13;
		}
		if (d23 <= d12 && d23 <= d13) {
			s = s23; q = q23; index = index23;
		}
	}

	void update_phalanges_transformation_double(int phalange_id) {
		Phalange phalange = model->phalanges[phalange_id];
		if (phalange.parent_id >= 0)
			globals_double[phalange_id] = globals_double[phalange.parent_id] * locals_double[phalange_id];
		else
			globals_double[phalange_id] = locals_double[phalange_id];
		for (size_t i = 0; i < phalange.children_ids.size(); i++)
			update_phalanges_transformation_double(phalange.children_ids[i]);
	}

	void transform_joints_double(const std::vector<double> & theta) {
		for (size_t i = 0; i < model->pose_dofs.size(); ++i) {
			if (abs(theta[i]) < 1e-16) continue;
			if (model->pose_dofs[i].phalange_id == -1) continue;
			Mat4d & local = locals_double[model->pose_dofs[i].phalange_id];
			switch (model->pose_dofs[i].type) {
			case TRANSLATION_AXIS: {
				Vec3d t = axis_double[i] * theta[i];
				local(0, 3) += t[0];
				local(1, 3) += t[1];
				local(2, 3) += t[2];
				update_phalanges_transformation_double(model->pose_dofs[i].phalange_id);
				break;
			}
			case ROTATION_AXIS:
				local = local * Transform3d(Eigen::AngleAxisd(theta[i], axis_double[i])).matrix();
				update_phalanges_transformation_double(model->pose_dofs[i].phalange_id);
				break;
			}
		}
	}

	void update_theta_double(bool incremental) {
		for (size_t i = 0; i < num_phalanges + 1; i++) {
			if (incremental)
				locals_double[i] = current_locals_double[i];
			else
				locals_double[i] = init_locals_double[i];
		}
		std::vector<double> rotateX(num_thetas, 0);
		std::vector<double> rotateZ(num_thetas, 0);
		std::vector<double> rotateY(num_thetas, 0);
		std::vector<double> translate(num_thetas, 0);

		for (size_t i = 0; i < num_thetas; ++i) {
			if (model->pose_dofs[i].phalange_id < num_phalanges && model->pose_dofs[i].type == ROTATION_AXIS) {
				if (axis_double[i] == Vec3d(1, 0, 0))
					rotateX[i] = theta_double[i];
				else if (axis_double[i] == Vec3d(0, 1, 0))
					rotateY[i] = theta_double[i];
				else if (axis_double[i] == Vec3d(0, 0, 1))
					rotateZ[i] = theta_double[i];
				else
					cout << "wrong axis" << endl;
			}
			else
				translate[i] = theta_double[i];
		}

		transform_joints_double(translate);
		transform_joints_double(rotateX);
		transform_joints_double(rotateZ);
		transform_joints_double(rotateY);
	}

	void update_beta_double() {
		for (size_t i = 0; i < model->num_betas; i++) {
			int phalange_id = model->shape_dofs[i].phalange_id;
			BetaType beta_type = model->shape_dofs[i].type;

			if ((beta_type == TOP_PHALANGE_LENGTH || beta_type == PHALANGE_LENGTH) && beta_double[i] < 0)
				cout << "error, beta[" << i << "] = " << beta_double[i] << endl;

			if (beta_type == PHALANGE_LENGTH) {
				size_t child_phalange_id = phalange_id + 1;
				Eigen::Vector3d v = init_locals_double[child_phalange_id].block(0, 3, 3, 1);
				Eigen::Vector3d u = v / v.norm();
				if (u[1] < 0)
					cout << "error, direction[" << i << "] = " << u[1] << endl;
				v = beta_double[i] * u;
				init_locals_double[child_phalange_id].block(0, 3, 3, 1) = v;
			}
			if (beta_type == TOP_PHALANGE_LENGTH) {
				Eigen::Vector3d v = model->phalanges[phalange_id].offsets[0];
				v = beta_double[i] * v / v.norm();
				model->phalanges[phalange_id].offsets[0] = v;
			}
			if (beta_type == FINGER_BASE_X) {
				init_locals_double[phalange_id](0, 3) = beta_double[i];
			}
			if (beta_type == FINGER_BASE_Y) {
				init_locals_double[phalange_id](1, 3) = beta_double[i];
			}
			if (beta_type == FINGER_BASE_Z) {
				init_locals_double[phalange_id](2, 3) = beta_double[i];
			}
			if (beta_type == PALM_CENTER_X) {
				if (phalange_id == model->phalanges_name_to_id_map["Hand"]) {
					size_t attachment_id = model->centerid_to_attachment_id_map[model->shape_dofs[i].center_id];
					model->phalanges[phalange_id].offsets[attachment_id][0] = beta_double[i];
				}
			}
			if (beta_type == PALM_CENTER_Y) {
				if (phalange_id == model->phalanges_name_to_id_map["Hand"]) {
					size_t attachment_id = model->centerid_to_attachment_id_map[model->shape_dofs[i].center_id];
					model->phalanges[phalange_id].offsets[attachment_id][1] = beta_double[i];
				}

			}
			if (beta_type == PALM_CENTER_X || beta_type == PALM_CENTER_Y) {
				// recompute attachments palm-middle and palm-ring - replace by something more elegant
				if (phalange_id == model->phalanges_name_to_id_map["Hand"]) {
					double palm_middle_offset_ratio = model->palm_middle_offset_ratio;
					double palm_ring_offset_ratio = model->palm_ring_offset_ratio;
					Vec3d palm_index_offset = model->phalanges[phalange_id].offsets[model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_index"]]];
					Vec3d palm_pinky_offset = model->phalanges[phalange_id].offsets[model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_pinky"]]];
					Vec3d palm_middle_offset = palm_index_offset + palm_middle_offset_ratio * (palm_pinky_offset - palm_index_offset);
					Vec3d palm_ring_offset = palm_index_offset + palm_ring_offset_ratio * (palm_pinky_offset - palm_index_offset);
					model->phalanges[phalange_id].offsets[model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_middle"]]] = palm_middle_offset;
					model->phalanges[phalange_id].offsets[model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_ring"]]] = palm_ring_offset;
				}
			}
			if (beta_type == RADIUS) {
				radii_double[model->shape_dofs[i].center_id] = beta_double[i];
			}
			if (beta_type == FINGER_TOP_Y) {
				Eigen::Vector3d v = model->phalanges[phalange_id].offsets[0];
				v = beta_double[2] * v / v.norm(); model->phalanges[phalange_id].offsets[0] = v;
				model->phalanges[phalange_id].offsets[1][1] = model->phalanges[phalange_id].offsets[0][1] + beta_double[i];
			}
			if (beta_type == FINGER_TOP_Z) {
				Eigen::Vector3d v = model->phalanges[phalange_id].offsets[0];
				v = beta_double[2] * v / v.norm(); model->phalanges[phalange_id].offsets[0] = v;
				model->phalanges[phalange_id].offsets[1][2] = model->phalanges[phalange_id].offsets[0][2] + beta_double[i];
			}
		}
		if (model->calibration_type == FULL) {
			{ /// membranes
				auto compute_membrane_offset = [=](int i, double length, double fraction, Vec3d & offset) {
					Vec3d top = init_locals_double[model->phalanges[i].children_ids[0]].block(0, 3, 3, 1);
					Vec3d membrane = model->phalanges[i].offsets[0];
					glm::dvec3 projection_glm = project_point_on_segment(glm::dvec3(membrane[0], membrane[1], membrane[2]), glm::dvec3(0, 0, 0), glm::dvec3(top[0], top[1], top[2]));
					Vec3d projection = Vec3d(projection_glm[0], projection_glm[1], projection_glm[2]);
					Vec3d radial_offset = model->phalanges[i].offsets[0] - projection;
					projection = fraction * length * projection / projection.norm();
					offset = projection + radial_offset;
				};
				compute_membrane_offset(model->phalanges_name_to_id_map["HandIndex1"], beta_double[3], model->membranes_fractional_length[0], model->phalanges[model->phalanges_name_to_id_map["HandIndex1"]].offsets[0]);
				compute_membrane_offset(model->phalanges_name_to_id_map["HandMiddle1"], beta_double[6], model->membranes_fractional_length[1], model->phalanges[model->phalanges_name_to_id_map["HandMiddle1"]].offsets[0]);
				compute_membrane_offset(model->phalanges_name_to_id_map["HandRing1"], beta_double[9], model->membranes_fractional_length[2], model->phalanges[model->phalanges_name_to_id_map["HandRing1"]].offsets[0]);
				compute_membrane_offset(model->phalanges_name_to_id_map["HandPinky1"], beta_double[12], model->membranes_fractional_length[3], model->phalanges[model->phalanges_name_to_id_map["HandPinky1"]].offsets[0]);
			}
			{ /// wrist
				model->phalanges[model->phalanges_name_to_id_map["Wrist"]].offsets[0][0] = beta_double[40]; // wrist_bottom_left_x
				model->phalanges[model->phalanges_name_to_id_map["Wrist"]].offsets[0][1] = -beta_double[41]; // wrist_bottom_left_y

				model->phalanges[model->phalanges_name_to_id_map["Wrist"]].offsets[1][0] = -beta_double[40]; // wrist_bottom_right_x
				model->phalanges[model->phalanges_name_to_id_map["Wrist"]].offsets[1][1] = -beta_double[41]; // wrist_bottom_right_y

				model->phalanges[model->phalanges_name_to_id_map["Wrist"]].offsets[2][0] = beta_double[45]; // wrist_top_left_x
				model->phalanges[model->phalanges_name_to_id_map["Wrist"]].offsets[3][0] = -beta_double[40]; // wrist_top_right_x

				size_t c = model->calibration_type_to_num_betas_map[FINGERS_AND_PALM];
				radii_double[model->centers_name_to_id_map["wrist_top_right"]] = beta_double[c + 27] + 0.01;
				radii_double[model->centers_name_to_id_map["wrist_bottom_right"]] = beta_double[c + 28] + 0.01;
			}
		}
	}

	void compute_tangent_point_double(const glm::dvec3 & camera_ray, const glm::dvec3 & c1, const glm::dvec3 & c2, const glm::dvec3 & c3, const double r1, const double r2, const double r3,
		glm::dvec3 & v1, glm::dvec3 & v2, glm::dvec3 & v3, glm::dvec3 & u1, glm::dvec3 & u2, glm::dvec3 & u3, glm::dvec3 & n, glm::dvec3 & m) {

		double epsilon = 1e-2;
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
			return;
		}

		glm::dvec3 z12 = c1 + (c2 - c1) * r1 / (r1 - r2);
		glm::dvec3 z13 = c1 + (c3 - c1) * r1 / (r1 - r3);

		glm::dvec3 l = (z12 - z13) / length(z12 - z13);
		double projection = dot(c1 - z12, l);
		glm::dvec3 z = z12 + projection * l;

		double eta = length(c1 - z);
		double sin_beta = r1 / eta;
		double nu = sqrt(eta * eta - r1 * r1);
		double cos_beta = nu / eta;

		glm::dvec3 f = (c1 - z) / eta;
		glm::dvec3 h = cross(l, f);
		normalize(h);

		glm::dvec3 g;

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

	void compute_tangent_points_double() {

		for (size_t i = 0; i < model->blocks.size(); i++) {
			if (model->blocks[i][2] > centers_double.size()) continue;
			compute_tangent_point_double(model->camera_ray, centers_double[model->blocks[i][0]], centers_double[model->blocks[i][1]], centers_double[model->blocks[i][2]],
				radii_double[model->blocks[i][0]], radii_double[model->blocks[i][1]], radii_double[model->blocks[i][2]],
				tangent_points_double[i].v1, tangent_points_double[i].v2, tangent_points_double[i].v3,
				tangent_points_double[i].u1, tangent_points_double[i].u2, tangent_points_double[i].u3,
				tangent_points_double[i].n, tangent_points_double[i].m);
		}
	}

	void update_centers_double() {
		for (size_t i = 0; i < num_phalanges; i++) {
			Vec3d p = globals_double[i].block(0, 3, 3, 1);
			if (model->phalanges[i].center_id != model->centers_name_to_id_map["palm_back"]) // this is because of a bug that I do not understand
				centers_double[model->phalanges[i].center_id] = glm::dvec3(p[0], p[1], p[2]);

			for (size_t j = 0; j < model->phalanges[i].attachments.size(); j++) {
				Vec3d t = globals_double[i].block(0, 0, 3, 3) * model->phalanges[i].offsets[j];
				centers_double[model->phalanges[i].attachments[j]] = glm::dvec3(p[0], p[1], p[2]) + glm::dvec3(t[0], t[1], t[2]);
			}
		}

		double alpha = 0.4; double beta = 0.3; double gamma = 0.3;
		centers_double[model->centers_name_to_id_map["thumb_membrane_middle"]] = alpha * centers_double[model->centers_name_to_id_map["palm_thumb"]] +
			beta * centers_double[model->centers_name_to_id_map["thumb_membrane_left"]] + gamma * centers_double[model->centers_name_to_id_map["thumb_base"]];
		radii_double[model->centers_name_to_id_map["thumb_membrane_middle"]] = alpha * radii_double[model->centers_name_to_id_map["palm_thumb"]] +
			beta * radii_double[model->centers_name_to_id_map["thumb_membrane_left"]] + gamma * radii_double[model->centers_name_to_id_map["thumb_base"]];

		//model->reindex();
		compute_tangent_points_double();
	}

	void convert_theta_infos_to_double() {
		theta_infos_globals = std::vector<glm::dmat4>(num_thetas, glm::dmat4());
		for (size_t i = 0; i < model->pose_dofs.size(); i++) {
			//cout << i << endl;
			if (model->pose_dofs[i].phalange_id != -1) {
				switch (model->pose_dofs[i].type) {
				case TRANSLATION_AXIS: {
					theta_infos_globals[i] = glm::dmat4(1.0); //identity matrix
					break;
				}
				case ROTATION_AXIS:
				default: {
					theta_infos_globals[i] = convert_eigen_matrix_to_glm_matrix(globals_double[model->pose_dofs[i].phalange_id]);
					break;
				}
				}
			}
		}
	}

	void convert_beta_infos_to_double() {
		beta_infos_globals = std::vector<glm::dmat4>(model->num_betas, glm::dmat4());
		for (size_t i = 0; i < model->num_betas; i++) {
			int phalange_id;
			BetaType beta_type = model->shape_dofs[i].type;
			if (beta_type == PHALANGE_LENGTH || beta_type == TOP_PHALANGE_LENGTH || beta_type == PALM_CENTER_X || beta_type == PALM_CENTER_Y || beta_type == RADIUS || beta_type == FINGER_TOP_Y || beta_type == FINGER_TOP_Z)
				phalange_id = model->shape_dofs[i].phalange_id;
			if (beta_type == FINGER_BASE_X || beta_type == FINGER_BASE_Y || beta_type == FINGER_BASE_Z)
				phalange_id = model->phalanges[model->shape_dofs[i].phalange_id].parent_id;

			beta_infos_globals[i] = convert_eigen_matrix_to_glm_matrix(globals_double[phalange_id]);
		}
	}

	void convert_to_doubles() {
		init_locals_double = std::vector<Mat4d>(num_phalanges + 1, Mat4d::Zero());
		current_locals_double = std::vector<Mat4d>(num_phalanges + 1, Mat4d::Zero());
		for (size_t i = 0; i < num_phalanges + 1; i++) {
			init_locals_double[i] = model->phalanges[i].init_local.cast<double>();
		}
		locals_double = std::vector<Mat4d>(num_phalanges + 1, Mat4d::Zero());
		globals_double = std::vector<Mat4d>(num_phalanges + 1, Mat4d::Zero());
		axis_double = std::vector<Vec3d>(model->pose_dofs.size(), Vec3d::Zero());
		for (size_t i = 0; i < model->pose_dofs.size(); i++) {
			axis_double[i] = model->pose_dofs[i].axis.cast<double>();
		}
		centers_double = std::vector<glm::dvec3>(model->centers.size(), glm::dvec3());
		for (size_t i = 0; i < model->centers.size(); i++) {
			centers_double[i] = model->centers[i];
		}
		radii_double = std::vector<double>(model->radii.begin(), model->radii.end());
		tangent_points_double = std::vector<Tangent>(model->blocks.size(), Tangent());

		update_beta_double();
		update_theta_double(false);
		update_centers_double();
		convert_theta_infos_to_double();
		convert_beta_infos_to_double();

		bool print = false;
		if (print) {
			for (size_t i = 0; i < model->num_centers; i++) {
				cout << model->centers[i][0] << ", " << model->centers[i][1] << ", " << model->centers[i][2] << endl;
				cout << centers_double[i][0] << ", " << centers_double[i][1] << ", " << centers_double[i][2] << endl << endl;
			}
		}
	}


};
