#pragma once
#include "cudax/kernel.h"
#include "CorrespondencesFinder.h"

namespace cudax {

	class ComputeJacobianRow {
	protected:
		const glm::mat3x3& inversed_projection_matrix;
		CorrespondencesFinder correspondences_finder;
		float* J_raw;
		float* e_raw;
		ThetaInfo* theta_infos;
		BetaInfo* beta_infos;
		KinematicChain * kinematic_chains;
		ShapeChain * shape_chains;

		float * centers;
		int * blocks;

		bool verify_jacobian;
		float * jacobian_data;
		float * jacobian_silhouette;

		__device__ glm::vec3 undistort_function(glm::vec3 x_d) {

			float k0 = -0.27686;
			float k1 = 0.12105;
			float k2 = -0.035386;
			float k3 = 0;
			float k4 = 0;
			float k5 = 0;
			float cx = -0.0091268;
			float cy = 0.0061785;
			float tx = 0.0021307;
			float ty = 0.00060417;
			float max_r = 0.78384;

			x_d -= glm::vec3(cx, cy, 0);
			glm::vec2 x_n = glm::vec2(x_d[0], x_d[1]);
			float r2, r4, r6;
			for (int iter = 0; iter < 20; iter++) {

				/// Add distortion 
				r2 = x_n[0] * x_n[0] + x_n[1] * x_n[1];
				r4 = r2 * r2;
				r6 = r4 * r2;

				/// Radial distortion :
				float cdist = 1 + k0 * r2 + k1 * r4 + k2 * r6;
				float cdist2 = 1 + k3 * r2 + k4 * r4 + k5 * r6;

				if (cdist == 0) cdist = 1;
				float invCdist = cdist2 / cdist;

				/// Tangential distortion
				glm::vec2 dx = glm::vec2(0, 0);
				dx[0] = 2 * tx * x_n[0] * x_n[1] + ty * (r2 + 2 * x_n[0] * x_n[0]);
				dx[1] = tx * (r2 + 2 * x_n[1] * x_n[1]) + 2 * ty * x_n[0] * x_n[1];

				x_n = invCdist * (glm::vec2(x_d[0], x_d[1]) - dx);
			}

			if (r2 > max_r * max_r) x_n = glm::vec2(0, 0);
			x_n += glm::vec2(cx, cy);
			x_d = glm::vec3(x_n[0], x_n[1], 1);

			return x_d;
		}

		__device__ glm::vec3 get_center(int i) {
			return glm::vec3(centers[D * i], centers[D * i + 1], centers[D * i + 2]);
		}

		__device__ void compute_segment_coordinates(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b, float & u, float & v) {
			float ab = glm::length(a - b);
			float ap = glm::length(a - p);
			u = 1 - ap / ab;
			v = 1 - u;
		}

		__device__ void compute_triangle_coordinates(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c, float & u, float & v, float & w) {
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

	public:
		ComputeJacobianRow(float* J_raw, float* e_raw, bool verify_jacobian) :
			inversed_projection_matrix(cudax::camera_matrix->D_inv_proj_matrix()) {
			assert(J_raw != NULL);
			assert(e_raw != NULL);
			this->J_raw = J_raw;
			this->e_raw = e_raw;
			this->theta_infos = thrust::raw_pointer_cast(&device_pointer_theta_infos[0]);
			this->kinematic_chains = thrust::raw_pointer_cast(&device_pointer_kinematic_chains[0]);

			this->centers = thrust::raw_pointer_cast(device_pointer_centers->data());
			this->blocks = thrust::raw_pointer_cast(device_pointer_blocks->data());

			if (host_calibrate) {
				this->beta_infos = thrust::raw_pointer_cast(&device_pointer_beta_infos[0]);
				this->shape_chains = thrust::raw_pointer_cast(&device_pointer_shape_chains[0]);
			}

			this->verify_jacobian = verify_jacobian;
			if (verify_jacobian) {
				this->jacobian_data = thrust::raw_pointer_cast(_jacobian_data->data());
				this->jacobian_silhouette = thrust::raw_pointer_cast(_jacobian_silhouette->data());
			}
		}

		__device__ float reweight_function(const glm::vec3 & p, const glm::vec3 & q, float weight) {
			float d = glm::length(p - q);

			float w = 1.0f / glm::sqrt(d + 1e-3);
			if (d > 1e-3) weight *= w * 3.5f;

			//weight = weight / (glm::length(p - q) + 1e-3);

			if (weight > 10) weight = 10;
			return weight;
		}

		__device__ void jacobian_theta_row(float* sub_J, const int constraint_index, const int pose_unit_id, const glm::vec3& p, const glm::vec3 & n, float weight, bool data_energy) {
			glm::mat3x2 J_proj;
			if (!data_energy) {
				J_proj = projection_jacobian(p);
			}

			for (int l = 0; l < KINEMATIC_MAX_LENGTH; l++) {
				int theta_info_id = kinematic_chains[pose_unit_id].data[l];
				if (theta_info_id == -1) break;
				const ThetaInfo & theta_info = theta_infos[theta_info_id];
				glm::vec3& axis = theta_infos[theta_info_id].axis;

				glm::vec3 dp;
				switch (theta_info.type) {
				case 1: {
					dp = glm::vec3(theta_infos[theta_info_id].mat * glm::vec4(axis, 1));
					break;
				}
				case 0: {
					glm::vec3 t(theta_infos[theta_info_id].mat[3][0], theta_infos[theta_info_id].mat[3][1], theta_infos[theta_info_id].mat[3][2]);
					glm::vec3 a = glm::normalize(glm::vec3(theta_infos[theta_info_id].mat * glm::vec4(axis, 1)) - t);
					dp = glm::cross(a, p - t);
					break;
				}
				}

				if (isnan(dp[0]) || isnan(dp[1]) || isnan(dp[2]) || isinf(dp[0]) || isinf(dp[1]) || isinf(dp[2])) {
					printf("ComputeJacobianRow: point-gradient is nan or inf in jacobian-theta\n");
				}

				if (data_energy) {
					sub_J[theta_info.jacobian_column] = weight * glm::dot(dp, n);
					if (isnan(sub_J[theta_info.jacobian_column]) || isinf(sub_J[theta_info.jacobian_column])) printf("ComputeJacobianRow: dp is nan or inf\n");
				}
				else {
					glm::vec2 dp_proj = J_proj * dp;
					if (isnan(dp_proj[0]) || isnan(dp_proj[1]) || isinf(dp_proj[0]) || isinf(dp_proj[1]))
						printf("ComputeJacobianRow: projection jacobian is nan or inf\n");
					sub_J[theta_info.jacobian_column] = weight * dp_proj.x;
					sub_J[constraint_index * NUM_PARAMETERS + theta_info.jacobian_column] = weight * dp_proj.y;
				}

				if (verify_jacobian) {
					if (data_energy) {
						jacobian_data[3 * constraint_index * NUM_PARAMETERS + 3 * theta_info.jacobian_column + 0] = dp[0];
						jacobian_data[3 * constraint_index * NUM_PARAMETERS + 3 * theta_info.jacobian_column + 1] = dp[1];
						jacobian_data[3 * constraint_index * NUM_PARAMETERS + 3 * theta_info.jacobian_column + 2] = dp[2];
					}
					else {
						jacobian_silhouette[3 * constraint_index * NUM_PARAMETERS + 3 * theta_info.jacobian_column + 0] = dp[0];
						jacobian_silhouette[3 * constraint_index * NUM_PARAMETERS + 3 * theta_info.jacobian_column + 1] = dp[1];
						jacobian_silhouette[3 * constraint_index * NUM_PARAMETERS + 3 * theta_info.jacobian_column + 2] = dp[2];
					}
				}
			}
		}

		__device__ void jacobian_beta_row(float * sub_J, const int constraint_index, const int shape_unit_id, const glm::vec3 & q, const glm::vec3 & s, glm::ivec3 & index, const glm::vec3 & n, float weight, bool data_energy) {
			//printf("jacobian_beta_row\n");
			if (shape_unit_id == -1) return;

			if (shape_unit_id == 26) {
				for (size_t d = 0; d < 3; d++) {
					if (index[d] == 38 && (index[0] == 33 || index[1] == 33 || index[2] == 33)) index[d] = 24;
					if (index[d] == 38 && (index[0] == 24 || index[1] == 24 || index[2] == 24)) index[d] = 33;
				}
			}


			glm::mat3x2 J_proj;
			if (!data_energy) {
				J_proj = projection_jacobian(q);
			}

			for (int l = 0; l < SHAPE_MAX_LENGTH; l++) {
				int beta_info_id = shape_chains[shape_unit_id].data[l];
				if (beta_info_id == -1) break;
				const BetaInfo& beta_info = beta_infos[beta_info_id];
				glm::vec3& axis = beta_infos[beta_info_id].axis;

				glm::vec3 t(beta_infos[beta_info_id].mat[3][0], beta_infos[beta_info_id].mat[3][1], beta_infos[beta_info_id].mat[3][2]);
				glm::vec4 v4 = glm::vec4(beta_infos[beta_info_id].mat * glm::vec4(axis, 1));
				glm::vec3 v = glm::vec3(v4[0] / v4[3], v4[1] / v4[3], v4[2] / v4[3]) - t;

				glm::vec4 parametrization = glm::vec4(0);				
				if (index[1] == RAND_MAX)
					parametrization[0] = 1.0;
				else if (index[2] == RAND_MAX)
					compute_segment_coordinates(s, get_center(index[0]), get_center(index[1]), parametrization[0], parametrization[1]);
				else
					compute_triangle_coordinates(s, get_center(index[0]), get_center(index[1]), get_center(index[2]), parametrization[0], parametrization[1], parametrization[2]);
				
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
								if (beta_info.attachment_center_id == 31) fraction = 0.40;
								if (beta_info.attachment_center_id == 30) fraction = 0.45;
								if (beta_info.attachment_center_id == 29) fraction = 0.51;
								if (beta_info.attachment_center_id == 28) fraction = 0.58;

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
					compute_triangle_coordinates(s, get_center(index[0]), get_center(index[1]), get_center(index[2]),
						parametrization[0], parametrization[1], parametrization[2]);
					float independent_scaling_factor = 0;
					float dependent_scaling_factor = 0;
					for (size_t d = 0; d < 3; d++) {
						if (index[d] == beta_info.center_id) {
							independent_scaling_factor = parametrization[d];
						}
						if (beta_info.center_id == 20 || beta_info.center_id == 23) { // centers 21 and 22 linearly depend on centers 20 and 23
							if (index[d] == 21 || index[d] == 22) {
								float palm_width = glm::length(get_center(20) - get_center(23));
								float width_ratio = 1 - glm::length(get_center(index[d]) - get_center(beta_info.center_id)) / palm_width;
								dependent_scaling_factor += parametrization[d] * width_ratio;
							}
						}
					}
					dq = (independent_scaling_factor + dependent_scaling_factor) * v;

					if (shape_unit_id == 27) { /// wrist
						glm::dvec4 get_i = glm::dvec4(3); for (size_t d = 0; d < 3; d++) get_i[index[d] - 34] = d;
						if (beta_info.type == 5 /*PALM_CENTER_X*/ && beta_info.top_center_id == 35)
							dq = parametrization[get_i[36 - 34]] * v - parametrization[get_i[35 - 34]] * v - parametrization[get_i[37 - 34]] * v;
						if (beta_info.type == 5 /*PALM_CENTER_X*/ && beta_info.top_center_id == 34) {
							dq = parametrization[get_i[34 - 34]] * v;
						}

						if (beta_info.type == 6 /*PALM_CENTER_Y*/)
							dq = -parametrization[get_i[36 - 34]] * v - parametrization[get_i[37 - 34]] * v;
					}

				}
				if (beta_info.type == 7 /*RADIUS*/) {
					glm::vec3 u = q - s;
					u = u / glm::length(u);
					float scaling_factor = 0;
					for (size_t d = 0; d < 3; d++) {
						if (index[d] == beta_info.center_id)
							scaling_factor = parametrization[d];
					}
					dq = scaling_factor * u;

					if (shape_unit_id == 27) {
						float second_scaling_factor = 0;
						for (size_t d = 0; d < 3; d++) {
							if (index[d] == beta_info.top_center_id) second_scaling_factor = parametrization[d];
						}
						dq = (scaling_factor + second_scaling_factor) * u;
					}
				}

				if (beta_info.type == 8 /*FINGER_TOP_Y*/ || beta_info.type == 9 /*FINGER_TOP_Z*/) {
					float scaling_factor = 0;
					if (index[0] == beta_info.center_id && index[1] == beta_info.top_center_id) scaling_factor = 1 - parametrization[0];
					if (index[1] == beta_info.center_id && index[0] == beta_info.top_center_id) scaling_factor = 1 - parametrization[1];
					if (index[0] == beta_info.center_id && index[1] == RAND_MAX) scaling_factor = 0.0;
					if (index[0] == beta_info.top_center_id && index[1] == RAND_MAX) scaling_factor = 1.0;
					dq = scaling_factor * v;
				}

				if (isnan(dq[0]) || isnan(dq[1]) || isnan(dq[2]) || isinf(dq[0]) || isinf(dq[1]) || isinf(dq[2])) {
					printf("ComputeJacobianRow: point-gradient is nan or inf in jacobian-beta: dq = %f %f %f\n", dq[0], dq[1], dq[2]);
					printf("q = %f %f %f, s = %f %f %f\n", q[0], q[1], q[2], s[0], s[1], s[2]);
					printf("index = %d %d %d\n", index[0], index[1], index[2]);
					printf("parametrization = %f %f %f\n", parametrization[0], parametrization[1], parametrization[2]);
					printf("beta_info.type = %d, scaling_factor = %f\n", beta_info.type, scaling_factor);
				}

				if (data_energy) {
					sub_J[beta_info.jacobian_column] = weight * glm::dot(dq, n);
				}
				else {
					glm::vec2 dq_proj = J_proj * dq;
					sub_J[beta_info.jacobian_column] = weight * dq_proj.x;
					sub_J[constraint_index + NUM_PARAMETERS + beta_info.jacobian_column] = weight * dq_proj.y;
				}

				if (verify_jacobian) {
					if (data_energy) {
						jacobian_data[3 * constraint_index * NUM_PARAMETERS + 3 * beta_info.jacobian_column + 0] = dq[0];
						jacobian_data[3 * constraint_index * NUM_PARAMETERS + 3 * beta_info.jacobian_column + 1] = dq[1];
						jacobian_data[3 * constraint_index * NUM_PARAMETERS + 3 * beta_info.jacobian_column + 2] = dq[2];
					}
					else {
						jacobian_silhouette[3 * constraint_index * NUM_PARAMETERS + 3 * beta_info.jacobian_column + 0] = dq[0];
						jacobian_silhouette[3 * constraint_index * NUM_PARAMETERS + 3 * beta_info.jacobian_column + 1] = dq[1];
						jacobian_silhouette[3 * constraint_index * NUM_PARAMETERS + 3 * beta_info.jacobian_column + 2] = dq[2];
					}
				}
			}
		}

	protected:
		__device__  glm::mat3x2 projection_jacobian(const glm::vec3& pos) {
			glm::mat3x2 M(0);
			M[0][0] = focal_length_x / pos[2];
			M[1][1] = focal_length_y / pos[2];
			M[2][0] = -pos[0] * focal_length_x / (pos[2] * pos[2]);
			M[2][1] = -pos[1] * focal_length_y / (pos[2] * pos[2]);
			return M;
		}
	};
}

