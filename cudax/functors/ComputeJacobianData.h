#pragma once
#include "ComputeJacobianRow.h"

//== NAMESPACE ================================================================
namespace cudax {
	//=============================================================================

	struct ComputeJacobianData : public ComputeJacobianRow {

		float fit3D_weight;
		bool point_to_plane;
		bool reweight;		

		float * radii;
		float * tangent_points;
		float * outline;
		int * blockid_to_pose_unit_id_map;
		int * blockid_to_shape_unit_id_map;

		int * sensor_indicator;

		bool verify_jacobian;
		float * correspondences_data_points;
		float * correspondences_model_points;
		int * correspondences_model_indices;
		int * correspondences_block_indices;

		bool compute_weighted_metric;
		float * weighted_metric_sensor_palm;
		float * weighted_metric_sensor_fingers;

		bool undistort;

	public:
		ComputeJacobianData(float* J_raw, float* e_raw, bool reweight, bool verify_jacobian, bool compute_weighted_metric) :
			ComputeJacobianRow(J_raw, e_raw, verify_jacobian) {
			point_to_plane = settings->fit3D_point2plane;
			fit3D_weight = settings->fit3D_weight;
			this->reweight = reweight;
			this->verify_jacobian = verify_jacobian;
			this->compute_weighted_metric = compute_weighted_metric;

			this->sensor_indicator = thrust::raw_pointer_cast(_sensor_indicator->data());

			this->radii = thrust::raw_pointer_cast(device_pointer_radii->data());
			this->tangent_points = thrust::raw_pointer_cast(device_pointer_tangent_points->data());
			this->outline = thrust::raw_pointer_cast(device_pointer_outline->data());
			this->blockid_to_pose_unit_id_map = thrust::raw_pointer_cast(device_pointer_blockid_to_pose_unit_id_map->data());
			if (host_calibrate)
				this->blockid_to_shape_unit_id_map = thrust::raw_pointer_cast(device_pointer_blockid_to_shape_unit_id_map->data());

			if (verify_jacobian) {
				this->correspondences_data_points = thrust::raw_pointer_cast(_correspondences_data_points->data());
				this->correspondences_model_points = thrust::raw_pointer_cast(_correspondences_model_points->data());
				this->correspondences_model_indices = thrust::raw_pointer_cast(_correspondences_model_indices->data());		
				this->correspondences_block_indices = thrust::raw_pointer_cast(_correspondences_block_indices->data());
			}
			if (compute_weighted_metric) {
				this->weighted_metric_sensor_palm = thrust::raw_pointer_cast(_weighted_metric_sensor_palm->data());
				this->weighted_metric_sensor_fingers = thrust::raw_pointer_cast(_weighted_metric_sensor_fingers->data());
			}

			undistort = settings->undistort;
		}	

		__device__ void store_correspondences(int constraint_index, const glm::vec3 & p, const glm::vec3 & q, int b, const glm::ivec3 & index) {

			correspondences_data_points[3 * constraint_index + 0] = p.x;
			correspondences_data_points[3 * constraint_index + 1] = p.y;
			correspondences_data_points[3 * constraint_index + 2] = p.z;

			correspondences_model_points[3 * constraint_index + 0] = q.x;
			correspondences_model_points[3 * constraint_index + 1] = q.y;
			correspondences_model_points[3 * constraint_index + 2] = q.z;

			correspondences_model_indices[3 * constraint_index + 0] = index.x;
			correspondences_model_indices[3 * constraint_index + 1] = index.y;
			correspondences_model_indices[3 * constraint_index + 2] = index.z;

			correspondences_block_indices[constraint_index] = b;
		}

		__device__ void assemble_linear_system(int constraint_index, const glm::vec3 & p, bool wrist_point) {

			glm::vec3 q, s; glm::ivec3 index; int b;
			correspondences_finder.find(p, b, q, s, index, constraint_index, wrist_point);

			if (verify_jacobian) {
				glm::vec3 p_new = p; glm::vec3 q_new = q;
				if (q_new == glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX)) {
					p_new = glm::vec3(0, 0, 0);
					q_new = glm::vec3(0, 0, 0);
				}
				store_correspondences(constraint_index, p_new, q_new, b, index);
			}

			if (q == glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX)) {
				return;
			}

			if (length(p - q) < 1e-5) return;
			glm::vec3 n = (p - q) / length(p - q);

			if (isnan(n[0]) || isnan(n[1]) || isnan(n[2])) return;
			if (isnan(p[0]) || isnan(p[1]) || isnan(p[2])) {
				printf("ComputeJacobianData: data-point is nan\n");
			}
			if (p[0] == 0 && p[1] == 0 && p[2] == 0) {
				return;
				printf("ComputeJacobianData: data-point is zeros\n");
			}
			if (isnan(q[0]) || isnan(q[1]) || isnan(q[2])) {
				return;
				printf("ComputeJacobianData: model-point is nan\n");
			}

			int pose_unit_id = blockid_to_pose_unit_id_map[b];

			float weight = fit3D_weight;
			if (reweight) weight = reweight_function(p, q, weight);

			float* J_sub = J_raw + constraint_index * NUM_PARAMETERS;
			float* e_sub = e_raw + constraint_index;
			
			*e_sub = weight * glm::dot(p - q, n);	
			if (isnan(*e_sub) || isinf(*e_sub)) printf("ComputeJacobianData: rhs is nan or inf\n");
			jacobian_theta_row(J_sub, constraint_index, pose_unit_id, q, n, weight, true /*data_energy*/);

			if (device_calibrate) {			
				int shape_unit_id = blockid_to_shape_unit_id_map[b];
				if (shape_unit_id >= 0) {
					glm::ivec3 full_index = glm::ivec3(blocks[D * b], blocks[D * b + 1], blocks[D * b + 2]);
					jacobian_beta_row(J_sub, constraint_index, shape_unit_id, q, s, full_index, n, weight, true /*data_energy*/);
				}				
			}

			if (compute_weighted_metric) {
				if (b >= 14 && b <= 26 || b == 28 || b == 29) { /// palm
					weighted_metric_sensor_palm[constraint_index] = glm::dot(p - q, n);
				}
				else { /// fingers
					weighted_metric_sensor_fingers[constraint_index] = glm::dot(p - q, n);
				}
			}
		}

		__device__ void operator()(int index) {

			bool wrist_point = false;
			if (sensor_indicator[index] < 0) wrist_point = true;
			int linear_index = abs(sensor_indicator[index]);

			int offset_y = linear_index / width;
			int offset_x = linear_index - width * offset_y;
			offset_y = height - 1 - offset_y;

			float depth = (float)tex2D(sensor_depth_texture_cuda, offset_x, height - 1 - offset_y).x;
			
			//glm::vec3 data_point = inversed_projection_matrix * glm::vec3(offset_x * depth, offset_y * depth, depth);
			glm::vec3 data_point = inversed_projection_matrix * glm::vec3(offset_x, offset_y, 1.0f);
			if (undistort) data_point = undistort_function(data_point);
			data_point = depth * data_point;
			
			int constraint_index = index;
			assemble_linear_system(constraint_index, data_point, wrist_point);
		}

	};
}