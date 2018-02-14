#pragma once
#include "ComputeJacobianRow.h"

namespace cudax {

	class ComputeJacobianSilhouette : public ComputeJacobianRow {

		const float epsilon = 0.00001;

		__device__ glm::vec3 ray_sphere_intersection(glm::vec3 c, float r, glm::vec3 p, glm::vec3 v, glm::vec3 & normal) {
			float A = dot(v, v);
			float B = -2 * dot(c - p, v);
			float C = dot(c - p, c - p) - r*r;
			float D = B*B - 4 * A*C;
			float t1 = RAND_MAX;
			float t2 = RAND_MAX;
			glm::vec3 i1, i2;
			if (D >= 0) {
				t1 = (-B - sqrt(D)) / 2 / A;
				t2 = (-B + sqrt(D)) / 2 / A;
				i1 = p + t1 * v;
				i2 = p + t2 * v;
			}
			glm::vec3 i = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			if (abs(t1) < abs(t2)) {
				i = i1;
			}
			if (abs(t1) > abs(t2)) {
				i = i2;
			}
			normal = (i - c) / glm::length(i - c);

			/*printf("p = [%f; %f; %f]; c = [%f; %f; %f]; v = [%f; %f; %f]; r = %f; i1 = [%f; %f; %f]; i2 = [%f; %f; %f]; t1 = %f; t2 = %f; D = %f; i = [%f; %f; %f];\n",
				p[0], p[1], p[2],
				c[0], c[1], c[2],
				v[0], v[1], v[2],
				r,
				i1[0], i1[1], i1[2],
				i2[0], i2[1], i2[2],
				t1, 
				t2,
				D,
				i[0], i[1], i[2]);*/

			return i;
		}

		__device__ glm::vec3 ray_cylinder_intersection(glm::vec3 pa, glm::vec3 va, float r, glm::vec3 p, glm::vec3 v, glm::vec3 & normal) {
			glm::vec3 i = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			glm::vec3 delta_p = p - pa;
			glm::vec3 e = v - dot(v, va) * va;
			float f = dot(v, va);
			glm::vec3 g = delta_p - dot(delta_p, va) * va;
			float h = dot(delta_p, va);
			float A = dot(e, e);
			float B = 2 * dot(e, g);
			float C = dot(g, g) - r * r;
			float D = B*B - 4 * A*C;
			float t1 = RAND_MAX;
			float t2 = RAND_MAX;
			glm::vec3 i1 = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			glm::vec3 i2 = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			if (D >= 0) {
				t1 = (-B - sqrt(D)) / 2 / A;
				t2 = (-B + sqrt(D)) / 2 / A;
				i1 = p + t1 * v;
				i2 = p + t2 * v;
				if (dot(va, i1 - pa) > 0) t1 = RAND_MAX;
				if (dot(va, i2 - pa) > 0) t2 = RAND_MAX;
			}

			if (length(p - i1) < length(p - i2)) {
				i = i1;
			}

			if (length(p - i2) < length(p - i1)) {
				i = i2;
			}
			// Find normal
			glm::vec3 c = pa + dot(i - pa, va) * va;
			normal = (i - c) / glm::length(i - c);
			return i;
		}

		__device__ glm::vec3 ray_cone_intersection(glm::vec3 pa, glm::vec3 va, float alpha, glm::vec3 p, glm::vec3 v, glm::vec3 & normal) {
			glm::vec3 i = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			float cos2 = cos(alpha) * cos(alpha);
			float sin2 = sin(alpha) * sin(alpha);
			glm::vec3 delta_p = p - pa;
			glm::vec3 e = v - dot(v, va) * va;
			float f = dot(v, va);
			glm::vec3 g = delta_p - dot(delta_p, va) * va;
			float h = dot(delta_p, va);
			float A = cos2 * dot(e, e) - sin2 * f * f;
			float B = 2 * cos2 * dot(e, g) - 2 * sin2 * f * h;
			float C = cos2 * dot(g, g) - sin2 * h * h;
			float D = B*B - 4 * A*C;
			float t1 = RAND_MAX;
			float t2 = RAND_MAX;
			glm::vec3 i1 = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			glm::vec3 i2 = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			if (D >= 0) {
				t1 = (-B - sqrt(D)) / 2 / A;
				t2 = (-B + sqrt(D)) / 2 / A;
				i1 = p + t1 * v;
				i2 = p + t2 * v;
				if (dot(va, i1 - pa) > 0) t1 = RAND_MAX;
				if (dot(va, i2 - pa) > 0) t2 = RAND_MAX;
			}

			if (length(p - i1) < length(p - i2)) {
				i = i1;
			}

			if (length(p - i2) < length(p - i1)) {
				i = i2;
			}

			// Find normal - fix this
			float l = length(pa - i);
			glm::vec3 c = pa - l * va;
			normal = (i - c) / glm::length(i - c);

			return i;
		}

		__device__ glm::vec3 ray_triangle_intersection(glm::vec3 p0, glm::vec3 p1, glm::vec3 p2, glm::vec3 o, glm::vec3 d, glm::vec3 & normal) {
			glm::vec3 i = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			glm::vec3 e1 = p1 - p0;
			glm::vec3 e2 = p2 - p0;
			glm::vec3 q = cross(d, e2);
			float a = dot(e1, q);

			// the vector is parallel to the plane(the intersection is at infinity)
			if (a > -epsilon && a < epsilon) {
				return i;
			}
			float f = 1 / a;
			glm::vec3 s = o - p0;
			float u = f * dot(s, q);

			// the intersection is outside of the triangle
			if (u < 0.0) {
				return i;
			}
			glm::vec3 r = cross(s, e1);
			float v = f * dot(d, r);

			// the intersection is outside of the triangle
			if (v <0.0 || u + v > 1.0) {
				return i;
			}
			float t = f * dot(e2, r);
			i = o + t * d;
			normal = normalize(cross(e1, e2));
			return i;
		}

		__device__ glm::vec3 ray_convsegment_intersection(glm::vec3 c1, glm::vec3 c2, float r1, float r2, glm::vec3 p, glm::vec3 v, glm::vec3 & normal) {

			// Ray - sphere intersection
			glm::vec3 n1 = glm::vec3(0, 0, 0);
			glm::vec3 i1 = ray_sphere_intersection(c1, r1, p, v, n1);		

			// Ray - sphere intersection
			glm::vec3 n2 = glm::vec3(0, 0, 0);
			glm::vec3 i2 = ray_sphere_intersection(c2, r2, p, v, n2);

			// Ray - cone intersections
			glm::vec3 n = (c2 - c1) / length(c2 - c1);
			glm::vec3 i = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);

			glm::vec3 n12 = glm::vec3(0, 0, 0);
			glm::vec3 i12, s1, s2;
			if (r1 - r2 < epsilon) {
				i12 = ray_cylinder_intersection(c2, n, r1, p, v, n12);
				s1 = c1;
				s2 = c2;
			}
			else {
				float beta = asin((r1 - r2) / length(c1 - c2));
				float eta1 = r1 * sin(beta);
				s1 = c1 + eta1 * n;
				float eta2 = r2 * sin(beta);
				s2 = c2 + eta2 * n;
				glm::vec3 z = c1 + (c2 - c1) * r1 / (r1 - r2);
				float r = r1 * cos(beta);
				float h = length(z - s1);
				float alpha = atan(r / h);
				i12 = ray_cone_intersection(z, n, alpha, p, v, n12);
			}

			if (dot(n, i12 - s1) >= 0 && dot(n, i12 - s2) <= 0 && length(i12) < RAND_MAX) {
				i = i12;
				normal = n12;
			}
			if (dot(n, i1 - s1) < 0 && length(i1) < RAND_MAX) {
				i = i1;
				normal = n1;
			}
			if (dot(n, i2 - s2) > 0 && length(i2) < RAND_MAX) {
				i = i2;
				normal = n2;
			}

			/*printf("p = [%f; %f; %f]; c1= [%f; %f; %f]; c2 = [%f; %f; %f]; r1 = %f; r2 = %f; i1 = [%f; %f; %f]; i2 = [%f; %f; %f]; i12 = [%f; %f; %f]; s1 = [%f; %f; %f]; s2 = [%f; %f; %f]; i = [%f; %f; %f]; n = [%f; %f; %f];\n",
				p[0], p[1], p[2],
				c1[0], c1[1], c1[2],
				c2[0], c2[1], c2[2],
				r1, r2,
				i1[0], i1[1], i1[2], 
				i2[0], i2[1], i2[2], 
				i12[0], i12[1], i12[2], 
				s1[0], s1[1], s1[2],
				s2[0], s2[1], s2[2],
				i[0], i[1], i[2], 
				n[0], n[1], n[2]);*/

			return i;
		}

		__device__ glm::vec3 ray_convtriangle_intersection(glm::vec3 c1, glm::vec3 c2, glm::vec3 c3, glm::vec3 v1, glm::vec3 v2, glm::vec3 v3,
			glm::vec3 u1, glm::vec3 u2, glm::vec3 u3, float r1, float r2, float r3, glm::vec3 p, glm::vec3 v, glm::vec3 & normal) {

			glm::vec3 n1 = glm::vec3(0, 0, 0);
			glm::vec3 n2 = glm::vec3(0, 0, 0);
			glm::vec3 n3 = glm::vec3(0, 0, 0);
			glm::vec3 n4 = glm::vec3(0, 0, 0);
			glm::vec3 n5 = glm::vec3(0, 0, 0);
			glm::vec3 i1 = ray_convsegment_intersection(c1, c2, r1, r2, p, v, n1);
			glm::vec3 i2 = ray_convsegment_intersection(c1, c3, r1, r3, p, v, n2);
			glm::vec3 i3 = ray_convsegment_intersection(c2, c3, r2, r3, p, v, n3);
			glm::vec3 i4 = ray_triangle_intersection(v1, v2, v3, p, v, n4);
			glm::vec3 i5 = ray_triangle_intersection(u1, u2, u3, p, v, n5);

			float min_value = distance(p, i1);
			glm::vec3 i = i1;
			normal = n1;
			if (distance(p, i2) < min_value) {
				min_value = distance(p, i2);
				i = i2;
				normal = n2;
			}
			if (distance(p, i3) < min_value) {
				min_value = distance(p, i3);
				i = i3;
				normal = n3;
			}
			if (distance(p, i4) < min_value) {
				min_value = distance(p, i4);
				i = i4;
				normal = n4;
			}
			if (distance(p, i5) < min_value) {
				min_value = distance(p, i5);
				i = i5;
				normal = n5;
			}
			if (dot(normal, i - c1) < 0) normal = -normal;

			return i;
		}
		
		__device__ glm::vec3 ray_model_intersection(glm::vec3 p, glm::vec3 d, int b, glm::vec3 & min_normal, int & min_b) {
			glm::vec3 i; glm::vec3 normal = glm::vec3(0, 0, 0);
			glm::vec3 min_i = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
			float min_distance = RAND_MAX;
			glm::vec3 c1, c2, c3, v1, v2, v3, u1, u2, u3;
			float r1, r2, r3;
			for (int j = 0; j < NUM_BLOCKS; j++) {
			//int j = b;

				glm::ivec3 block = glm::ivec3(blocks[D * j], blocks[D * j + 1], blocks[D * j + 2]);
				if (block[2] < RAND_MAX) {
					c1 = get_center(block[0]); c2 = get_center(block[1]); c3 = get_center(block[2]);
					r1 = radii[block[0]]; r2 = radii[block[1]]; r3 = radii[block[2]];
	
					v1 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j], tangent_points[NUM_TANGENT_FIELDS * D * j + 1], tangent_points[NUM_TANGENT_FIELDS * D * j + 2]);
					v2 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 3], tangent_points[NUM_TANGENT_FIELDS * D * j + 4], tangent_points[NUM_TANGENT_FIELDS * D * j + 5]);
					v3 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 6], tangent_points[NUM_TANGENT_FIELDS * D * j + 7], tangent_points[NUM_TANGENT_FIELDS * D * j + 8]);					
					u1 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 12], tangent_points[NUM_TANGENT_FIELDS * D * j + 13], tangent_points[NUM_TANGENT_FIELDS * D * j + 14]);
					u2 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 15], tangent_points[NUM_TANGENT_FIELDS * D * j + 16], tangent_points[NUM_TANGENT_FIELDS * D * j + 17]);
					u3 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 18], tangent_points[NUM_TANGENT_FIELDS * D * j + 19], tangent_points[NUM_TANGENT_FIELDS * D * j + 20]);
					
					i = ray_convtriangle_intersection(c1, c2, c3, v1, v2, v3, u1, u2, u3, r1, r2, r3, p, d, normal);
					if (length(p - i) < min_distance) {
						min_distance = length(p - i);
						min_i = i;
						min_normal = normal;
						min_b = j;
					}
				}
				if (block[2] >= RAND_MAX) {
					c1 = get_center(block[0]); c2 = get_center(block[1]);
					r1 = radii[block[0]] + 0.01; r2 = radii[block[1]] + 0.01;

					i = ray_convsegment_intersection(c1, c2, r1, r2, p, d, normal);
					//printf("c1 = %f %f %f, c2 = %f %f %f, r1 = %f, r2 = %f, p = %f %f %f, i = %f, %f, %f\n", c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], r1, r2, p[0], p[1], p[2], i[0], i[1], i[2]);
					if (length(p - i) < min_distance) {
						min_distance = length(p - i);
						min_i = i;
						min_normal = normal;
						min_b = j;
					}
				}
			}
			return min_i;
		}

		int * sensor_silhouette_distance_transform;
		int * sensor_silhouette_wrist_distance_transform;
		int * sensor_outline_distance_transform;			

		int * blockid_to_pose_unit_id_map;
		int * blockid_to_shape_unit_id_map;

		int * rendered_pixels;
		float * rendered_points;
		int * rendered_block_ids;

		bool fit2D_outline_enable;
		bool fit2D_silhouette2outline_enable;
		bool fit2D_silhouette_enable;
		bool fit2D_unproject;

		float fit2D_weight;
		float fit2D_weight_palm;
		float fit2D_weight_segment;

		float * centers;
		float * radii;
		int * blocks;
		float * tangent_points;	

		bool verify_jacobian;
		float * correspondences_silhouette_data_points;
		float * correspondences_silhouette_model_points;
		int * correspondences_silhouette_indices;

		bool compute_weighted_metric;
		float * weighted_metric_rendered_palm;
		float * weighted_metric_rendered_fingers;
		bool fit_wrist_separately;

		bool undistort;
		int dataset_type;

		/*__device__ int find_closest_index(int index) {
			int min_index = 0;
			float min_distance = RAND_MAX;

			int row = index / width;
			int col = index - width * row;
			glm::vec3 p = 300.0f * inversed_projection_matrix * glm::vec3(col, row, 1.0f);

			int count = 0;
			for (size_t i = 0; i < height * width; i++)	{
				if (sensor_outline[i] == 255) {
					count++;
					int current_row = i / width;
					int current_col = i - width * current_row;

					glm::vec3 p_current = 300.0f * inversed_projection_matrix * glm::vec3(current_col, current_row, 1.0f);
					float distance = glm::length(p - p_current);

					if (distance < min_distance) {
						min_distance = distance;
						min_index = i;
					}
				}
			}

			return min_index;
		}*/

		__device__ bool check_if_visible(glm::vec3 model_point, int block_id) {			
			glm::vec3 ray_direction = glm::vec3(0, 0, 1);
			glm::vec3 normal = glm::vec3(0, 0, 0);
			glm::vec3 camera_center = glm::vec3(model_point[0], model_point[1], 0);
			int out_color = 255;
			glm::vec3 i = ray_model_intersection(camera_center, ray_direction, block_id, normal, out_color);
			if (out_color == block_id) return true;
			else return false;
		}

		__device__ glm::vec3 project_point_on_line(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 b) {
			glm::vec3  u = b - a;
			glm::vec3 v = p - a;
			float alpha = glm::dot(u, v) / glm::dot(u, u);
			glm::vec3 q = a + alpha * u;
			return q;
		}

	public:
		ComputeJacobianSilhouette(float * J_raw, float* e_raw, bool verify_jacobian, bool compute_weighted_metric) : ComputeJacobianRow(J_raw, e_raw, verify_jacobian) {

			this->blockid_to_pose_unit_id_map = thrust::raw_pointer_cast(device_pointer_blockid_to_pose_unit_id_map->data());
			if (host_calibrate)
				this->blockid_to_shape_unit_id_map = thrust::raw_pointer_cast(device_pointer_blockid_to_shape_unit_id_map->data());

			fit2D_unproject = settings->fit2D_unproject;
			fit2D_outline_enable = settings->fit2D_outline_enable;
			fit2D_silhouette2outline_enable = settings->fit2D_silhouette2outline_enable;
			fit2D_silhouette_enable = settings->fit2D_silhouette_enable;
			fit2D_weight = settings->fit2D_weight;
			fit2D_weight_palm = settings->fit2D_weight_palm;
			fit2D_weight_segment = settings->fit2D_weight_segment;

			this->verify_jacobian = verify_jacobian;
			this->compute_weighted_metric = compute_weighted_metric;

			fit_wrist_separately = settings->fit_wrist_separately;
			this->sensor_silhouette_distance_transform = thrust::raw_pointer_cast(_sensor_silhouette_distance_transform->data());
			if (fit2D_outline_enable) {
				this->sensor_outline_distance_transform = thrust::raw_pointer_cast(_sensor_outline_distance_transform->data());
				if (fit_wrist_separately)
					this->sensor_silhouette_wrist_distance_transform = thrust::raw_pointer_cast(_sensor_silhouette_wrist_distance_transform->data());
			}

			this->rendered_pixels = thrust::raw_pointer_cast(_rendered_pixels->data());
			if (fit2D_silhouette2outline_enable || fit2D_outline_enable) {
				this->rendered_points = thrust::raw_pointer_cast(_rendered_points->data());
				this->rendered_block_ids = thrust::raw_pointer_cast(_rendered_block_ids->data());
				if (fit2D_outline_enable) {					

					this->centers = thrust::raw_pointer_cast(device_pointer_centers->data());
					this->radii = thrust::raw_pointer_cast(device_pointer_radii->data());
					this->blocks = thrust::raw_pointer_cast(device_pointer_blocks->data());
					this->tangent_points = thrust::raw_pointer_cast(device_pointer_tangent_points->data());
				}
			}

			if (verify_jacobian) {
				this->correspondences_silhouette_data_points = thrust::raw_pointer_cast(_correspondences_silhouette_data_points->data());
				this->correspondences_silhouette_model_points = thrust::raw_pointer_cast(_correspondences_silhouette_model_points->data());
				this->correspondences_silhouette_indices = thrust::raw_pointer_cast(_correspondences_silhouette_indices->data());
			}

			if (compute_weighted_metric) {
				this->weighted_metric_rendered_palm = thrust::raw_pointer_cast(_weighted_metric_rendered_palm->data());
				this->weighted_metric_rendered_fingers = thrust::raw_pointer_cast(_weighted_metric_rendered_fingers->data());
			}

			undistort = settings->undistort;
			dataset_type = settings->dataset_type;			
		}

		__device__ void assemble_linear_system(int constraint_index, int block_id, glm::vec2 p_diff, glm::vec3 model_point, glm::vec3 data_point, bool outside_of_sensor_silhouette, bool is_segment_point, bool reject_correspondence, bool wrist_point) {
	
			{
				if (isnan(p_diff[0]) || isnan(p_diff[1])) {
					printf("ComputeJacobianSilhouette: p_diff is nan\n");
				}
				if (isnan(model_point[0]) || isnan(model_point[1]) || isnan(model_point[2]) || isinf(model_point[0]) || isinf(model_point[1]) || isinf(model_point[2])) {
					printf("ComputeJacobianSilhouette: model_point is nan or isinf\n");
				}
				if (isnan(data_point[0]) || isnan(data_point[1]) || isnan(data_point[2]) || isinf(data_point[0]) || isinf(data_point[1]) || isinf(data_point[2])) {
					printf("ComputeJacobianSilhouette: data_point is nan or isinf\n");
				}
				if (abs(model_point[2]) < 1e-7) {
					printf("ComputeJacobianSilhouette: model_point.z = 0 -> projection jacobian will fail, constraint_index = %d\n", constraint_index);
				}
				if (constraint_index < 0 || constraint_index > 10000) {
					printf("ComputeJacobianSilhouette: constraint_index is negative or > 10000\n");
				}
			}
			glm::vec3 q, s; glm::ivec3 index; int b;
			correspondences_finder.find(model_point, b, q, s, index, constraint_index, wrist_point);
			int pose_unit_id = blockid_to_pose_unit_id_map[block_id];

			float z_threshold_difference = 7.0;
			float length_threshold_difference = 10.0;
			if (fit2D_outline_enable && !outside_of_sensor_silhouette && !reject_correspondence && pose_unit_id != 0) {
				if (glm::length(data_point - model_point) > length_threshold_difference ||
					glm::abs(data_point[2] - model_point[2]) > z_threshold_difference) {
					reject_correspondence = true;
				}
			}

			float weight = fit2D_weight;
			if (fit2D_outline_enable && is_segment_point && device_calibrate) weight = fit2D_weight_segment;
			if (fit2D_outline_enable && pose_unit_id == 0 && device_calibrate) weight = fit2D_weight_palm;

			float * J_sub; float * e_sub; glm::vec3 n;
			if (fit2D_unproject && !reject_correspondence) {
				weight = reweight_function(data_point, model_point, weight);
				J_sub = J_raw + constraint_index * NUM_PARAMETERS;
				e_sub = e_raw + constraint_index;

				glm::vec3 camera_center = glm::vec3(0, 0, 0);
				data_point = project_point_on_line(model_point, data_point, camera_center);

				n = (data_point - model_point) / glm::length(data_point - model_point);
				if (glm::length(data_point - model_point) < 1e-6) {
					printf("to small length of normal, data_point = (%f, %f, %f), model_point = (%f, %f, %f)\n)",
						data_point[0], data_point[1], data_point[2], model_point[0], model_point[1], model_point[2]);
					return;
				}

				*e_sub = weight * glm::dot(data_point - model_point, n);
				if (isnan(*e_sub) || isinf(*e_sub)) printf("ComputeJacobianSilhouette: rhs is nan or inf\n");

				jacobian_theta_row(J_sub, constraint_index, pose_unit_id, model_point, n, weight, true);
				if (device_calibrate) {
					int shape_unit_id = blockid_to_shape_unit_id_map[block_id];
					if (shape_unit_id >= 0) {
						glm::ivec3 full_index = glm::ivec3(blocks[D * b], blocks[D * b + 1], blocks[D * b + 2]);
						jacobian_beta_row(J_sub, constraint_index, shape_unit_id, model_point, s, full_index, n, weight, true);
					}
				}
			}

			if (!fit2D_unproject && !reject_correspondence) {
				J_sub = J_raw + 2 * constraint_index * NUM_PARAMETERS;
				e_sub = e_raw + 2 * constraint_index;
				*(e_sub + 0) = weight * p_diff.x;
				*(e_sub + 1) = weight * p_diff.y;

				jacobian_theta_row(J_sub, constraint_index, pose_unit_id, model_point, n, weight, false);
				if (device_calibrate) {
					int shape_unit_id = blockid_to_shape_unit_id_map[block_id];
					if (shape_unit_id >= 0) {
						jacobian_beta_row(J_sub, constraint_index, shape_unit_id, model_point, s, index, n, weight, false);
					}
				}
			}

			if (verify_jacobian) {
				if (reject_correspondence || weight == 0) {
					data_point = glm::vec3(0, 0, 0);
					model_point = glm::vec3(0, 0, 0);
				}

				correspondences_silhouette_data_points[3 * constraint_index + 0] = data_point.x;
				correspondences_silhouette_data_points[3 * constraint_index + 1] = data_point.y;
				correspondences_silhouette_data_points[3 * constraint_index + 2] = data_point.z;

				correspondences_silhouette_model_points[3 * constraint_index + 0] = model_point.x;
				correspondences_silhouette_model_points[3 * constraint_index + 1] = model_point.y;
				correspondences_silhouette_model_points[3 * constraint_index + 2] = model_point.z;

				correspondences_silhouette_indices[3 * constraint_index + 0] = index.x;
				correspondences_silhouette_indices[3 * constraint_index + 1] = index.y;
				correspondences_silhouette_indices[3 * constraint_index + 2] = index.z;
			}

			if (compute_weighted_metric) {
				if (block_id >= 14 && block_id <= 26 || block_id == 28 || block_id == 29) { /// palm
					weighted_metric_rendered_palm[constraint_index] = glm::dot(data_point - model_point, n);
				}
				else { /// fingers
					weighted_metric_rendered_fingers[constraint_index] = glm::dot(data_point - model_point, n);
				}
			}

		}

	public:

		__device__ void operator()(int index) {

			bool outside_of_sensor_silhouette = false;
			bool is_segment_point = false;
			bool reject_correspondence = false;
			bool wrist_point = false;

			glm::vec3 model_point; int block_id;
			if (fit2D_silhouette2outline_enable || fit2D_outline_enable) {
				model_point[0] = rendered_points[3 * index];
				model_point[1] = rendered_points[3 * index + 1];
				model_point[2] = rendered_points[3 * index + 2];
				block_id = rendered_block_ids[index];
			}

			int linear_index = rendered_pixels[index];
			int constraint_index = index;

			int offset_y = linear_index / width;
			int offset_x = linear_index - width * offset_y;
			offset_y = height - 1 - offset_y;
			int offset_z = offset_y * width + offset_x;

			// Fetch closest point on sensor data
			int closest_index;
			if (fit2D_silhouette2outline_enable || fit2D_silhouette_enable) {
				closest_index = sensor_silhouette_distance_transform[offset_z];
			}
			if (fit2D_outline_enable) {
				if (fit_wrist_separately && (block_id == 28 || block_id == 29 || block_id == -29 || block_id == -30)) {
					closest_index = sensor_silhouette_wrist_distance_transform[offset_z];					
					if (offset_z == sensor_silhouette_distance_transform[offset_z] ||
						offset_z == sensor_silhouette_wrist_distance_transform[offset_z] ||
						closest_index == 76799 /*320 * 240 - 1*/) {
						reject_correspondence = true;
					}
					wrist_point = true;
				}
				else {
					closest_index = sensor_outline_distance_transform[offset_z];
				}
			}
			int closest_row = closest_index / width;
			int closest_col = closest_index - width * closest_row;

			glm::vec2 p_rend(offset_x, offset_y);
			glm::vec2 p_sens(closest_col, closest_row);
			glm::vec2 p_diff = p_sens - p_rend;


			if (isnan(p_rend[0]) || isnan(p_rend[1])) {
				printf("ComputeJacobianSilhouette: rendered-point is nan\n");
			}
			if (isnan(p_sens[0]) || isnan(p_sens[1])) {
				printf("ComputeJacobianSilhouette: sensor-point is nan\n");
			}

			if (fit2D_silhouette_enable) {
				block_id = (int)tex2D(rendered_block_indices_texture_cuda, offset_x, offset_y);
				float depth = (float)tex2D(rendered_depth_texture_cuda, offset_x, offset_y).x;
				model_point = inversed_projection_matrix * glm::vec3(offset_x, offset_y, 1.0f);
				if (undistort) model_point = undistort_function(model_point);
				model_point = depth * model_point;
			}

			glm::vec3 data_point = glm::vec3(0, 0, 0);
			if (fit2D_unproject || verify_jacobian) {
				float depth = (float)tex2D(sensor_depth_texture_cuda, closest_col, height - 1 - closest_row).x;

				if (fit2D_outline_enable && depth == 0) { // this is a hole in sensor silhouette that was filled in
					depth = model_point[2];
				}
				if (isnan(depth) || isinf(depth) || depth == 0) {
					if (index % 500 == 0) printf("ComputeJacobianSilhouette: depth is nan, inf or 0\n");
					return;
				}
				data_point = inversed_projection_matrix * glm::vec3(closest_col, closest_row, 1.0f);
				if (undistort) data_point = undistort_function(data_point);
				data_point = depth * data_point;
			}
	
			if (offset_z != sensor_silhouette_distance_transform[offset_z]) outside_of_sensor_silhouette = true;

			/// > Use segment point only if it is outside of senser silhouette, otherwise return
			if (fit2D_outline_enable && block_id < 0 ) {
				is_segment_point = true;
				if (!outside_of_sensor_silhouette) reject_correspondence = true;
				block_id = -block_id;
				block_id = block_id - 1;

				int pose_unit_id = blockid_to_pose_unit_id_map[block_id];
				if (pose_unit_id == 0 && (block_id != 19 && block_id != 25 && block_id != 26)) reject_correspondence = false; // attract palm outline to data outline even if it is inside sensor silhouette
																															 // reject correspondences between index and palm, if the inside sensor silhoette, the are occluded
			}	

			if (outside_of_sensor_silhouette == false && reject_correspondence == false) {
				bool is_visible = check_if_visible(model_point, block_id);
				if (is_visible == false) reject_correspondence = true;
			}
			assemble_linear_system(constraint_index, block_id, p_diff, model_point, data_point, outside_of_sensor_silhouette, is_segment_point, reject_correspondence, wrist_point);
		}
	};


}
