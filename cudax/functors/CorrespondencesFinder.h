#include "cudax/kernel.h"
#include "cudax/cuda_glm.h"

#define M_PI 3.14159265358979;

struct CorrespondencesFinder {
	float * centers;
	float * radii;
	int * blocks;
	float * tangent_points;
	float * outline;
	bool disable_finger_bases_fitting;
	bool fit_wrist_separately;

	ThetaInfo* theta_infos;

	CorrespondencesFinder() {
		this->centers = thrust::raw_pointer_cast(device_pointer_centers->data());
		this->radii = thrust::raw_pointer_cast(device_pointer_radii->data());
		this->blocks = thrust::raw_pointer_cast(device_pointer_blocks->data());
		this->tangent_points = thrust::raw_pointer_cast(device_pointer_tangent_points->data());
		this->outline = thrust::raw_pointer_cast(device_pointer_outline->data());

		this->theta_infos = thrust::raw_pointer_cast(&device_pointer_theta_infos[0]);
		disable_finger_bases_fitting = settings->disable_finger_bases_fitting;
		fit_wrist_separately = settings->fit_wrist_separately;
	}

	__device__ void print_vec3(glm::vec3 v, char * name) {
		printf("%s: ", name); printf("%f, %f, %f\n", v[0], v[1], v[2]);
	}
	
	__device__ void print_ivec3(glm::ivec3 v, char * name) {
		printf("%s: ", name); printf("%d, %d, %d\n", v[0], v[1], v[2]);
	}
	
	__device__ void print_float(float a, char * name) {
		printf("%s: ", name); printf("%f\n", a);
	}
	
	__device__ void print_int(int a, char * name) {
		printf("%s: ", name); printf("%d\n", a);
	}

	__device__ float myatan2(glm::vec2 v) {
		float alpha = atan2(v[1], v[0]);
		if (alpha < 0) alpha = alpha + 2 * M_PI;
		return alpha;
	}
	
	__device__ float sign(float a) {
		if (a >= 0) return 1.0;
		else return -1.0;
	}

	__device__ int index_size(glm::ivec3 index) {
		if (index[1] > NUM_CENTERS) return 1;
		if (index[2] > NUM_CENTERS) return 2;
		return 3;
	}

	__device__ bool is_point_on_segment(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b) {
		float alpha = dot(b - a, p - a);
		if (alpha < 0) return false;
		if (alpha > dot(b - a, b - a)) return false;
		return true;
	}

	__device__ bool is_point_in_triangle(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b, const glm::vec3 & c) {
		glm::vec3 v0 = b - a;
		glm::vec3 v1 = c - a;
		glm::vec3 v2 = p - a;
		float d00 = dot(v0, v0);
		float d01 = dot(v0, v1);
		float d11 = dot(v1, v1);
		float d20 = dot(v2, v0);
		float d21 = dot(v2, v1);
		float denom = d00 * d11 - d01 * d01;
		float alpha = (d11 * d20 - d01 * d21) / denom;
		float beta = (d00 * d21 - d01 * d20) / denom;
		float gamma = 1.0 - alpha - beta;
		if (alpha >= 0 && alpha <= 1 && beta >= 0 && beta <= 1 && gamma >= 0 && gamma <= 1) return true;
		else return false;
	}

	__device__ bool is_point_on_arc(const glm::vec2 & c, const glm::vec2 & p, const glm::vec2 & q, const glm::vec2 & t) {
		float alpha = myatan2(p - c);
		float beta = myatan2(q - c);
		float gamma = myatan2(t - c);

		if (beta < alpha) beta = beta + 2 * M_PI;
		if (gamma < alpha) gamma = gamma + 2 * M_PI;
		if (gamma < beta) return true;
		else return false;
	}

	__device__ glm::vec3 projection_plane(const glm::vec3 & p, const glm::vec3 & p0, const glm::vec3 & n) {
		float distance = dot(p - p0, n);
		return p - n * distance;
	}

	__device__ glm::vec3 projection_arc(const glm::vec3 & p, const glm::vec3 & c, float r, const glm::vec3 & n, glm::vec3 & t1, glm::vec3 & t2) {
		glm::vec3 s = projection_plane(p, c, n);
		glm::vec3 q = c + r * (s - c) / length(s - c);

		// asume that arc is in xy plane(this is always the case in our system)
		if (!is_point_on_arc(glm::vec2(c[0], c[1]), glm::vec2(t1[0], t1[1]), glm::vec2(t2[0], t2[1]), glm::vec2(q[0], q[1]))) {
			float d1 = length(p - t1);
			float d2 = length(p - t2);
			if (d1 < d2)q = t1;
			else q = t2;
		}
		return q;
	}

	__device__ glm::vec3 projection_segment(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b) {
		glm::vec3 u = b - a;
		glm::vec3 v = p - a;
		float alpha = dot(u, v) / dot(u, u);
		if (alpha <= 0) return a;
		if (alpha > 0 && alpha < 1) return a + alpha * u;
		if (alpha >= 1) return b;
		return glm::vec3(0, 0, 0);
	}

	__device__ void projection_segment(const glm::vec3 & p, const glm::vec3 & a, const glm::vec3 & b, int index1, int index2, glm::vec3 & q, glm::ivec3 & index) {
		glm::vec3 u = b - a;
		glm::vec3 v = p - a;
		float alpha = dot(u, v) / dot(u, u);
		if (alpha <= 0) {
			q = a;
			index = glm::ivec3(index1, RAND_MAX, RAND_MAX);
		}
		if (alpha > 0 && alpha < 1) {
			q = a + alpha * u;
			index = glm::ivec3(index1, index2, RAND_MAX);
		}
		if (alpha >= 1) {
			q = b;
			index = glm::ivec3(index2, RAND_MAX, RAND_MAX);
		}
	}

	__device__ void projection_segments(const glm::vec3 & p, const glm::vec3 & v1, const glm::vec3 & v2, const glm::vec3 & v3, int index1, int index2, int index3,
		glm::vec3 & q, glm::ivec3 & index) {
		glm::vec3 q12; glm::ivec3 index12; projection_segment(p, v1, v2, index1, index2, q12, index12);
		glm::vec3 q13; glm::ivec3 index13; projection_segment(p, v1, v3, index1, index3, q13, index13);
		glm::vec3 q23; glm::ivec3 index23; projection_segment(p, v2, v3, index2, index3, q23, index23);
		float d12 = length(p - q12);
		float d13 = length(p - q13);
		float d23 = length(p - q23);
		if (d12 <= d13 && d12 <= d23) {
			q = q12;
			index = index12;
		}
		if (d13 <= d12 && d13 <= d23) {
			q = q13;
			index = index13;
		}
		if (d23 <= d12 && d23 <= d13) {
			q = q23;
			index = index23;
		}
	}

	__device__ void projection_convsegment(const glm::vec3 & p, const glm::vec3 & c1, const glm::vec3 & c2, float r1, float r2, int index1, int index2,
		glm::vec3 & q, glm::vec3 & s, glm::ivec3 & index) {

		glm::vec3 x = c2 - c1;
		float delta_r = r1 - r2;
		float length_x = length(x);
		float length_x2 = length_x * length_x;

		float alpha = dot(x, p - c1) / length_x2;
		glm::vec3 t = c1 + alpha * x;
		float omega = sqrt(length_x2 - delta_r * delta_r);
		float beta = length(p - t) * delta_r / omega;
		s = t - beta * x / length_x;

		if (is_point_on_segment(s, c1, c2) && length_x > 0.00001) {
			float gamma = delta_r * length(c2 - t + beta * x / length_x) / length_x;
			q = s + (p - s) / length(p - s) * (gamma + r2);
			index = glm::ivec3(index1, index2, RAND_MAX);
			//print_glm::vec3(q, "q 1");
		}
		else {
			glm::vec3 q1 = c1 + r1 * (p - c1) / length(p - c1);
			glm::vec3 q2 = c2 + r2 * (p - c2) / length(p - c2);

			if (sign(length(p - c1) - length(q1 - c1)) * length(p - q1) < sign(length(p - c2) - length(q2 - c2)) * length(p - q2)) {
				s = c1;
				q = q1;
				index = glm::ivec3(index1, RAND_MAX, RAND_MAX);
				//print_glm::vec3(q, "q 2");
			}
			else {
				s = c2;
				q = q2;
				index = glm::ivec3(index2, RAND_MAX, RAND_MAX);
				//print_glm::vec3(q, "q 3");
			}
		}
	}

	__device__ void projection_convtriangle(const glm::vec3 & p, const glm::vec3 & c1, const glm::vec3 & c2, const glm::vec3 & c3,
		float r1, float r2, float r3, int index1, int index2, int index3,
		const glm::vec3 & v1, const glm::vec3 & v2, const glm::vec3 & v3, const glm::vec3 & n,
		const glm::vec3 & u1, const glm::vec3 & u2, const glm::vec3 & u3, const glm::vec3 & m, const glm::vec3 & camera_ray,
		glm::vec3 & q, glm::vec3 & s, glm::ivec3 & index) {

		// project on triangle
		glm::vec3 l = normalize(cross(c2 - c1, c3 - c1));
		index = glm::ivec3(index1, index2, index3);

		glm::vec3 s1, s2, q1, q2;
		float cos_alpha, distance;
		bool f1 = false; bool f2 = false;

		if (dot(n, camera_ray) < 0) {
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
		if (dot(m, camera_ray) < 0) {
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
		//print_int(f1, "f1"); print_int(f2, "f2");

		if (f1 && f2) {
			if (length(p - q1) < length(p - q2)) { q = q1; s = s1; }
			else { q = q2; s = s2; } return;
		}
		if (f1 && !f2) { q = q1; s = s1; return; }
		if (f2 && !f1) { q = q2; s = s2; return; }

		// project on convsegments
		glm::vec3 q12; glm::vec3 s12; glm::ivec3 index12;
		projection_convsegment(p, c1, c2, r1, r2, index1, index2, q12, s12, index12);
		glm::vec3 q13; glm::vec3 s13; glm::ivec3 index13;
		projection_convsegment(p, c1, c3, r1, r3, index1, index3, q13, s13, index13);
		glm::vec3 q23; glm::vec3 s23; glm::ivec3 index23;
		projection_convsegment(p, c2, c3, r2, r3, index2, index3, q23, s23, index23);

		float d12 = sign(length(p - s12) - length(q12 - s12)) * length(p - q12);
		float d13 = sign(length(p - s13) - length(q13 - s13)) * length(p - q13);
		float d23 = sign(length(p - s23) - length(q23 - s23)) * length(p - q23);

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

	__device__ void projection(const glm::vec3 & p, const glm::vec3 & camera_ray, int & min_j, glm::vec3 & min_q, glm::vec3 & min_s, glm::ivec3 & min_index, int constraint_index, bool wrist_point) {

		int pinky_base_block_index = 2; int index_base_block_index = 11;
		bool debug = false;
		//if (debug) NUM_BLOCKS = 1;

		glm::vec3 q;
		glm::vec3 s;
		glm::ivec3 index;

		/*glm::vec3 second_min_s;
		glm::vec3 second_min_q;
		glm::ivec3 second_min_index;		
		int second_min_j;
		float second_min_distance = RAND_MAX;*/

		float distance; float fraction = 1.0; float angle = 1;
		float min_distance = RAND_MAX;
		for (size_t j = 0; j < NUM_BLOCKS; j++) {

			if (fit_wrist_separately) {
				if (wrist_point) {
					if (j != 28 && j != 29) continue;					
				}
				else {
					if (j == 28 || j == 29) continue;
				}
			}

			if (blocks[D * j + 2] > NUM_CENTERS) {
				int index1 = blocks[D * j];
				int index2 = blocks[D * j + 1];
				glm::vec3 c1 = glm::vec3(centers[D * index1], centers[D * index1 + 1], centers[D * index1 + 2]);
				glm::vec3 c2 = glm::vec3(centers[D * index2], centers[D * index2 + 1], centers[D * index2 + 2]);
				float r1 = radii[index1];
				float r2 = radii[index2];

				if (debug) {
					print_int(index1, "index1"); print_int(index2, "index2");
					print_vec3(c1, "c1"); print_vec3(c2, "c2");
					print_float(r1, "r1"); print_float(r2, "r2");
				}
				projection_convsegment(p, c1, c2, r1, r2, index1, index2, q, s, index);

				if (glm::length(q - s) < 0.000001) {
					printf("CorrespondenceFinger.h: projection(), projection_convsegment(): q = s, p = %f, %f, %f, c1 = %f %f %f, c2 = %f %f %f, r1 = %f, r2 = %f\n",
						p[0], p[1], p[2], c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], r1, r2);
				}

				if (disable_finger_bases_fitting && (j == pinky_base_block_index || j == index_base_block_index)) {
					int theta_info_index;
					if (j == pinky_base_block_index) theta_info_index = 25;
					if (j == index_base_block_index) theta_info_index = 13;
					fraction = glm::length(s - c1) / glm::length(c2 - c1);
					glm::mat3 global_inv = glm::inverse(glm::mat3(theta_infos[theta_info_index].mat));
					glm::vec3 offset = global_inv * (q - s); angle = offset[2];
				}
			}
			else {
				int index1 = blocks[D * j];
				int index2 = blocks[D * j + 1];
				int index3 = blocks[D * j + 2];
				glm::vec3 c1 = glm::vec3(centers[D * index1], centers[D * index1 + 1], centers[D * index1 + 2]);
				glm::vec3 c2 = glm::vec3(centers[D * index2], centers[D * index2 + 1], centers[D * index2 + 2]);
				glm::vec3 c3 = glm::vec3(centers[D * index3], centers[D * index3 + 1], centers[D * index3 + 2]);
				float r1 = radii[index1];
				float r2 = radii[index2];
				float r3 = radii[index3];
				glm::vec3 v1 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j], tangent_points[NUM_TANGENT_FIELDS * D * j + 1], tangent_points[NUM_TANGENT_FIELDS * D * j + 2]);
				glm::vec3 v2 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 3], tangent_points[NUM_TANGENT_FIELDS * D * j + 4], tangent_points[NUM_TANGENT_FIELDS * D * j + 5]);
				glm::vec3 v3 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 6], tangent_points[NUM_TANGENT_FIELDS * D * j + 7], tangent_points[NUM_TANGENT_FIELDS * D * j + 8]);
				glm::vec3 n = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 9], tangent_points[NUM_TANGENT_FIELDS * D * j + 10], tangent_points[NUM_TANGENT_FIELDS * D * j + 11]);
				glm::vec3 u1 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 12], tangent_points[NUM_TANGENT_FIELDS * D * j + 13], tangent_points[NUM_TANGENT_FIELDS * D * j + 14]);
				glm::vec3 u2 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 15], tangent_points[NUM_TANGENT_FIELDS * D * j + 16], tangent_points[NUM_TANGENT_FIELDS * D * j + 17]);
				glm::vec3 u3 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 18], tangent_points[NUM_TANGENT_FIELDS * D * j + 19], tangent_points[NUM_TANGENT_FIELDS * D * j + 20]);
				glm::vec3 m = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * j + 21], tangent_points[NUM_TANGENT_FIELDS * D * j + 22], tangent_points[NUM_TANGENT_FIELDS * D * j + 23]);

				if (debug) {
					print_int(index1, "index1"); print_int(index2, "index2"); print_int(index3, "index3");
					print_vec3(c1, "c1"); print_vec3(c2, "c2"); print_vec3(c3, "c3");
					print_float(r1, "r1"); print_float(r2, "r2"); print_float(r3, "r3");
					print_vec3(v1, "v1"); print_vec3(v2, "v2"); print_vec3(v3, "v3"); print_vec3(n, "n");
					print_vec3(u1, "u1"); print_vec3(u2, "u2"); print_vec3(u3, "u3"); print_vec3(m, "m");
				}
				projection_convtriangle(p, c1, c2, c3, r1, r2, r3, index1, index2, index3, v1, v2, v3, n, u1, u2, u3, m, camera_ray, q, s, index);

				if (glm::length(q - s) < 0.000001) {
					printf("CorrespondenceFinger.h: projection(), projection_convtriangle(): q = s, p = %f, %f, %f, c1 = %f %f %f, c2 = %f %f %f, c3 = %f %f %f, r1 = %f, r2 = %f, r3 = %f\n",
						p[0], p[1], p[2], c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], c3[0], c3[1], c3[2], r1, r2, r3);
				}

			}
			if (debug) { print_ivec3(index, "index"); print_int(j, "j"); print_vec3(q, "q"); print_vec3(s, "s"); printf("\n"); }

			distance = sign(length(p - s) - length(q - s)) * length(p - q);

			if (distance < min_distance) {
				if (!disable_finger_bases_fitting || 
					((j != pinky_base_block_index && j != index_base_block_index) || fraction > 0.6 || angle < 0)) {
					min_s = s;
					min_q = q;
					min_index = index;
					min_distance = distance;
					min_j = j;
				}
			}

			/*if (distance < second_min_distance) {
				if (distance < min_distance) {

					/// copy first min to second min
					if ((j == 2 || j == 5 || j == 8 ||j == 11) && (min_j == 20 || min_j == 21 || min_j == 22 || min_j == 23 || min_j == 24 || min_j == 25)) {
						second_min_distance = min_distance;
						second_min_s = min_s;
						second_min_q = min_q;
						second_min_index = min_index;
						second_min_j = min_j;
					}

					/// asign first min
					min_distance = distance;
					min_s = s;
					min_q = q;
					min_index = index;
					min_j = j;
				}
				else {
					if ((j == 2 || j == 5 || j == 8 || j == 11) && (min_j == 20 || min_j == 21 || min_j == 22 || min_j == 23 || min_j == 24 || min_j == 25)) {
						second_min_distance = distance;
						second_min_s = s;
						second_min_q = q;
						second_min_index = index;
						second_min_j = j;
					}
				}
			}*/
		}
		/*if ((min_j == 2 || min_j == 5 || min_j == 8 || min_j == 11) && (second_min_j == 20 || second_min_j == 21 || second_min_j == 22 || second_min_j == 23 || second_min_j == 24 || second_min_j == 25)) {
			if (second_min_distance / min_distance < 4.0f) {
				min_s = second_min_s;
				min_q = second_min_q;
				min_index = second_min_index;
				min_j = second_min_j;
			}
		}*/
		if (debug) { print_int(min_j, "j_min"); print_vec3(min_q, "q_min"); print_vec3(min_s, "s_min"); print_ivec3(min_index, "index_min"); }
		if (isnan(min_s[0]) || isnan(min_s[1]) || isnan(min_s[2]) || isinf(min_s[0]) || isinf(min_s[1]) || isinf(min_s[2])) printf("CorrespondenceFinger.h: projection(): s is nan or inf\n");
		if (glm::length(min_q - min_s) < 0.000001) printf("CorrespondenceFinger.h: projection(): q = s\n");
	}

	__device__ void backfacing(const glm::vec3 & p, const glm::vec3 & camera_ray, int & min_j, glm::vec3 & min_q, glm::vec3 & min_s, glm::ivec3 & min_index) {
		bool debug = false;
		if (dot(camera_ray, min_q - min_s) < 0)  return;
		if (blocks[D * min_j + 2] < NUM_CENTERS) {
			if (debug) printf("%s\n", "backfacing");
			bool f1 = false;
			bool f2 = false;
			glm::vec3 q1, q2; glm::ivec3 index1, index2;
			glm::vec3 n = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 9], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 10], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 11]);
			if (dot(n, camera_ray) < 0) {
				f1 = true;
				glm::vec3 v1 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 1], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 2]);
				glm::vec3 v2 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 3], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 4], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 5]);
				glm::vec3 v3 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 6], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 7], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 8]);
				projection_segments(p, v1, v2, v3, blocks[D * min_j], blocks[D * min_j + 1], blocks[D * min_j + 2], q1, index1);
			}
			glm::vec3 m = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 21], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 22], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 23]);
			if (dot(m, camera_ray) < 0) {
				f2 = true;
				glm::vec3 v1 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 12], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 13], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 14]);
				glm::vec3 v2 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 15], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 16], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 17]);
				glm::vec3 v3 = glm::vec3(tangent_points[NUM_TANGENT_FIELDS * D * min_j + 18], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 19], tangent_points[NUM_TANGENT_FIELDS * D * min_j + 20]);
				projection_segments(p, v1, v2, v3, blocks[D * min_j], blocks[D * min_j + 1], blocks[D * min_j + 2], q2, index2);
			}
			if (f1 && f2) {
				if (length(p - q1) < length(p - q2)) { min_q = q1; min_index = index1; }
				else { min_q = q2; min_index = index2; }
			}
			if (f1 && !f2) { min_q = q1; min_index = index1; }
			if (f2 && !f1) { min_q = q2; min_index = index2; }
			if (!f1 && !f2) { min_q = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX); }
		}
		else {
			min_q = glm::vec3(RAND_MAX, RAND_MAX, RAND_MAX);
		}

		if (debug) { print_int(min_j, "j back"); print_vec3(min_q, "q back"); print_vec3(min_s, "s back"); print_ivec3(min_index, "index back"); }
		if (isnan(min_s[0]) || isnan(min_s[1]) || isnan(min_s[2]) || isinf(min_s[0]) || isinf(min_s[1]) || isinf(min_s[2])) printf("CorrespondenceFinger.h: backfacing(): s is nan or inf\n");
		if (glm::length(min_q - min_s) < 0.000001) printf("CorrespondenceFinger.h: backfacing(): q = s\n");
	}

	__device__ void projection_outline(const glm::vec3 & p, const glm::vec3 & camera_ray, int & min_j, glm::vec3 & min_q, glm::vec3 & min_s, glm::ivec3 & min_index, bool wrist_point) {
		size_t shift_indices = 6;
		size_t shift_start = 0;
		size_t shift_end = 3;
		size_t shift_block = 8;

		float min_distance;
		glm::vec3 t1, t2, c1, c2, q, s, temp_c; float r1, r2, temp_r; int index1, index2; glm::ivec3 temp_index;
		min_distance = RAND_MAX;
		for (size_t j = 0; j < NUM_OUTLINES; j++) {
			index1 = outline[D * NUM_OUTLINE_FIELDS * j + shift_indices];
			index2 = outline[D * NUM_OUTLINE_FIELDS * j + shift_indices + 1];

			if (fit_wrist_separately) {
				if (wrist_point) {
					if (index1 != 34 && index1 != 35 && index1 != 36 && index1 != 37) continue;
				}
				else {
					if (index1 == 34 || index1 == 35 || index1 == 36 || index1 == 37) continue;
				}
			}

			if (disable_finger_bases_fitting) {
				int pinky_base_center_index = 3; int pinky_bottom_center_index = 2;
				int index_base_center_index = 15; int index_bottom_center_index = 14;
				if ((index1 == pinky_base_center_index && index2 == pinky_bottom_center_index) || (index2 == pinky_base_center_index && index1 == pinky_bottom_center_index)
					|| (index1 == index_base_center_index && index2 == index_bottom_center_index) || (index1 == index_bottom_center_index && index2 == index_base_center_index)) continue;
			}

			if (outline[D * NUM_OUTLINE_FIELDS * j + shift_indices + 1] == RAND_MAX) {
				
				t1 = glm::vec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_start], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 2]);
				t2 = glm::vec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_end], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 2]);
				c1 = glm::vec3(centers[D * index1], centers[D * index1 + 1], centers[D * index1 + 2]);
				r1 = radii[index1];
				q = projection_arc(p, c1, r1, camera_ray, t1, t2);
				s = glm::vec3(centers[D * index1], centers[D * index1 + 1], centers[D * index1 + 2]);
			}
			else {
				t1 = glm::vec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_start], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 2]);
				t2 = glm::vec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_end], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 2]);
				q = projection_segment(p, t1, t2);

				c1 = glm::vec3(centers[D * index1], centers[D * index1 + 1], centers[D * index1 + 2]);
				c2 = glm::vec3(centers[D * index2], centers[D * index2 + 1], centers[D * index2 + 2]);
				r1 = radii[index1];
				r2 = radii[index2];
				if (r2 > r1) {
					temp_c = c1; c1 = c2; c2 = temp_c;
					temp_r = r1; r1 = r2; r2 = temp_r;
				}
				projection_convsegment(q, c1, c2, r1, r2, index1, index2, temp_c, s, temp_index);

				if (glm::length(q - s) < 0.000001) {
					printf("CorrespondenceFinger.h: projection_outline(), projection_convsegment(): q = s, p = %f, %f, %f, c1 = %f %f %f, c2 = %f %f %f, r1 = %f, r2 = %f\n",
						p[0], p[1], p[2], c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], r1, r2);
				}
	
				if (isnan(s[0]) || isnan(s[1]) || isnan(s[2]) || isinf(s[0]) || isinf(s[1]) || isinf(s[2]) || glm::length(c1 - c2) == 0) {
					printf("p = %f %f %f, t1 %f %f %f, t2 = %f %f %f, q = %f %f %f, c1 = %f %f %f, c2 = %f %f %f, r1 = %f, r2 = %f, index1  = %d, index2 = %d\n", 
						p[0], p[1], p[2], t1[0], t1[1], t1[2], t2[0], t2[1], t2[2], q[0], q[1], q[2], c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], r1, r2, index1, index2);
				}
			}
			if (length(p - q) < min_distance) {
				min_distance = length(p - q);
				min_q = q;
				min_index = glm::ivec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_indices], outline[D * NUM_OUTLINE_FIELDS * j + shift_indices + 1], RAND_MAX);
				min_s = s;
				min_j = outline[D * NUM_OUTLINE_FIELDS * j + shift_block];
			}
		}
		if (isnan(min_s[0]) || isnan(min_s[1]) || isnan(min_s[2]) || isinf(min_s[0]) || isinf(min_s[1]) || isinf(min_s[2])) printf("CorrespondenceFinger.h: projection_outline(): s is nan or inf\n");
		if (glm::length(min_q - min_s) < 0.000001) printf("CorrespondenceFinger.h: projection_outline(): q = s\n");
	}

	__device__ void find(const glm::vec3 & p, int & b, glm::vec3 & q, glm::vec3 & s, glm::ivec3 & index, int constraint_index, bool wrist_point) {
		glm::vec3 q_o;
		glm::vec3 s_o;
		glm::ivec3 index_o;
		int b_o;
		glm::vec3 camera_ray = glm::vec3(0, 0, 1);
		projection(p, camera_ray, b, q, s, index, constraint_index, wrist_point);
		backfacing(p, camera_ray, b, q, s, index);		
		projection_outline(p, camera_ray, b_o, q_o, s_o, index_o, wrist_point);
		if (length(p - q_o) < length(p - q)) {
			q = q_o;
			index = index_o;
			s = s_o;
			b = b_o;
		}
	}

	/*__device__ void operator()(glm::vec4 input) {
		glm::vec3 p = glm::vec3(input[0], input[1], input[2]);
		int n = (int)input[3];

		glm::vec3 q, q_o;
		glm::vec3 s, s_o;
		glm::ivec3 index, index_o;
		int b, b_o;
		glm::vec3 camera_ray = glm::vec3(0, 0, 1);
		projection(p, camera_ray, b, q, s, index);
		backfacing(p, camera_ray, b, q, s, index);
		
		projection_outline(p, camera_ray, b_o, q_o, s_o, index_o);

		if (length(p - q_o) < length(p - q)) {
			q = q_o;
			index = index_o;
			s = s_o;
			b = b_o;			
		}

		for (size_t i = 0; i < D; i++) {
			output[4 * D * n + i] = q[i];
			output[4 * D * n + 3 + i] = index[i];
			output[4 * D * n + 6 + i] = s[i];			
		}
		output[4 * D * n + 9] = b;		
	}*/
};