#pragma once
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include "cudax/cuda_glm.h"
#include <vector>

namespace energy{namespace fitting{struct Settings;}}
struct ThetaInfo;
struct BetaInfo;
struct KinematicChain;
struct ShapeChain;
typedef unsigned char uchar;

namespace cudax {

extern cudaArray* sensor_depth_array;
extern cudaArray* rendered_block_indices_array;
extern cudaArray* rendered_depth_array;
}

extern "C"
void kernel_init(energy::fitting::Settings* settings,
                 int _width,
                 int _height,
                 int num_parameters,
                 float H_focal_length_x,
                 float H_focal_length_y,
                 const float *H_inv_proj_matrix, 
				 int d, int num_centers, int num_blocks, int max_num_outlines, int num_tangent_fields, int num_outline_fields, 
				 bool multiply_on_cpu, bool calibrate, bool verify_jacobian, bool compute_weighted_metric);

extern "C" 
void kernel_upload_kinematic(const std::vector<ThetaInfo>& theta_infos, const std::vector<KinematicChain>& host_pointer_kinematic_chains,
							const std::vector<BetaInfo>& beta_infos, const std::vector<ShapeChain>& host_pointer_shape_chains);

extern "C" 
void kernel_upload_model(int d, int num_parameters, int num_centers, int num_blocks, int num_outlines, int num_tangent_fields, int num_outline_fields, 
const float * host_pointer_centers, const float * host_pointer_radii, const int * host_pointer_blocks, 
const float * host_pointer_tangent_points, const float * host_pointer_outline, const int * host_pointer_blockid_to_pose_unit_id_map, const int * host_pointer_blockid_to_shape_unit_id_map, bool calibrate);

extern "C"
void kernel_upload_data_energy_data(int * sensor_indicator, int num_sensor_points);

extern "C"
void kernel_upload_silhouette_energy_data(int * rendered_pixels, int num_rendered_points);

extern "C"
void kernel_upload_outline_energy_data(int * rendered_pixels, float * rendered_points, float * rendered_normals, int * rendered_block_ids, int num_rendered_points);

extern "C" 
void kernel_upload_sensor_silhouette_distance_transform(int* sensor_silhouette_distance_transform);

extern "C"
void kernel_upload_sensor_silhouette_wrist_distance_transform(int* sensor_silhouette_wrist_distance_transform);

extern "C"
void kernel_upload_sensor_outline_distance_transform(int* sensor_outline_distance_transform);

extern "C" 
void kernel_bind();

extern "C" 
void kernel(float* eigen_JtJ, float* eigen_Jte, float & push_error, float & pull_error, float & weighted_error, bool eval_metric, 
	bool reweight, int id, int iter, int num_sensor_points, int num_rendered_points,
	float * correspondences_data_points, float * correspondences_model_points, int * correspondences_model_indices, int * correspondences_block_indices, float * jacobian_data, 
	float * correspondences_silhouette_data_points, float * correspondences_silhouette_model_points, int * correspondences_silhouette_indices, float * jacobian_silhouette);

extern "C" 
void kernel_unbind();

extern "C"
void kernel_copy_extra_corresp(thrust::host_vector<float4>& H_queries, thrust::host_vector<float4>& H_target, thrust::host_vector<int>& H_idxs);

extern "C"
void kernel_cleanup();

extern "C"
void kernel_memory_tests();

extern "C"
void kernel_constraint_type_image(uchar *, int, int);

extern "C"
void kernel_simplify_jacobian();

extern "C"
void kernel_delete();

