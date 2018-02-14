/// @warning NEVER include this on the C++ side, only externals
#pragma once

#ifndef __CUDACC__
    #error you cannot compile c++ of this, only cudax/externs.h can be included
#endif

///--- system
#include <iostream>
#include <cuda_runtime.h>
#include <thrust/device_vector.h>
#include "cudax/cuda_glm.h"
#include "cudax/externs.h" ///< only thing that is exposed to C++
#include "tracker/Energy/Fitting/Settings.h"

namespace cudax {

class KinectCamera;
struct CudaTimer;
class PixelIndexer;

energy::fitting::Settings* settings;

__constant__ float focal_length_x;
__constant__ float focal_length_y;
__constant__ int width;
__constant__ int height;
int H_width;
int H_height;

///--- These are the arrays mapped to the textures below
cudaArray* sensor_depth_array = NULL;
cudaArray* rendered_block_indices_array = NULL;
cudaArray* rendered_depth_array = NULL;

///--- Textures containing the OpenGL input data (must be declared in global scope)
texture <ushort1, 2, cudaReadModeElementType> sensor_depth_texture_cuda;
texture <uchar, 2, cudaReadModeElementType> rendered_block_indices_texture_cuda;
texture <ushort1, 2, cudaReadModeElementType> rendered_depth_texture_cuda;
CudaTimer* t = NULL;

thrust::device_vector<int>* _sensor_silhouette_distance_transform = NULL;
thrust::device_vector<int>* _sensor_silhouette_wrist_distance_transform = NULL;
thrust::device_vector<int>* _sensor_outline_distance_transform = NULL;
    
thrust::device_vector<float> * J = NULL; 
thrust::device_vector<float>* F = NULL;
thrust::device_vector<float>* JtJ = NULL;
thrust::device_vector<float>* JtF = NULL; 

thrust::device_vector<float>* _weighted_metric_sensor_palm = NULL;
thrust::device_vector<float>* _weighted_metric_sensor_fingers = NULL;
thrust::device_vector<float>* _weighted_metric_rendered_palm = NULL;
thrust::device_vector<float>* _weighted_metric_rendered_fingers = NULL;

uchar* opencv_image = NULL;

KinectCamera* camera_matrix = NULL;
thrust::device_vector<ThetaInfo> device_pointer_theta_infos;
thrust::device_vector<BetaInfo> device_pointer_beta_infos;
thrust::device_vector<KinematicChain> device_pointer_kinematic_chains;
thrust::device_vector<ShapeChain> device_pointer_shape_chains;
    
#define SEGMENT_VALUES 44
#define SEGMENT_JOINTS 17

bool _multiply_on_cpu;
bool host_calibrate;
bool _verify_jacobian;
bool _compute_weighted_metric;

__device__ bool device_multiply_on_cpu;
__device__ bool device_calibrate;

int _num_parameters;
__device__ int D;
__device__ int NUM_PARAMETERS;
__device__ int NUM_CENTERS;
__device__ int NUM_BLOCKS;
__device__ int NUM_OUTLINES;
__device__ int NUM_TANGENT_FIELDS;
__device__ int NUM_OUTLINE_FIELDS;


__device__ int _num_sensor_points;
thrust::device_vector<int>* _sensor_indicator = NULL;
__device__ int _num_rendered_points;
thrust::device_vector<int>* _rendered_pixels = NULL;
thrust::device_vector<float>* _rendered_points = NULL;
thrust::device_vector<float>* _rendered_normals = NULL;
thrust::device_vector<int>* _rendered_block_ids = NULL;

thrust::device_vector<int>* push_indices = NULL;
thrust::device_vector<int>* pull_indices = NULL;

thrust::device_vector<float>* device_pointer_centers = NULL;
thrust::device_vector<float>* device_pointer_radii = NULL;
thrust::device_vector<int>* device_pointer_blocks = NULL;
thrust::device_vector<float>* device_pointer_tangent_points = NULL;
thrust::device_vector<float>* device_pointer_outline = NULL;
thrust::device_vector<int>* device_pointer_blockid_to_pose_unit_id_map = NULL;
thrust::device_vector<int>* device_pointer_blockid_to_shape_unit_id_map = NULL;

thrust::device_vector<float>* _correspondences_data_points = NULL;
thrust::device_vector<float>* _correspondences_model_points = NULL;
thrust::device_vector<int>* _correspondences_model_indices = NULL;
thrust::device_vector<int>* _correspondences_block_indices = NULL;
thrust::device_vector<float>* _jacobian_data = NULL;
thrust::device_vector<float>* _correspondences_silhouette_data_points = NULL;
thrust::device_vector<float>* _correspondences_silhouette_model_points = NULL;
thrust::device_vector<int>* _correspondences_silhouette_indices = NULL;
thrust::device_vector<float>* _jacobian_silhouette = NULL;
} 





