#include "util/gl_wrapper.h" ///< for cuda_gl_interop
#include <cuda_gl_interop.h>
#include <thrust/host_vector.h>
#include <thrust/copy.h>
#include <thrust/sequence.h>
#include <thrust/count.h>

#include "cudax/kernel.h"
#include "cudax/CudaTimer.h"
#include "cudax/helper_cuda.h" ///< SDK error checking
#include "cudax/CublasHelper.h"
#include "cudax/CudaHelper.h"
#include "cudax/KinectCamera.h"
#include "cudax/kernel_init.h"
#include "cudax/kernel_upload.h"
#include "cudax/kernel_debug.h"

#include "cudax/functors/ComputeJacobianSilhouette.h"
#include "cudax/functors/ComputeJacobianData.h"

#include <iostream>
#include <fstream>
#include <string>

using namespace cudax;

struct absolute_value : public thrust::unary_function < float, float > {
	__host__ __device__
		float operator()(float x) const {
		return (x >= 0) ? x : -x;
	}
};

void kernel_bind() {
	if (cudax::sensor_depth_array)
		CHECK_CUDA(cudaBindTextureToArray(sensor_depth_texture_cuda, cudax::sensor_depth_array));
	if (settings->fit2D_silhouette_enable) {
		if (cudax::rendered_block_indices_array)
			CHECK_CUDA(cudaBindTextureToArray(rendered_block_indices_texture_cuda, cudax::rendered_block_indices_array));
		if (cudax::rendered_depth_array)
			CHECK_CUDA(cudaBindTextureToArray(rendered_depth_texture_cuda, cudax::rendered_depth_array));
	}
}

void kernel_unbind() {
	if (cudax::sensor_depth_array) CHECK_CUDA(cudaUnbindTexture(sensor_depth_texture_cuda));
	cudax::sensor_depth_array = NULL;
	if (settings->fit2D_silhouette_enable) {
		if (cudax::rendered_block_indices_array) CHECK_CUDA(cudaUnbindTexture(rendered_block_indices_texture_cuda));
		cudax::rendered_block_indices_array = NULL;
		if (cudax::rendered_depth_array) CHECK_CUDA(cudaUnbindTexture(rendered_depth_texture_cuda));
		cudax::rendered_depth_array = NULL;
	}
}

void multiply_on_cpu(float* eigen_JtJ, float* eigen_JtF, thrust::device_vector<float> * J, thrust::device_vector<float>* F, int n_total, size_t frame_id, size_t iter) {
	thrust::host_vector<float> F_host(n_total);
	thrust::copy(F->begin(), F->begin() + n_total, F_host.begin());
	thrust::host_vector<float> J_host(n_total * _num_parameters);
	thrust::copy(J->begin(), J->begin() + n_total * _num_parameters, J_host.begin());

	for (size_t i = 0; i < _num_parameters; i++) {
		double result = 0;
		for (size_t k = 0; k < n_total; k++) {
			result = result + (double)J_host[k * _num_parameters + i] * (double)F_host[k];
		}
		eigen_JtF[i] = result;
	}
	for (size_t i = 0; i < _num_parameters; i++) {
		for (size_t j = 0; j < _num_parameters; j++) {
			double result = 0;
			for (size_t k = 0; k < n_total; k++) {
				result = result + (double)J_host[k * _num_parameters + i] * (double)J_host[k * _num_parameters + j];
			}
			eigen_JtJ[i * _num_parameters + j] = result;
		}
	}
	
	if (settings->write_jacobian_to_file) {
		// write jacobian to file
		std::ofstream output_file;
		std::string data_path = "C:/Users/tkach/Desktop/Test/";
		output_file.open(data_path + "j-" + std::to_string(frame_id) + "-" + std::to_string(iter) + ".txt");
		for (size_t i = 0; i < n_total; i++) {
			for (size_t k = 0; k < _num_parameters; k++) {
				output_file << (double)J_host[k * _num_parameters + i] << " ";
			}
		}
		output_file.close();

		// write function to file
		output_file.open(data_path + "f-" + std::to_string(frame_id) + "-" + std::to_string(iter) + ".txt");
		for (size_t i = 0; i < n_total; i++) {
			output_file << (double)F_host[i] << std::endl;
		}
		output_file.close();
	}
	
}

float compute_weighted_metric(int num_sensor_points, int num_rendered_points) {

	//float alpha_fingers = 0.8; float alpha_palm = 1 - alpha_fingers;
	//float alpha_sensor = 0.8; float alpha_rendered = 1 - alpha_sensor;

	float alpha_fingers = 0.5; float alpha_palm = 1 - alpha_fingers;
	float alpha_sensor = 1; float alpha_rendered = 1 - alpha_sensor;

	/// Sensor metrics	
	thrust::transform(_weighted_metric_sensor_palm->begin(), _weighted_metric_sensor_palm->begin() + num_sensor_points, _weighted_metric_sensor_palm->begin(), absolute_value());
	float error_sensor_palm = thrust::reduce(_weighted_metric_sensor_palm->begin(), _weighted_metric_sensor_palm->begin() + num_sensor_points);
	int num_sensor_points_palm = num_sensor_points - thrust::count(_weighted_metric_sensor_palm->begin(), _weighted_metric_sensor_palm->begin() + num_sensor_points, 0);
		
	thrust::transform(_weighted_metric_sensor_fingers->begin(), _weighted_metric_sensor_fingers->begin() + num_sensor_points, _weighted_metric_sensor_fingers->begin(), absolute_value());
	float error_sensor_fingers = thrust::reduce(_weighted_metric_sensor_fingers->begin(), _weighted_metric_sensor_fingers->begin() + num_sensor_points);
	int num_sensor_points_fingers = num_sensor_points - thrust::count(_weighted_metric_sensor_fingers->begin(), _weighted_metric_sensor_fingers->begin() + num_sensor_points, 0);

	//std::cout << "num_sensor_points = " << num_sensor_points << ", num_sensor_points_palm = " << num_sensor_points_palm << ", num_sensor_points_fingers = " << num_sensor_points_fingers <<
	//	", sum = " << num_sensor_points_palm + num_sensor_points_fingers << std::endl;
	
	/// Rendered metrics	
	thrust::transform(_weighted_metric_rendered_palm->begin(), _weighted_metric_rendered_palm->begin() + num_rendered_points, _weighted_metric_rendered_palm->begin(), absolute_value());
	float error_rendered_palm = thrust::reduce(_weighted_metric_rendered_palm->begin(), _weighted_metric_rendered_palm->begin() + num_rendered_points);
	int num_rendered_points_palm = num_rendered_points - thrust::count(_weighted_metric_rendered_palm->begin(), _weighted_metric_rendered_palm->begin() + num_rendered_points, 0);

	thrust::transform(_weighted_metric_rendered_fingers->begin(), _weighted_metric_rendered_fingers->begin() + num_rendered_points, _weighted_metric_rendered_fingers->begin(), absolute_value());
	float error_rendered_fingers = thrust::reduce(_weighted_metric_rendered_fingers->begin(), _weighted_metric_rendered_fingers->begin() + num_rendered_points);
	int num_rendered_points_fingers = num_rendered_points - thrust::count(_weighted_metric_rendered_fingers->begin(), _weighted_metric_rendered_fingers->begin() + num_rendered_points, 0);

	//std::cout << "num_rendered_points = " << num_rendered_points << ", num_rendered_points_palm = " << num_rendered_points_palm << ", num_rendered_points_fingers = " << num_rendered_points_fingers <<
	//	", sum = " << num_rendered_points_palm + num_rendered_points_fingers << std::endl << std::endl;

	/// Weighted metric
	float weighted_error_sensor = (alpha_palm * error_sensor_palm + alpha_fingers * error_sensor_fingers) / (alpha_palm * num_sensor_points_palm + alpha_fingers * num_sensor_points_fingers);
	float weighted_error_rendered = (alpha_palm * error_rendered_palm + alpha_fingers * error_rendered_fingers) / (alpha_palm * num_rendered_points_palm + alpha_fingers * num_rendered_points_fingers);

	float weighted_error = alpha_sensor * weighted_error_sensor + alpha_rendered * weighted_error_rendered;

	return weighted_error;
}

void kernel(float* eigen_JtJ, float* eigen_JtF, float & push_error, float & pull_error, float & weighted_error, bool eval_metric,
	bool reweight, int frame_id, int iter, int num_sensor_points, int num_rendered_points,
	float * correspondences_data_points, float * correspondences_model_points, int * correspondences_model_indices, int * correspondences_block_indices, float * jacobian_data,
	float * correspondences_silhouette_data_points, float * correspondences_silhouette_model_points, int * correspondences_silhouette_indices, float * jacobian_silhouette) {

	int n_pull, n_push;

	if (settings->fit2D_outline_enable || settings->fit2D_silhouette2outline_enable || settings->fit2D_silhouette_enable) {
		if (settings->fit2D_unproject) {
			n_push = 1 * num_rendered_points;
		}
		else {
			n_push = 2 * num_rendered_points;
		}
		n_pull = 1 * num_sensor_points;
	}
	else {
		num_rendered_points = 0;
		n_push = 0;
		n_pull = 1 * num_sensor_points; 
	}

	int n_total = n_pull + n_push;
	thrust::fill(J->begin(), J->begin() + n_total * _num_parameters, 0.0f);
	thrust::fill(F->begin(), F->begin() + n_total, 0.0f);
	if (_compute_weighted_metric) {
		thrust::fill(_weighted_metric_sensor_palm->begin(), _weighted_metric_sensor_palm->begin() + num_sensor_points, 0.0f);
		thrust::fill(_weighted_metric_sensor_fingers->begin(), _weighted_metric_sensor_fingers->begin() + num_sensor_points, 0.0f);
		thrust::fill(_weighted_metric_rendered_palm->begin(), _weighted_metric_rendered_palm->begin() + num_rendered_points, 0.0f);
		thrust::fill(_weighted_metric_rendered_fingers->begin(), _weighted_metric_rendered_fingers->begin() + num_rendered_points, 0.0f);
	}
	if (n_total == 0) return;

	float * J_push = thrust::raw_pointer_cast(J->data());
	float * J_pull = J_push + n_push * _num_parameters;
	float* F_push = thrust::raw_pointer_cast(F->data());
	float* F_pull = F_push + n_push;

	/// Parallel run
	ComputeJacobianSilhouette jacobian_silhouette_functor(J_push, F_push, _verify_jacobian, _compute_weighted_metric);
	ComputeJacobianData jacobian_data_functor(J_pull, F_pull, reweight, _verify_jacobian, _compute_weighted_metric);
	
	if (settings->fit2D_outline_enable || settings->fit2D_silhouette2outline_enable || settings->fit2D_silhouette_enable) {
		thrust::sequence(push_indices->begin(), push_indices->begin() + num_rendered_points);
		thrust::for_each(push_indices->begin(), push_indices->begin() + num_rendered_points, jacobian_silhouette_functor);
	}
	if (settings->fit3D_enable) {
		thrust::sequence(pull_indices->begin(), pull_indices->begin() + num_sensor_points);
		thrust::for_each(pull_indices->begin(), pull_indices->begin() + num_sensor_points, jacobian_data_functor);
	}
	
	/// Multiply with GPU
	CublasHelper::outer_product_J(*J, *JtJ, n_total, _num_parameters);
	thrust::copy(JtJ->begin(), JtJ->end(), eigen_JtJ);
	CublasHelper::vector_product_J(*J, *F, *JtF, n_total, _num_parameters);
	thrust::copy(JtF->begin(), JtF->end(), eigen_JtF);

	/// Multiply with CPU
	if (_multiply_on_cpu) {
		multiply_on_cpu(eigen_JtJ, eigen_JtF, J, F, n_total, frame_id, iter);
	}

	/// Only need evaluate metric on the last iteration
	if (eval_metric) {
		thrust::device_vector<float> f_pull(n_pull);
		thrust::transform(F->begin() + n_push, F->begin() + n_push + n_pull, f_pull.begin(), absolute_value());
		pull_error = thrust::reduce(f_pull.begin(), f_pull.end());
		pull_error = pull_error / n_pull;
		thrust::device_vector<float> f_push(n_push);
		thrust::transform(F->begin(), F->begin() + n_push, f_push.begin(), absolute_value());
		push_error = thrust::reduce(f_push.begin(), f_push.end());
		push_error = push_error / n_push;

		if (_compute_weighted_metric) weighted_error = compute_weighted_metric(num_sensor_points, num_rendered_points);
	}

	//Return the correspondences	
	if (_verify_jacobian) {
		thrust::copy(_correspondences_data_points->begin(), _correspondences_data_points->begin() + num_sensor_points * 3, correspondences_data_points);
		thrust::copy(_correspondences_model_points->begin(), _correspondences_model_points->begin() + num_sensor_points * 3, correspondences_model_points);
		thrust::copy(_correspondences_model_indices->begin(), _correspondences_model_indices->begin() + num_sensor_points * 3, correspondences_model_indices);
		thrust::copy(_correspondences_block_indices->begin(), _correspondences_block_indices->begin() + num_sensor_points, correspondences_block_indices);
		thrust::copy(_jacobian_data->begin(), _jacobian_data->begin() + num_sensor_points * 3 * _num_parameters, jacobian_data);

		thrust::copy(_correspondences_silhouette_data_points->begin(), _correspondences_silhouette_data_points->begin() + num_rendered_points * 3, correspondences_silhouette_data_points);
		thrust::copy(_correspondences_silhouette_model_points->begin(), _correspondences_silhouette_model_points->begin() + num_rendered_points * 3, correspondences_silhouette_model_points);
		thrust::copy(_correspondences_silhouette_indices->begin(), _correspondences_silhouette_indices->begin() + num_rendered_points * 3, correspondences_silhouette_indices);
		thrust::copy(_jacobian_silhouette->begin(), _jacobian_silhouette->begin() + num_rendered_points * 3 * _num_parameters, jacobian_silhouette);
	}
}
