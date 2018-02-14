#pragma once
#include "kernel.h"

using namespace cudax;

void kernel_upload_sensor_silhouette_distance_transform(int * sensor_silhouette_distance_transform){
    thrust::copy(sensor_silhouette_distance_transform, sensor_silhouette_distance_transform + H_width * H_height, _sensor_silhouette_distance_transform->begin());
}

void kernel_upload_sensor_silhouette_wrist_distance_transform(int * sensor_silhouette_wrist_distance_transform) {
	thrust::copy(sensor_silhouette_wrist_distance_transform, sensor_silhouette_wrist_distance_transform + H_width * H_height, _sensor_silhouette_wrist_distance_transform->begin());
}

void kernel_upload_sensor_outline_distance_transform(int * sensor_outline_distance_transform) {
	thrust::copy(sensor_outline_distance_transform, sensor_outline_distance_transform + H_width * H_height, _sensor_outline_distance_transform->begin());
}

void kernel_upload_data_energy_data(int * sensor_indicator, int num_sensor_points) {
	thrust::copy(sensor_indicator, sensor_indicator + num_sensor_points, _sensor_indicator->begin());
	cudaMemcpyToSymbol(_num_sensor_points, &num_sensor_points, sizeof(int));
}

void kernel_upload_silhouette_energy_data(int * rendered_indicator, int num_rendered_points) {
	thrust::copy(rendered_indicator, rendered_indicator + num_rendered_points, _rendered_pixels->begin());
	cudaMemcpyToSymbol(_num_rendered_points, &num_rendered_points, sizeof(int));
}

void kernel_upload_outline_energy_data(int * rendered_pixels, float * rendered_points, float * rendered_normals, int * rendered_block_ids, int num_rendered_points) {
	thrust::copy(rendered_pixels, rendered_pixels + num_rendered_points, _rendered_pixels->begin());
	thrust::copy(rendered_points, rendered_points + 3 * num_rendered_points, _rendered_points->begin());
	if (settings->fit2D_outline_enable) thrust::copy(rendered_normals, rendered_normals + 2 * num_rendered_points, _rendered_normals->begin());
	thrust::copy(rendered_block_ids, rendered_block_ids + num_rendered_points, _rendered_block_ids->begin());
	cudaMemcpyToSymbol(_num_rendered_points, &num_rendered_points, sizeof(int));
}

void kernel_upload_kinematic(const std::vector<ThetaInfo>& host_pointer_theta_infos, const std::vector<KinematicChain>& host_pointer_kinematic_chains,
	const std::vector<BetaInfo>& host_pointer_beta_infos, const std::vector<ShapeChain>& host_pointer_shape_chains)
{

	if (host_pointer_beta_infos.size() > 0) {
		if (device_pointer_beta_infos.size() != host_pointer_beta_infos.size()) device_pointer_beta_infos.resize(host_pointer_beta_infos.size());
		thrust::copy(host_pointer_beta_infos.begin(), host_pointer_beta_infos.end(), device_pointer_beta_infos.begin());
	}

	if (host_pointer_shape_chains.size() > 0) {
		if (device_pointer_shape_chains.size() != host_pointer_shape_chains.size()) device_pointer_shape_chains.resize(host_pointer_shape_chains.size());
		thrust::copy(host_pointer_shape_chains.begin(), host_pointer_shape_chains.end(), device_pointer_shape_chains.begin());
	}

	if (device_pointer_theta_infos.size() != host_pointer_theta_infos.size()) device_pointer_theta_infos.resize(host_pointer_theta_infos.size());
	thrust::copy(host_pointer_theta_infos.begin(), host_pointer_theta_infos.end(), device_pointer_theta_infos.begin());

	if (device_pointer_kinematic_chains.size() != host_pointer_kinematic_chains.size()) device_pointer_kinematic_chains.resize(host_pointer_kinematic_chains.size());	
	thrust::copy(host_pointer_kinematic_chains.begin(), host_pointer_kinematic_chains.end(), device_pointer_kinematic_chains.begin());
}

void kernel_upload_model(int d, int num_parameters, int num_centers, int num_blocks, int num_outlines, int num_tangent_fields, int num_outline_fields, 
	const float * host_pointer_centers, const float * host_pointer_radii, const int * host_pointer_blocks,
	const float * host_pointer_tangent_points, const float * host_pointer_outline, const int * host_pointer_blockid_to_pose_unit_id_map, const int * host_pointer_blockid_to_shape_unit_id_map, bool calibrate) {

	/*printf("D = %d\n", d);
	printf("num_centers = %d\n", num_centers);
	printf("num_blocks = %d\n", num_blocks);
	printf("num_outlines = %d\n", num_outlines);
	printf("num_tangent_fields = %d\n", num_tangent_fields);
	printf("num_outline_fields = %d\n", num_outline_fields);*/

	/*printf("\n CENTERS: \n");
	for (size_t i = 0; i < NUM_CENTERS; i++) {
		printf("%f, %f, %f\n", host_pointer_centers[d * i], host_pointer_centers[d * i + 1], host_pointer_centers[d * i + 2]);
	}
	printf("\n RADII: \n");
	for (size_t i = 0; i < NUM_CENTERS; i++) {
		printf("%f\n", host_pointer_radii[i]);
	}
	printf("\n BLOCKS: \n");
	for (size_t i = 0; i < num_blocks; i++) {
		printf("%d, %d, %d\n", host_pointer_blocks[d * i], host_pointer_blocks[d * i + 1], host_pointer_blocks[d * i + 2]);
	}*/

	cudax::host_calibrate = calibrate;
	cudaMemcpyToSymbol(device_calibrate, &calibrate, sizeof(bool));
	if (cudax::_num_parameters != num_parameters) { // if calibration type was changed
		cudax::_num_parameters = num_parameters;
		cudaMemcpyToSymbol(NUM_PARAMETERS, &num_parameters, sizeof(int));
		JtJ->resize(num_parameters * num_parameters);
		JtF->resize(num_parameters);
	}

	cudaMemcpyToSymbol(NUM_OUTLINES, &num_outlines, sizeof(int));
	thrust::copy(host_pointer_centers, host_pointer_centers + d * num_centers, device_pointer_centers->begin());
	thrust::copy(host_pointer_radii, host_pointer_radii + num_centers, device_pointer_radii->begin());
	thrust::copy(host_pointer_blocks, host_pointer_blocks + d * num_blocks, device_pointer_blocks->begin());
	thrust::copy(host_pointer_tangent_points, host_pointer_tangent_points + d * num_tangent_fields * num_blocks, device_pointer_tangent_points->begin());
	thrust::copy(host_pointer_outline, host_pointer_outline + d * num_outline_fields * num_outlines, device_pointer_outline->begin());
	thrust::copy(host_pointer_blockid_to_pose_unit_id_map, host_pointer_blockid_to_pose_unit_id_map + num_blocks, device_pointer_blockid_to_pose_unit_id_map->begin());
	if (host_calibrate)
		thrust::copy(host_pointer_blockid_to_shape_unit_id_map, host_pointer_blockid_to_shape_unit_id_map + num_blocks, device_pointer_blockid_to_shape_unit_id_map->begin());
}

