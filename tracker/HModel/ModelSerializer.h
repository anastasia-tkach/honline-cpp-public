#pragma once

class Model;

class ModelSerializer {
	
	void serialize_centers();

	void serialize_radii();

	void serialize_blocks();

	void serialize_tangent_points();

	void serialize_outline();

	void serialize_blockid_to_pose_unit_id_map();

	void serialize_blockid_to_shape_unit_id_map();

	void serialize_theta_infos();

	void serialize_beta_infos();

	void serialize_kinematic_chains();

	void serialize_shape_chains();

public:
	Model * model;

	ModelSerializer(Model * _model);

	void serialize_model();
	
};
