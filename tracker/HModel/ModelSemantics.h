#pragma once

class Model;
class Cylinders;
class Skeleton;

class ModelSemantics {

	Model * model;

public:

	ModelSemantics(Model * _model);

	void setup_pose_units_and_kinematic_chains();

	void setup_shape_units_and_shape_chains();

	void setup_phalanges();

	void setup_jointid_to_phalangeid_map();

	void setup_outline();

	void setup_centers_name_to_id_map();

	void setup_attachments();

	void setup_jointid_to_centerid_map();

	void setup_blockid_to_phalangeid_map();

	void setup_blockid_to_pose_unit_id_map();

	void setup_blockid_to_shape_unit_id_map();

	void setup_calibration_type_to_num_betas_map();

	void setup_calibration_type_to_beta_template_map();

	void setup_pose_dofs();

	void setup_thetas_limits();

	void setup_shape_dofs();

	void setup_betas_limits();

	void setup_semantic_limits();

};
