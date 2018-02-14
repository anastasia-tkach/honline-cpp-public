#include "ModelSerializer.h"
#include "Model.h"

void ModelSerializer::serialize_centers() {
	if (!model->host_pointer_centers)
		model->host_pointer_centers = new float[d * model->centers.size()];
	for (size_t i = 0; i < model->centers.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			model->host_pointer_centers[d * i + j] = model->centers[i][j];
		}
	}
}

void ModelSerializer::serialize_radii() {
	if (!model->host_pointer_radii)
		model->host_pointer_radii = new float[model->radii.size()];
	for (size_t i = 0; i < model->radii.size(); i++) {
		model->host_pointer_radii[i] = model->radii[i];
	}
}

void ModelSerializer::serialize_blocks() {
	if (!model->host_pointer_blocks)
		model->host_pointer_blocks = new int[d * model->blocks.size()];
	for (size_t i = 0; i < model->blocks.size(); i++) {
		for (size_t j = 0; j < d; j++) {
			model->host_pointer_blocks[d * i + j] = model->blocks[i][j];
		}
	}
}

void ModelSerializer::serialize_tangent_points() {
	if (!model->host_pointer_tangent_points)
		model->host_pointer_tangent_points = new float[d * model->num_tangent_fields * model->tangent_points.size()];
	for (size_t i = 0; i < model->tangent_points.size(); i++) {
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + j] = model->tangent_points[i].v1[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 3 + j] = model->tangent_points[i].v2[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 6 + j] = model->tangent_points[i].v3[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 9 + j] = model->tangent_points[i].n[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 12 + j] = model->tangent_points[i].u1[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 15 + j] = model->tangent_points[i].u2[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 18 + j] = model->tangent_points[i].u3[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_tangent_points[d * model->num_tangent_fields * i + 21 + j] = model->tangent_points[i].m[j];
	}
}

void ModelSerializer::serialize_outline() {
	if (!model->host_pointer_outline)
		model->host_pointer_outline = new float[d * model->num_outline_fields * model->max_num_outlines];
	for (size_t i = 0; i < model->outline_finder.outline3D.size(); i++) {
		for (size_t j = 0; j < d; j++)
			model->host_pointer_outline[d * model->num_outline_fields * i + j] = model->outline_finder.outline3D[i].start[j];
		for (size_t j = 0; j < d; j++)
			model->host_pointer_outline[d * model->num_outline_fields * i + 3 + j] = model->outline_finder.outline3D[i].end[j];
		for (size_t j = 0; j < 2; j++)
			model->host_pointer_outline[d * model->num_outline_fields * i + 6 + j] = model->outline_finder.outline3D[i].indices[j];
		model->host_pointer_outline[d * model->num_outline_fields * i + 8] = model->outline_finder.outline3D[i].block;
	}
}

void ModelSerializer::serialize_blockid_to_pose_unit_id_map() {
	if (!model->host_pointer_blockid_to_pose_unit_id_map)
		model->host_pointer_blockid_to_pose_unit_id_map = new int[model->blockid_to_pose_unit_id_map.size()];
	for (size_t i = 0; i < model->blockid_to_pose_unit_id_map.size(); i++) {
		model->host_pointer_blockid_to_pose_unit_id_map[i] = model->blockid_to_pose_unit_id_map[i];
	}
}

void ModelSerializer::serialize_blockid_to_shape_unit_id_map() {
	if (model->num_betas == 0) return;
	if (!model->host_pointer_blockid_to_shape_unit_id_map)
		model->host_pointer_blockid_to_shape_unit_id_map = new int[model->blockid_to_shape_unit_id_map.size()];
	for (size_t i = 0; i < model->blockid_to_shape_unit_id_map.size(); i++) {
		model->host_pointer_blockid_to_shape_unit_id_map[i] = model->blockid_to_shape_unit_id_map[i];
	}
}

void ModelSerializer::serialize_theta_infos() {
	model->theta_infos.resize(num_thetas);
	for (size_t i = 0; i < model->pose_dofs.size(); i++) {
		Eigen::Map<Vector3>(model->theta_infos[i].axis) = model->pose_dofs[i].axis;
		model->theta_infos[i].type = model->pose_dofs[i].type;
		model->theta_infos[i].jacobian_column = i;
		if (model->pose_dofs[i].phalange_id != -1) {
			switch (model->pose_dofs[i].type) {
			case TRANSLATION_AXIS: {
				Matrix3 mat = Matrix3::Identity(3, 3);
				Eigen::Map<Matrix4>(model->theta_infos[i].mat) = Transform3f(mat).matrix();
				break;
			}
			case ROTATION_AXIS:
			default: {
				Matrix4 mat = model->phalanges[model->pose_dofs[i].phalange_id].global;
				Eigen::Map<Matrix4>(model->theta_infos[i].mat) = Transform3f(mat).matrix();
				break;
			}
			}
		}
	}
}

void ModelSerializer::serialize_beta_infos() {
	if (model->num_betas == 0) return;
	if (model->beta_infos.size() != model->num_betas) model->beta_infos.resize(model->num_betas);
	for (size_t i = 0; i < model->num_betas; i++) {
		int phalange_id = model->shape_dofs[i].phalange_id;
		BetaType beta_type = model->shape_dofs[i].type;
		Vector3 axis;
		Matrix4 mat;
		if (beta_type == PHALANGE_LENGTH) {
			size_t child_phalange_id = phalange_id + 1;
			axis = model->phalanges[child_phalange_id].init_local.block(0, 3, 3, 1);
			mat = model->phalanges[phalange_id].global;
		}
		if (beta_type == TOP_PHALANGE_LENGTH) {
			axis = model->phalanges[phalange_id].offsets[0].cast<float>();
			mat = model->phalanges[phalange_id].global;
		}
		if (beta_type == FINGER_BASE_X) {
			axis = Vector3(1, 0, 0);
			// we need the trasformation of palm phalange
			size_t parent_phalange_id = model->phalanges[phalange_id].parent_id; 
			mat = model->phalanges[parent_phalange_id].global;
		}
		if (beta_type == FINGER_BASE_Y) {
			axis = Vector3(0, 1, 0);
			// we need the trasformation of palm phalange
			size_t parent_phalange_id = model->phalanges[phalange_id].parent_id;
			mat = model->phalanges[parent_phalange_id].global;
		}
		if (beta_type == FINGER_BASE_Z) {
			axis = Vector3(0, 0, 1);
			// we need the trasformation of palm phalange
			size_t parent_phalange_id = model->phalanges[phalange_id].parent_id;
			mat = model->phalanges[parent_phalange_id].global;
		}
		if (beta_type == PALM_CENTER_X) {
			axis = Vector3(1, 0, 0);
			mat = model->phalanges[phalange_id].global;
		}
		if (beta_type == PALM_CENTER_Y) {
			axis = Vector3(0, 1, 0);
			mat = model->phalanges[phalange_id].global;
		}
		if (beta_type == RADIUS) {
			//axis = Vector3(0, 1, 0);
			// TODO: figure out what is axis
			mat = model->phalanges[phalange_id].global;
		}
		if (beta_type == FINGER_TOP_Y) {
			axis = Vector3(0, 1, 0);
			mat = model->phalanges[phalange_id].global;
		}
		if (beta_type == FINGER_TOP_Z) {
			axis = Vector3(0, 0, 1);
			mat = model->phalanges[phalange_id].global;
		}
		model->beta_infos[i].beta = model->beta[i];
		Eigen::Map<Vector3>(model->beta_infos[i].axis) = axis / axis.norm();
		Eigen::Map<Matrix4>(model->beta_infos[i].mat) = Transform3f(mat).matrix();

		model->beta_infos[i].jacobian_column = num_thetas + i;
		model->beta_infos[i].type = beta_type;
		model->beta_infos[i].center_id = model->shape_dofs[i].center_id;
		model->beta_infos[i].top_center_id = model->shape_dofs[i].top_center_id;
		model->beta_infos[i].attachment_center_id = model->shape_dofs[i].attachment_center_id;
	}
}

void ModelSerializer::serialize_kinematic_chains() {
	if (model->host_pointer_kinematic_chains.size() != model->pose_units.size()) model->host_pointer_kinematic_chains.resize(model->pose_units.size());
	for (size_t i = 0; i < model->pose_units.size(); i++) {
		model->host_pointer_kinematic_chains[i].set_data(model->pose_units[i].kinematic_chain);
	}
}

void ModelSerializer::serialize_shape_chains() {
	if (model->num_betas == 0) return;
	if (model->host_pointer_shape_chains.size() != model->shape_units.size()) model->host_pointer_shape_chains.resize(model->shape_units.size());
	for (size_t i = 0; i < model->shape_units.size(); i++) {
		model->host_pointer_shape_chains[i].set_data(model->shape_units[i].shape_chain);
	}
}

ModelSerializer::ModelSerializer(Model * _model) : model(_model) {}

void ModelSerializer::serialize_model() {
	serialize_centers();
	serialize_radii();
	serialize_blocks();
	serialize_tangent_points();
	serialize_outline();
	serialize_blockid_to_pose_unit_id_map();
	serialize_blockid_to_shape_unit_id_map();
	serialize_theta_infos();
	serialize_beta_infos();
	serialize_kinematic_chains();
	serialize_shape_chains();
}

