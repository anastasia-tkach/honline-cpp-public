#include "ModelSemantics.h";
#include "Model.h"

ModelSemantics::ModelSemantics(Model * _model) : model(_model) {}

void ModelSemantics::setup_pose_units_and_kinematic_chains() {

	model->num_pose_units = 17;
	model->pose_units.resize(model->num_pose_units);

	model->pose_units[0].set("hand", { 0, 1, 2, 3, 4, 5 });

	model->pose_units[1].set("thumb1", { 9, 10, 29, 0, 1, 2, 3, 4, 5 });
	model->pose_units[2].set("thumb2", { 11, 9, 10, 29, 0, 1, 2, 3, 4, 5 });
	model->pose_units[3].set("thumb3", { 12, 11, 9, 10, 29, 0, 1, 2, 3, 4, 5 });

	model->pose_units[4].set("pinky1", { 25, 26, 33, 0, 1, 2, 3, 4, 5 });
	model->pose_units[5].set("pinky2", { 27, 25, 26, 33, 0, 1, 2, 3, 4, 5 });
	model->pose_units[6].set("pinky3", { 28, 27, 25, 26, 33, 0, 1, 2, 3, 4, 5 });

	model->pose_units[7].set("ring1", { 21, 22, 32, 0, 1, 2, 3, 4, 5 });
	model->pose_units[8].set("ring2", { 23, 21, 22, 32, 0, 1, 2, 3, 4, 5 });
	model->pose_units[9].set("ring3", { 24, 23, 21, 22, 32, 0, 1, 2, 3, 4, 5 });

	model->pose_units[10].set("middle1", { 17, 18, 31, 0, 1, 2, 3, 4, 5 });
	model->pose_units[11].set("middle2", { 19, 17, 18, 31, 0, 1, 2, 3, 4, 5 });
	model->pose_units[12].set("middle3", { 20, 19, 17, 18, 31, 0, 1, 2, 3, 4, 5 });

	model->pose_units[13].set("index1", { 13, 14, 30, 0, 1, 2, 3, 4, 5 });
	model->pose_units[14].set("index2", { 15, 13, 14, 30, 0, 1, 2, 3, 4, 5 });
	model->pose_units[15].set("index3", { 16, 15, 13, 14, 30, 0, 1, 2, 3, 4, 5 });

	model->pose_units[16].set("wrist", { 7, 8, 0, 1, 2, 3, 4, 5 });
}

void ModelSemantics::setup_shape_units_and_shape_chains() {

	if (model->calibration_type == FINGERS_PARTIAL) {
		model->num_shape_units = 15;
		model->shape_units.resize(model->num_shape_units);
		// thumb
		model->shape_units[0].set("thumb1", { 0 });
		model->shape_units[1].set("thumb2", { 0, 1 });
		model->shape_units[2].set("thumb3", { 0, 1, 2 });
		// index
		model->shape_units[3].set("index1", { 3 });
		model->shape_units[4].set("index2", { 3, 4 });
		model->shape_units[5].set("index3", { 3, 4, 5 });
		// middle
		model->shape_units[6].set("middle1", { 6 });
		model->shape_units[7].set("middle2", { 6, 7 });
		model->shape_units[8].set("middle3", { 6, 7, 8 });
		// ring
		model->shape_units[9].set("ring1", { 9 });
		model->shape_units[10].set("ring2", { 9, 10 });
		model->shape_units[11].set("ring3", { 9, 10, 11 });
		// pinky
		model->shape_units[12].set("pinky1", { 12 });
		model->shape_units[13].set("pinky2", { 12, 13 });
		model->shape_units[14].set("pinky3", { 12, 13, 14 });
	}

	if (model->calibration_type == FINGERS || model->calibration_type == FINGERS_AND_PALM) {
		model->num_shape_units = 15;
		model->shape_units.resize(model->num_shape_units);
		// thumb
		model->shape_units[0].set("thumb1", { 15, 16, 17, 0 });
		model->shape_units[1].set("thumb2", { 15, 16, 17, 0, 1 });
		model->shape_units[2].set("thumb3", { 15, 16, 17, 0, 1, 2 });
		// index
		model->shape_units[3].set("index1", { 18, 19, 20, 3 });
		model->shape_units[4].set("index2", { 18, 19, 20, 3, 4 });
		model->shape_units[5].set("index3", { 18, 19, 20, 3, 4, 5 });
		// middle
		model->shape_units[6].set("middle1", { 21, 22, 23, 6 });
		model->shape_units[7].set("middle2", { 21, 22, 23, 6, 7 });
		model->shape_units[8].set("middle3", { 21, 22, 23, 6, 7, 8 });
		// ring
		model->shape_units[9].set("ring1", { 24, 25, 26, 9 });
		model->shape_units[10].set("ring2", { 24, 25, 26, 9, 10 });
		model->shape_units[11].set("ring3", { 24, 25, 26, 9, 10, 11 });
		// pinky
		model->shape_units[12].set("pinky1", { 27, 28, 29, 12 });
		model->shape_units[13].set("pinky2", { 27, 28, 29, 12, 13 });
		model->shape_units[14].set("pinky3", { 27, 28, 29, 12, 13, 14 });

		if (model->calibration_type == FINGERS_AND_PALM) {
			model->num_shape_units = 28;
			model->shape_units.resize(model->num_shape_units);

			// palm
			model->shape_units[15].set("palm_top_left", { 30, 31, 32, 33, 34, 35});
			model->shape_units[16].set("palm_bottom_left", { 30, 31, 32, 33, 34, 35});
			model->shape_units[17].set("palm_middle", { 30, 31, 32, 33, 21});
			model->shape_units[18].set("palm_bottom_right", { 30, 31, 32, 33, 36, 37 });
			model->shape_units[19].set("palm_top_right", { 30, 31, 32, 33, 36, 37 });
			// membranes
			model->shape_units[20].set("membrane_index_middle_top", { 3, 6, 18, 19, 20, 21, 22, 23, 30, 31 });
			model->shape_units[21].set("membrane_index_middle_bottom", { 6, 21, 22, 23, 30, 31, 32, 33 });
			model->shape_units[22].set("membrane_middle_ring_top", { 6, 9, 21, 22, 23, 24, 25, 26, 30, 31, 32, 33 });
			model->shape_units[23].set("membrane_middle_ring_bottom", { 6, 21, 22, 23, 30, 31, 32, 33 });
			model->shape_units[24].set("membrane_ring_pinky_top", { 9, 12, 24, 25, 26, 27, 28, 29, 32, 33 });
			model->shape_units[25].set("membrane_ring_pinky_bottom", { 9, 24, 25, 26, 30, 31, 32, 33 });
			model->shape_units[26].set("membrane_thumb", { 0, 15, 16, 17, 38, 39, 42 });
			// wrist
			model->shape_units[27].set("wrist", { 40, 41, 45 });
		}
	}

	if (model->calibration_type == FULL) {
		model->num_shape_units = 29;
		model->shape_units.resize(model->num_shape_units);
		size_t c = model->calibration_type_to_num_betas_map[FINGERS_AND_PALM];
		// thumb
		model->shape_units[0].set("thumb1", { 15, 16, 17, 0, 0 + c, 1 + c });
		model->shape_units[1].set("thumb2", { 15, 16, 17, 0, 1, 1 + c, 2 + c });
		model->shape_units[2].set("thumb3", { 15, 16, 17, 0, 1, 2, 2 + c, 3 + c });
		// index
		model->shape_units[3].set("index1", { 18, 19, 20, 3, 4 + c, 5 + c });
		model->shape_units[4].set("index2", { 18, 19, 20, 3, 4, 5 + c, 6 + c });
		model->shape_units[5].set("index3", { 18, 19, 20, 3, 4, 5, 6 + c, 7 + c });
		// middle
		model->shape_units[6].set("middle1", { 21, 22, 23, 6, 8 + c, 9 + c });
		model->shape_units[7].set("middle2", { 21, 22, 23, 6, 7, 9 + c, 10 + c });
		model->shape_units[8].set("middle3", { 21, 22, 23, 6, 7, 8, 10 + c, 11 + c });
		// ring
		model->shape_units[9].set("ring1", { 24, 25, 26, 9, 12 + c, 13 + c });
		model->shape_units[10].set("ring2", { 24, 25, 26, 9, 10, 13 + c, 14 + c });
		model->shape_units[11].set("ring3", { 24, 25, 26, 9, 10, 11, 14 + c, 15 + c });
		// pinky
		model->shape_units[12].set("pinky1", { 27, 28, 29, 12, 16 + c, 17 + c });
		model->shape_units[13].set("pinky2", { 27, 28, 29, 12, 13, 17 + c, 18 + c });
		model->shape_units[14].set("pinky3", { 27, 28, 29, 12, 13, 14, 18 + c, 19 + c });
		// palm
		model->shape_units[15].set("palm_top_left", { 30, 31, 32, 33, 34, 35, 20 + c, 21 + c, 24 + c });
		model->shape_units[16].set("palm_bottom_left", { 30, 31, 32, 33, 34, 35, 21 + c, 24 + c, 25 + c });
		model->shape_units[17].set("palm_middle", { 30, 31, 32, 33, 21 + c, 22 + c, 25 + c });
		model->shape_units[18].set("palm_bottom_right", { 30, 31, 32, 33, 36, 37, 22 + c, 25 + c, 26 + c });
		model->shape_units[19].set("palm_top_right", { 30, 31, 32, 33, 36, 37, 22 + c, 23 + c, 26 + c });
		// membranes
		model->shape_units[20].set("membrane_index_middle_top", { 3, 6, 18, 19, 20, 21, 22, 23, 30, 31, 20 + c });
		model->shape_units[21].set("membrane_index_middle_bottom", { 6, 21, 22, 23, 30, 31, 32, 33, 20 + c, 21 + c });
		model->shape_units[22].set("membrane_middle_ring_top", { 6, 9, 21, 22, 23, 24, 25, 26, 30, 31, 32, 33, 22 + c });
		model->shape_units[23].set("membrane_middle_ring_bottom", { 6, 21, 22, 23, 30, 31, 32, 33, 21 + c, 22 + c });
		model->shape_units[24].set("membrane_ring_pinky_top", { 9, 12, 24, 25, 26, 27, 28, 29, 32, 33, 23 + c });
		model->shape_units[25].set("membrane_ring_pinky_bottom", { 9, 24, 25, 26, 30, 31, 32, 33, 22 + c, 23 + c });
		model->shape_units[26].set("membrane_thumb", { 0, 15, 16, 17, 38, 39, 42, 0 + c });
		// wrist
		model->shape_units[27].set("wrist", { 40, 41, 45, c + 27, c + 28 });
		// thumb top
		model->shape_units[28].set("thumb4", { 15, 16, 17, 0, 1, 2, 43, 44, 3 + c });
	}

	bool print = false;
	if (print) {
		for (size_t i = 0; i < model->shape_units.size(); i++) {
			cout << i << ": (";
			for (size_t j = 0; j < model->shape_units[i].shape_chain.size(); j++) {
				cout << model->shape_units[i].shape_chain[j] << " ";
			}
			cout << ")" << endl;
		}
	}
}

void ModelSemantics::setup_jointid_to_phalangeid_map() {
	for (size_t i = 0; i < num_phalanges - 1; i++)
		model->jointid_to_phalangeid_map[model->phalanges[i].segment_id] = i;

	model->jointid_to_phalangeid_map[7] = 3;
	model->jointid_to_phalangeid_map[11] = 6;
	model->jointid_to_phalangeid_map[15] = 9;
	model->jointid_to_phalangeid_map[19] = 12;
	model->jointid_to_phalangeid_map[23] = 15;
}

void ModelSemantics::setup_outline() {

	std::vector<int> finger_block_indices;
	finger_block_indices.push_back(0);
	finger_block_indices.push_back(1);
	model->fingers_block_indices.push_back(finger_block_indices);
	finger_block_indices.clear();
	finger_block_indices.push_back(3);
	finger_block_indices.push_back(4);
	model->fingers_block_indices.push_back(finger_block_indices);
	finger_block_indices.clear();
	finger_block_indices.push_back(6);
	finger_block_indices.push_back(7);
	model->fingers_block_indices.push_back(finger_block_indices);
	finger_block_indices.clear();
	finger_block_indices.push_back(9);
	finger_block_indices.push_back(10);
	model->fingers_block_indices.push_back(finger_block_indices);
	finger_block_indices.clear();
	finger_block_indices.push_back(12);
	finger_block_indices.push_back(13);
	finger_block_indices.push_back(14);
	model->fingers_block_indices.push_back(finger_block_indices);

	model->fingers_base_centers.push_back(model->centers_name_to_id_map["pinky_bottom"]);
	model->fingers_base_centers.push_back(model->centers_name_to_id_map["ring_bottom"]);
	model->fingers_base_centers.push_back(model->centers_name_to_id_map["middle_bottom"]);
	model->fingers_base_centers.push_back(model->centers_name_to_id_map["index_bottom"]);
	model->fingers_base_centers.push_back(model->centers_name_to_id_map["thumb_bottom"]);

	model->crop_indices_thumb.push_back(glm::ivec2(model->centers_name_to_id_map["thumb_bottom"], model->centers_name_to_id_map["thumb_middle"]));
	model->limit_indices_thumb.push_back(glm::ivec2(model->centers_name_to_id_map["thumb_bottom"], model->centers_name_to_id_map["thumb_membrane_left"]));

	model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["pinky_base"], model->centers_name_to_id_map["pinky_bottom"]));
	model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["ring_base"], model->centers_name_to_id_map["ring_bottom"]));
	model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["middle_base"], model->centers_name_to_id_map["middle_bottom"]));
	model->crop_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["index_base"], model->centers_name_to_id_map["index_bottom"]));

	model->limit_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["pinky_membrane"], model->centers_name_to_id_map["ring_membrane"]));
	model->limit_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["ring_membrane"], model->centers_name_to_id_map["middle_membrane"]));
	model->limit_indices_fingers.push_back(glm::ivec2(model->centers_name_to_id_map["middle_membrane"], model->centers_name_to_id_map["index_membrane"]));

	model->adjuct_block_indices_fingers.push_back(2);
	model->adjuct_block_indices_fingers.push_back(5);
	model->adjuct_block_indices_fingers.push_back(8);
	model->adjuct_block_indices_fingers.push_back(11);

	model->fingers_block_indices[4][2] = 27;

	/*model->palm_block_indices.push_back(14);
	model->palm_block_indices.push_back(15);
	model->palm_block_indices.push_back(16);
	model->palm_block_indices.push_back(17);
	model->palm_block_indices.push_back(18);
	model->palm_block_indices.push_back(19);
	model->palm_block_indices.push_back(20);
	model->palm_block_indices.push_back(21);
	model->palm_block_indices.push_back(22);
	model->palm_block_indices.push_back(23);
	model->palm_block_indices.push_back(24);
	model->palm_block_indices.push_back(25);
	model->palm_block_indices.push_back(26);
	model->palm_block_indices.push_back(2);
	model->palm_block_indices.push_back(5);
	model->palm_block_indices.push_back(8);
	model->palm_block_indices.push_back(11);
	model->palm_block_indices.push_back(28);
	model->palm_block_indices.push_back(29);*/

	model->palm_block_indices = { 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 2, 5, 8, 11, 28, 29, 30 };	
	model->wrist_block_indices = {};

	if (model->fit_wrist_separately) {
		model->palm_block_indices = { 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 2, 5, 8, 11, 30 };
		model->wrist_block_indices = { 28, 29 };
	}

}

void ModelSemantics::setup_centers_name_to_id_map() {

	model->centers_name_to_id_map = std::map<string, size_t>();

	model->centers_name_to_id_map["index_base"] = 15;
	model->centers_name_to_id_map["index_bottom"] = 14;
	model->centers_name_to_id_map["index_membrane"] = 31;
	model->centers_name_to_id_map["index_middle"] = 13;
	model->centers_name_to_id_map["index_top"] = 12;
	model->centers_name_to_id_map["middle_base"] = 11;
	model->centers_name_to_id_map["middle_bottom"] = 10;
	model->centers_name_to_id_map["middle_membrane"] = 30;
	model->centers_name_to_id_map["middle_middle"] = 9;
	model->centers_name_to_id_map["middle_top"] = 8;
	model->centers_name_to_id_map["palm_back"] = 25;
	model->centers_name_to_id_map["palm_index"] = 23;
	model->centers_name_to_id_map["palm_left"] = 27;
	model->centers_name_to_id_map["palm_middle"] = 22;
	model->centers_name_to_id_map["palm_pinky"] = 20;
	model->centers_name_to_id_map["palm_right"] = 26;
	model->centers_name_to_id_map["palm_ring"] = 21;
	model->centers_name_to_id_map["palm_thumb"] = 24;
	model->centers_name_to_id_map["pinky_base"] = 3;
	model->centers_name_to_id_map["pinky_bottom"] = 2;
	model->centers_name_to_id_map["pinky_membrane"] = 28;
	model->centers_name_to_id_map["pinky_middle"] = 1;
	model->centers_name_to_id_map["pinky_top"] = 0;
	model->centers_name_to_id_map["ring_base"] = 7;
	model->centers_name_to_id_map["ring_bottom"] = 6;
	model->centers_name_to_id_map["ring_membrane"] = 29;
	model->centers_name_to_id_map["ring_middle"] = 5;
	model->centers_name_to_id_map["ring_top"] = 4;
	model->centers_name_to_id_map["thumb_additional"] = 32;
	model->centers_name_to_id_map["thumb_base"] = 19;
	model->centers_name_to_id_map["thumb_bottom"] = 18;
	model->centers_name_to_id_map["thumb_membrane_left"] = 33;
	model->centers_name_to_id_map["thumb_middle"] = 17;
	model->centers_name_to_id_map["thumb_top"] = 16;

	model->centers_name_to_id_map["wrist_bottom_left"] = 36;
	model->centers_name_to_id_map["wrist_bottom_right"] = 37;
	model->centers_name_to_id_map["wrist_top_left"] = 34;
	model->centers_name_to_id_map["wrist_top_right"] = 35;

	model->centers_name_to_id_map["thumb_membrane_middle"] = 38;
}

void ModelSemantics::setup_attachments() {
	{
		// Hand
		model->phalanges[0].center_id = model->centers_name_to_id_map["palm_back"];
		model->phalanges[0].attachments.resize(8);

		model->phalanges[0].attachments[0] = model->centers_name_to_id_map["palm_back"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_back"]] = 0;

		model->phalanges[0].attachments[1] = model->centers_name_to_id_map["palm_index"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_index"]] = 1;

		model->phalanges[0].attachments[2] = model->centers_name_to_id_map["palm_left"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_left"]] = 2;

		model->phalanges[0].attachments[3] = model->centers_name_to_id_map["palm_middle"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_middle"]] = 3;

		model->phalanges[0].attachments[4] = model->centers_name_to_id_map["palm_pinky"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_pinky"]] = 4;

		model->phalanges[0].attachments[5] = model->centers_name_to_id_map["palm_right"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_right"]] = 5;

		model->phalanges[0].attachments[6] = model->centers_name_to_id_map["palm_ring"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_ring"]] = 6;

		model->phalanges[0].attachments[7] = model->centers_name_to_id_map["palm_thumb"];
		model->centerid_to_attachment_id_map[model->centers_name_to_id_map["palm_thumb"]] = 7;

		// HandThumb1
		model->phalanges[1].center_id = model->centers_name_to_id_map["thumb_base"];

		// HandThumb2
		model->phalanges[2].center_id = model->centers_name_to_id_map["thumb_bottom"];
		model->phalanges[2].attachments.push_back(model->centers_name_to_id_map["thumb_membrane_left"]);

		// HandThumb3
		model->phalanges[3].center_id = model->centers_name_to_id_map["thumb_middle"];
		model->phalanges[3].attachments.push_back(model->centers_name_to_id_map["thumb_top"]);
		model->phalanges[3].attachments.push_back(model->centers_name_to_id_map["thumb_additional"]);

		// HandPinky1
		model->phalanges[4].center_id = model->centers_name_to_id_map["pinky_base"];
		model->phalanges[4].attachments.push_back(model->centers_name_to_id_map["pinky_membrane"]);

		// HandPinky2
		model->phalanges[5].center_id = model->centers_name_to_id_map["pinky_bottom"];

		// HandPinky3
		model->phalanges[6].center_id = model->centers_name_to_id_map["pinky_middle"];
		model->phalanges[6].attachments.push_back(model->centers_name_to_id_map["pinky_top"]);

		// HandRing1
		model->phalanges[7].center_id = model->centers_name_to_id_map["ring_base"];
		model->phalanges[7].attachments.push_back(model->centers_name_to_id_map["ring_membrane"]);

		// HandRing2
		model->phalanges[8].center_id = model->centers_name_to_id_map["ring_bottom"];

		// HandRing3
		model->phalanges[9].center_id = model->centers_name_to_id_map["ring_middle"];
		model->phalanges[9].attachments.push_back(model->centers_name_to_id_map["ring_top"]);

		// HandMiddle1
		model->phalanges[10].center_id = model->centers_name_to_id_map["middle_base"];
		model->phalanges[10].attachments.push_back(model->centers_name_to_id_map["middle_membrane"]);

		// HandMiddle2
		model->phalanges[11].center_id = model->centers_name_to_id_map["middle_bottom"];

		// HandMiddle3
		model->phalanges[12].center_id = model->centers_name_to_id_map["middle_middle"];
		model->phalanges[12].attachments.push_back(model->centers_name_to_id_map["middle_top"]);

		// HandIndex1
		model->phalanges[13].center_id = model->centers_name_to_id_map["index_base"];
		model->phalanges[13].attachments.push_back(model->centers_name_to_id_map["index_membrane"]);

		// HandIndex2
		model->phalanges[14].center_id = model->centers_name_to_id_map["index_bottom"];

		// HandIndex3
		model->phalanges[15].center_id = model->centers_name_to_id_map["index_middle"];
		model->phalanges[15].attachments.push_back(model->centers_name_to_id_map["index_top"]);

		// Wrist
		model->phalanges[16].center_id = model->centers_name_to_id_map["palm_back"];
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_bottom_left"]);
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_bottom_right"]);
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_top_left"]);
		model->phalanges[16].attachments.push_back(model->centers_name_to_id_map["wrist_top_right"]);
	}

}

void ModelSemantics::setup_jointid_to_centerid_map() {
	model->jointid_to_centerid_map[0] = -1; // root
	model->jointid_to_centerid_map[1] = -1; // pose
	model->jointid_to_centerid_map[2] = -1; // scale
	model->jointid_to_centerid_map[3] = model->centers_name_to_id_map["palm_back"]; // Hand
	model->jointid_to_centerid_map[4] = model->centers_name_to_id_map["thumb_base"]; // HandThumb1
	model->jointid_to_centerid_map[5] = model->centers_name_to_id_map["thumb_bottom"]; // HandThumb2
	model->jointid_to_centerid_map[6] = model->centers_name_to_id_map["thumb_middle"]; // HandThumb3
	model->jointid_to_centerid_map[7] = model->centers_name_to_id_map["thumb_top"]; // HandThumb4
	model->jointid_to_centerid_map[8] = model->centers_name_to_id_map["pinky_base"]; // HandPinky1
	model->jointid_to_centerid_map[9] = model->centers_name_to_id_map["pinky_bottom"]; // HandPinky2
	model->jointid_to_centerid_map[10] = model->centers_name_to_id_map["pinky_middle"]; // HandPinky3
	model->jointid_to_centerid_map[11] = model->centers_name_to_id_map["pinky_top"]; // HandPinky4
	model->jointid_to_centerid_map[12] = model->centers_name_to_id_map["ring_base"]; // HandRing1
	model->jointid_to_centerid_map[13] = model->centers_name_to_id_map["ring_bottom"]; // HandRing2
	model->jointid_to_centerid_map[14] = model->centers_name_to_id_map["ring_middle"]; // HandRing3
	model->jointid_to_centerid_map[15] = model->centers_name_to_id_map["ring_top"]; // HandRing4
	model->jointid_to_centerid_map[16] = model->centers_name_to_id_map["middle_base"]; // HandMiddle1
	model->jointid_to_centerid_map[17] = model->centers_name_to_id_map["middle_bottom"]; // HandMiddle2
	model->jointid_to_centerid_map[18] = model->centers_name_to_id_map["middle_middle"]; // HandMiddle3
	model->jointid_to_centerid_map[19] = model->centers_name_to_id_map["middle_top"]; // HandMiddle4
	model->jointid_to_centerid_map[20] = model->centers_name_to_id_map["index_base"]; // HandIndex1
	model->jointid_to_centerid_map[21] = model->centers_name_to_id_map["index_bottom"]; // HandIndex2
	model->jointid_to_centerid_map[22] = model->centers_name_to_id_map["index_middle"]; // HandIndex3
	model->jointid_to_centerid_map[23] = model->centers_name_to_id_map["index_top"]; // HandIndex4
}

void ModelSemantics::setup_blockid_to_phalangeid_map() {
	model->blockid_to_phalangeid_map = vector<int>(model->num_blocks, 0);

	model->blockid_to_phalangeid_map[0] = 6; // pinky_middle, pinky_top
	model->blockid_to_phalangeid_map[1] = 5; // pinky_bottom, pinky_middle
	model->blockid_to_phalangeid_map[2] = 4; // pinky_base, pinky_bottom
	model->blockid_to_phalangeid_map[3] = 9; // ring_top, ring_middle
	model->blockid_to_phalangeid_map[4] = 8; // ring_bottom, ring_middle
	model->blockid_to_phalangeid_map[5] = 7; // ring_bottom, ring_base
	model->blockid_to_phalangeid_map[6] = 12; // middle_top, middle_middle
	model->blockid_to_phalangeid_map[7] = 11; // middle_bottom, middle_middle
	model->blockid_to_phalangeid_map[8] = 10; // middle_bottom, middle_base
	model->blockid_to_phalangeid_map[9] = 15; // index_middle, index_top
	model->blockid_to_phalangeid_map[10] = 14; // index_bottom, index_middle
	model->blockid_to_phalangeid_map[11] = 13; // index_base, index_bottom
	model->blockid_to_phalangeid_map[12] = 3; // thumb_top, thumb_middle
	model->blockid_to_phalangeid_map[13] = 2; // thumb_bottom, thumb_middle
	model->blockid_to_phalangeid_map[14] = 1; // thumb_base, thumb_bottom, thumb_fold
	model->blockid_to_phalangeid_map[26] = 0; // thumb_base, thumb_fold, palm_thumb	
	model->blockid_to_phalangeid_map[27] = 3; // thumb_top, thumb_additional

	model->blockid_to_phalangeid_map[15] = 0; // palm_right, palm_ring, palm_pinky
	model->blockid_to_phalangeid_map[16] = 0; // palm_back, palm_right, palm_ring
	model->blockid_to_phalangeid_map[17] = 0; // palm_back, palm_middle, palm_ring
	model->blockid_to_phalangeid_map[18] = 0; // palm_back, palm_left, palm_middle
	model->blockid_to_phalangeid_map[19] = 0; // palm_left, palm_index, palm_middle
	model->blockid_to_phalangeid_map[20] = 0; // palm_pinky, ring_membrane, pinky_membrane
	model->blockid_to_phalangeid_map[21] = 0; // palm_ring, palm_pinky, ring_membrane
	model->blockid_to_phalangeid_map[22] = 0; // palm_ring, middle_membrane, ring_membrane
	model->blockid_to_phalangeid_map[23] = 0; // palm_middle, palm_ring, middle_membrane
	model->blockid_to_phalangeid_map[24] = 0; // palm_index, palm_middle, middle_membrane
	model->blockid_to_phalangeid_map[25] = 0; // palm_index, middle_membrane, index_membrane

	model->blockid_to_phalangeid_map[28] = 16; // wrist_bottom_left, wrist_top_left, wrist_top_right
	model->blockid_to_phalangeid_map[29] = 16; // wrist_bottom_right, wrist_bottom_left, wrist_top_right	

	model->blockid_to_phalangeid_map[30] = 0; 	
}

void ModelSemantics::setup_blockid_to_pose_unit_id_map() {
	model->blockid_to_pose_unit_id_map = vector<int>(model->num_blocks, 0);

	model->blockid_to_pose_unit_id_map[0] = 6; // pinky_middle, pinky_top
	model->blockid_to_pose_unit_id_map[1] = 5; // pinky_bottom, pinky_middle
	model->blockid_to_pose_unit_id_map[2] = 4; // pinky_base, pinky_bottom
	model->blockid_to_pose_unit_id_map[3] = 9; // ring_top, ring_middle
	model->blockid_to_pose_unit_id_map[4] = 8; // ring_bottom, ring_middle
	model->blockid_to_pose_unit_id_map[5] = 7; // ring_bottom, ring_base
	model->blockid_to_pose_unit_id_map[6] = 12; // middle_top, middle_middle
	model->blockid_to_pose_unit_id_map[7] = 11; // middle_bottom, middle_middle
	model->blockid_to_pose_unit_id_map[8] = 10; // middle_bottom, middle_base
	model->blockid_to_pose_unit_id_map[9] = 15; // index_middle, index_top
	model->blockid_to_pose_unit_id_map[10] = 14; // index_bottom, index_middle
	model->blockid_to_pose_unit_id_map[11] = 13; // index_base, index_bottom
	model->blockid_to_pose_unit_id_map[12] = 3; // thumb_top, thumb_middle
	model->blockid_to_pose_unit_id_map[13] = 2; // thumb_bottom, thumb_middle
	model->blockid_to_pose_unit_id_map[14] = 1; // thumb_base, thumb_bottom, thumb_fold
	model->blockid_to_pose_unit_id_map[26] = 0; // thumb_base, thumb_fold, palm_thumb	
	model->blockid_to_pose_unit_id_map[27] = 3; // thumb_top, thumb_additional

	model->blockid_to_pose_unit_id_map[15] = 0; // palm_right, palm_ring, palm_pinky
	model->blockid_to_pose_unit_id_map[16] = 0; // palm_back, palm_right, palm_ring
	model->blockid_to_pose_unit_id_map[17] = 0; // palm_back, palm_middle, palm_ring
	model->blockid_to_pose_unit_id_map[18] = 0; // palm_back, palm_left, palm_middle
	model->blockid_to_pose_unit_id_map[19] = 0; // palm_left, palm_index, palm_middle
	model->blockid_to_pose_unit_id_map[20] = 0; // palm_pinky, ring_membrane, pinky_membrane
	model->blockid_to_pose_unit_id_map[21] = 0; // palm_ring, palm_pinky, ring_membrane
	model->blockid_to_pose_unit_id_map[22] = 0; // palm_ring, middle_membrane, ring_membrane
	model->blockid_to_pose_unit_id_map[23] = 0; // palm_middle, palm_ring, middle_membrane
	model->blockid_to_pose_unit_id_map[24] = 0; // palm_index, palm_middle, middle_membrane
	model->blockid_to_pose_unit_id_map[25] = 0; // palm_index, middle_membrane, index_membrane

	model->blockid_to_pose_unit_id_map[28] = 16; // wrist_bottom_left, wrist_top_left, wrist_top_right
	model->blockid_to_pose_unit_id_map[29] = 16; // wrist_bottom_right, wrist_bottom_left, wrist_top_right		

	model->blockid_to_pose_unit_id_map[30] = 0; 	
}

void ModelSemantics::setup_blockid_to_shape_unit_id_map() {

	model->blockid_to_shape_unit_id_map = vector<int>(model->blocks.size(), -1);

	if (model->calibration_type == FINGERS_PARTIAL || model->calibration_type == FINGERS || model->calibration_type == FINGERS_AND_PALM || model->calibration_type == FULL) {
		model->blockid_to_shape_unit_id_map[0] = 14; // pinky_middle, pinky_top
		model->blockid_to_shape_unit_id_map[1] = 13; // pinky_bottom, pinky_middle
		model->blockid_to_shape_unit_id_map[2] = 12; // pinky_base, pinky_bottom
		model->blockid_to_shape_unit_id_map[3] = 11; // ring_top, ring_middle
		model->blockid_to_shape_unit_id_map[4] = 10; // ring_bottom, ring_middle
		model->blockid_to_shape_unit_id_map[5] = 9; // ring_bottom, ring_base
		model->blockid_to_shape_unit_id_map[6] = 8; // middle_top, middle_middle
		model->blockid_to_shape_unit_id_map[7] = 7; // middle_bottom, middle_middle
		model->blockid_to_shape_unit_id_map[8] = 6; // middle_bottom, middle_base
		model->blockid_to_shape_unit_id_map[9] = 5; // index_middle, index_top
		model->blockid_to_shape_unit_id_map[10] = 4; // index_bottom, index_middle
		model->blockid_to_shape_unit_id_map[11] = 3; // index_base, index_bottom
		model->blockid_to_shape_unit_id_map[12] = 2; // thumb_top, thumb_middle
		model->blockid_to_shape_unit_id_map[13] = 1; // thumb_bottom, thumb_middle
		model->blockid_to_shape_unit_id_map[14] = 0; // thumb_base, thumb_bottom, thumb_fold
	}
	if (model->calibration_type == FINGERS_AND_PALM || model->calibration_type == FULL) {

		model->blockid_to_shape_unit_id_map[15] = 19; // palm_right, palm_ring, palm_pinky
		model->blockid_to_shape_unit_id_map[16] = 18; // palm_back, palm_right, palm_ring
		model->blockid_to_shape_unit_id_map[17] = 17; // palm_back, palm_middle, palm_ring
		model->blockid_to_shape_unit_id_map[18] = 16; // palm_back, palm_left, palm_middle
		model->blockid_to_shape_unit_id_map[19] = 15; // palm_left, palm_index, palm_middle

		model->blockid_to_shape_unit_id_map[20] = 24; // palm_pinky, ring_membrane, pinky_membrane
		model->blockid_to_shape_unit_id_map[21] = 25; // palm_ring, palm_pinky, ring_membrane
		model->blockid_to_shape_unit_id_map[22] = 22; // palm_ring, middle_membrane, ring_membrane
		model->blockid_to_shape_unit_id_map[23] = 23; // palm_middle, palm_ring, middle_membrane
		model->blockid_to_shape_unit_id_map[24] = 21; // palm_index, palm_middle, middle_membrane
		model->blockid_to_shape_unit_id_map[25] = 20; // palm_index, middle_membrane, index_membrane
		model->blockid_to_shape_unit_id_map[26] = 26; // thumb_base, thumb_fold, palm_thumb	

		model->blockid_to_shape_unit_id_map[27] = 28; // thumb_top, thumb_additional

		model->blockid_to_shape_unit_id_map[28] = 27; // wrist_bottom_left, wrist_top_left, wrist_top_right
		model->blockid_to_shape_unit_id_map[29] = 27; // wrist_bottom_right, wrist_bottom_left, wrist_top_right	

		model->blockid_to_shape_unit_id_map[30] = 26; 
	}
}

void ModelSemantics::setup_calibration_type_to_num_betas_map() {
	model->calibration_type_to_num_betas_map[NONE] = 0;
	model->calibration_type_to_num_betas_map[FINGERS_PARTIAL] = 15;
	model->calibration_type_to_num_betas_map[FINGERS] = 30;
	model->calibration_type_to_num_betas_map[FINGERS_AND_PALM] = 46;
	model->calibration_type_to_num_betas_map[FULL] = 75;
}

void ModelSemantics::setup_calibration_type_to_beta_template_map() {
	model->calibration_type_to_beta_template_map[NONE] = {};
	model->calibration_type_to_beta_template_map[FINGERS_PARTIAL] = {
		/*phalnages*/ 37.1409f, 28.436f, 14.0316f, 37.0552f, 20.5967f, 12.8331f, 40.4944f, 23.2687f, 15.9073f, 37.9263f, 23.9032f, 13.6011f, 31.9997f, 19.1319f, 12.8207f };

	model->calibration_type_to_beta_template_map[FINGERS] = {
		/*phalnages*/ 37.1409f, 28.436f, 14.0316f, 37.0552f, 20.5967f, 12.8331f, 40.4944f, 23.2687f, 15.9073f, 37.9263f, 23.9032f, 13.6011f, 31.9997f, 19.1319f, 12.8207f,
		/*fingers-bases*/ 9.72575f, 3.87206f, -7.16046f, 25.3963f, 50.8191f, 2.28707f, 8.13206f, 52.8925f, 7.5463f, -5.93253f, 49.1046f, 7.7991f, -18.1006f, 44.8872f, 3.8978f };

	model->calibration_type_to_beta_template_map[FINGERS_AND_PALM] = {
		/*phalnages*/ 37.1409f, 28.436f, 14.0316f, 37.0552f, 20.5967f, 12.8331f, 40.4944f, 23.2687f, 15.9073f, 37.9263f, 23.9032f, 13.6011f, 31.9997f, 19.1319f, 12.8207f,
		/*fingers-bases*/  9.72575f, 3.87206f, -7.16046f, 25.3963f, 50.8191f, 2.28707f, 8.13206f, 52.8925f, 7.5463f, -5.93253f, 49.1046f, 7.7991f, -18.1006f, 44.8872f, 3.8978f,
		/*palm-centers*/ 23.9371f, 45.629f, -17.9329f, 38.572f, 7.31059f, 3.0928f, -10.0083f, 1.01374f, 28.5967f, 41.231f,
		/*wrist*/ 4.02408f,  48.6212f,
		/*membranes*/ 12.0f,
		/*thumb additional*/ 6.7374f, 1.52305f,
		/*wrist_width_left*/ 10.02408f,
	};

	model->calibration_type_to_beta_template_map[FULL] = {
		/*phalnages*/ 37.1409f, 28.436f, 14.0316f, 37.0552f, 20.5967f, 12.8331f, 40.4944f, 23.2687f, 15.9073f, 37.9263f, 23.9032f, 13.6011f, 31.9997f, 19.1319f, 12.8207f,
		/*fingers-bases*/  9.72575f, 3.87206f, -7.16046f, 25.3963f, 50.8191f, 2.28707f, 8.13206f, 52.8925f, 7.5463f, -5.93253f, 49.1046f, 7.7991f, -18.1006f, 44.8872f, 3.8978f,
		/*palm-centers*/ 23.9371f, 45.629f, -17.9329f, 38.572f, 7.31059f, 3.0928f, -10.0083f, 1.01374f, 28.5967f, 41.231f,
		/*wrist*/ 4.02408f,  48.6212f,
		/*membranes*/ 12.0f,
		/*thumb additional*/ 6.7374f, 1.52305f,
		/*wrist_width_left*/ 10.02408f,
		/*radii*/ 15.6827f, 10.3118f, 7.30846f, 7.0311f, 8.44143f, 7.55251f, 5.85299f, 5.17427f, 7.68834f, 7.64302f, 5.67621f, 5.58432f, 7.26768f, 6.97092f, 5.01217f, 4.84959f, 
			7.84562f, 6.22559f, 4.77166f, 	4.18002f, 9.50548f, 10.726f, 10.2172f, 8.98482f, 13.2994f, 13.6244f, 13.4193f, 12.9863f, 15.4f};

	model->radii_template = { 4.18002f, 4.77166f, 6.22559f, 7.84562f, 4.84959f, 5.01217f, 6.97092f, 7.26768f, 5.58432f, 5.67621f, 7.64302f, 7.68834f, 5.17427f, 5.85299f, 7.55251f, 8.44143f, 7.0311f,
		7.30846f, 10.3118f, 15.6827f, 8.98482f, 10.2172f, 10.726f, 9.50548f, 3.0f, 13.6244f, 13.4193f, 13.2994f, 1.75341f, 2.81212f, 3.15775f, 1.29623f, 4.99162f, 3.0f, 12.9f, 12.9863f, 15.4f, 15.4213f, 5.68371f };

	model->centers_template = {
		glm::vec3(-39.1921f, 104.429f, -2.03344f), glm::vec3(-36.422f, 92.1801f, 0.5467f), glm::vec3(-30.1593f, 74.2435f, 2.80196f), glm::vec3(-18.1006f, 44.8872f, 6.8978f),
		glm::vec3(-15.8237f, 123.282f, 2.66811f), glm::vec3(-15.4072, 109.963, 5.39034), glm::vec3(-11.9253, 86.4072, 7.48125), glm::vec3(-5.93253, 49.1046, 10.7991), glm::vec3(3.87815f, 132.34f, 12.6011f),
		glm::vec3(5.30699f, 116.501f, 12.9707f), glm::vec3(7.48805, 93.3575, 11.9488), glm::vec3(8.13206f, 52.8925f, 10.5463), glm::vec3(35.1592f, 120.538f, 8.59861f), glm::vec3(33.6977f, 107.802f, 8.02238f),
		glm::vec3(30.9986f, 87.4095f, 6.97584f), glm::vec3(25.3963f, 50.8191f, 5.28707f), glm::vec3(26.7855f, 47.9654f, -70.3597f), glm::vec3(26.0498f, 38.851f, -59.7169f), glm::vec3(21.59f, 21.5549f, -37.5908f),
		glm::vec3(9.72575f, 3.87206f, -7.16046f), glm::vec3(-17.9329f, 38.572f, 3.26178f), glm::vec3(-6.55444f, 42.5953f, 6.32104f), glm::vec3(5.55773f, 45.0207f, 6.94769f), glm::vec3(23.9371f, 45.629f, 3.4816f),
		glm::vec3(28.5967f, 41.231f, 1.30524f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(-10.0083f, 1.01374f, 0.437164f), glm::vec3(7.31059f, 3.0928f, -0.0490112f), glm::vec3(-25.2322f, 60.8668f, 0.0655236f),
		glm::vec3(-7.56245f, 66.5954f, 4.81082f), glm::vec3(6.76715f, 68.1079f, 6.43872f), glm::vec3(24.194f, 64.3739f, 0.365077f), glm::vec3(27.9609f, 51.9754f, -75.8588f), glm::vec3(19.6827f, 22.8506f, -26.804f),
		glm::vec3(8.1165f, -4.05823f, 0.0f), glm::vec3(-8.1165f, -4.05823f, 0.0f), glm::vec3(4.05823f, -48.6988f, 0.0f), glm::vec3(-4.05823f, -48.6988f, 0.0f), glm::vec3(24.139700, 32.040798, -12.749390) };
}

void ModelSemantics::setup_pose_dofs() {

	// 0 - Translation
	model->pose_dofs[0].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[0].type = TRANSLATION_AXIS;
	model->pose_dofs[0].phalange_id = 17;
	// 1 - Translation
	model->pose_dofs[1].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[1].type = TRANSLATION_AXIS;
	model->pose_dofs[1].phalange_id = 17;
	// 2 - Translation
	model->pose_dofs[2].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[2].type = TRANSLATION_AXIS;
	model->pose_dofs[2].phalange_id = 17;
	// 3 - Rotation
	model->pose_dofs[3].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[3].type = ROTATION_AXIS;
	model->pose_dofs[3].phalange_id = 17;
	// 4 - Rotation
	model->pose_dofs[4].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[4].type = ROTATION_AXIS;
	model->pose_dofs[4].phalange_id = 17;
	// 5 - Rotation
	model->pose_dofs[5].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[5].type = ROTATION_AXIS;
	model->pose_dofs[5].phalange_id = 17;

	// 6 - Nothing
	model->pose_dofs[6].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[6].type = ROTATION_AXIS;
	model->pose_dofs[6].phalange_id = -1;

	// 7 - Wrist abduction
	model->pose_dofs[7].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[7].type = ROTATION_AXIS;
	model->pose_dofs[7].phalange_id = 16;
	// 8 - Wrist flexion
	model->pose_dofs[8].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[8].type = ROTATION_AXIS;
	model->pose_dofs[8].phalange_id = 16;

	// 9 - HandThumb1 abduction
	model->pose_dofs[9].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[9].type = ROTATION_AXIS;
	model->pose_dofs[9].phalange_id = 1;
	// 10 - HandThumb1 flexion
	model->pose_dofs[10].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[10].type = ROTATION_AXIS;
	model->pose_dofs[10].phalange_id = 1;
	// 11 - HandThumb2
	model->pose_dofs[11].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[11].type = ROTATION_AXIS;
	model->pose_dofs[11].phalange_id = 2;
	// 12 - HandThumb3
	model->pose_dofs[12].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[12].type = ROTATION_AXIS;
	model->pose_dofs[12].phalange_id = 3;

	// 13 - HandIndex1 abduction
	model->pose_dofs[13].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[13].type = ROTATION_AXIS;
	model->pose_dofs[13].phalange_id = 13;
	// 14 - HandIndex1 flexion
	model->pose_dofs[14].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[14].type = ROTATION_AXIS;
	model->pose_dofs[14].phalange_id = 13;
	// 15 - HandIndex2
	model->pose_dofs[15].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[15].type = ROTATION_AXIS;
	model->pose_dofs[15].phalange_id = 14;
	// 16 - HandIndex3
	model->pose_dofs[16].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[16].type = ROTATION_AXIS;
	model->pose_dofs[16].phalange_id = 15;

	// 17 - HandMiddle1 abduction
	model->pose_dofs[17].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[17].type = ROTATION_AXIS;
	model->pose_dofs[17].phalange_id = 10;
	// 18 - HandMiddle1 flexion
	model->pose_dofs[18].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[18].type = ROTATION_AXIS;
	model->pose_dofs[18].phalange_id = 10;
	// 19 - HandMiddle2 flexion
	model->pose_dofs[19].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[19].type = ROTATION_AXIS;
	model->pose_dofs[19].phalange_id = 11;
	// 20 - HandMiddle3 flexion
	model->pose_dofs[20].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[20].type = ROTATION_AXIS;
	model->pose_dofs[20].phalange_id = 12;

	// 21 - HandRing1 abduction
	model->pose_dofs[21].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[21].type = ROTATION_AXIS;
	model->pose_dofs[21].phalange_id = 7;
	// 22 - HandRing1 flexion
	model->pose_dofs[22].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[22].type = ROTATION_AXIS;
	model->pose_dofs[22].phalange_id = 7;
	// 23 - HandRing2 flexion
	model->pose_dofs[23].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[23].type = ROTATION_AXIS;
	model->pose_dofs[23].phalange_id = 8;
	// 24 - HandRing3 flexion
	model->pose_dofs[24].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[24].type = ROTATION_AXIS;
	model->pose_dofs[24].phalange_id = 9;

	// 25 - HandPinky1 abduction
	model->pose_dofs[25].axis = Eigen::Vector3f(0, 0, 1);
	model->pose_dofs[25].type = ROTATION_AXIS;
	model->pose_dofs[25].phalange_id = 4;
	// 26 - HandPinky1 flexion
	model->pose_dofs[26].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[26].type = ROTATION_AXIS;
	model->pose_dofs[26].phalange_id = 4;
	// 27 - HandPinky2 flexion
	model->pose_dofs[27].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[27].type = ROTATION_AXIS;
	model->pose_dofs[27].phalange_id = 5;
	// 28 - HandPinky3 flexion
	model->pose_dofs[28].axis = Eigen::Vector3f(1, 0, 0);
	model->pose_dofs[28].type = ROTATION_AXIS;
	model->pose_dofs[28].phalange_id = 6;

	// 29 - HandThumb1 twist
	model->pose_dofs[29].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[29].type = ROTATION_AXIS;
	model->pose_dofs[29].phalange_id = 1;
	// 30 - HandIndex1 twist
	model->pose_dofs[30].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[30].type = ROTATION_AXIS;
	model->pose_dofs[30].phalange_id = 13;
	// 31 - HandMiddle1 twist
	model->pose_dofs[31].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[31].type = ROTATION_AXIS;
	model->pose_dofs[31].phalange_id = 10;
	// 32 - HandRing1 twist
	model->pose_dofs[32].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[32].type = ROTATION_AXIS;
	model->pose_dofs[32].phalange_id = 7;
	// 33 - HandPinky1 twist
	model->pose_dofs[33].axis = Eigen::Vector3f(0, 1, 0);
	model->pose_dofs[33].type = ROTATION_AXIS;
	model->pose_dofs[33].phalange_id = 4;

	/*for (size_t i = 0; i < model->pose_dofs.size(); i++) {
		cout << endl << i << endl;
		cout << "min: " << model->pose_dofs[i].min << endl;
		cout << "max: " << model->pose_dofs[i].max << endl;
		cout << "axis = Eigen::Vector3f(" << model->pose_dofs[i].axis[0] << ", " << model->pose_dofs[i].axis[1] << ", " << model->pose_dofs[i].axis[02] << ");" << endl;
		cout << "type = " << model->pose_dofs[i].type << ";" << endl;
		cout << "parameter_id = " << model->pose_dofs[i].parameter_id << ";" << endl;
		cout << "phalange_id = " << model->pose_dofs[i].phalange_id << ";" << endl;
		}
		*/
}

void ModelSemantics::setup_thetas_limits() {
	// Joint limits
	// translation x
	model->pose_dofs[0].min = -numeric_limits<float>::max();
	model->pose_dofs[0].max = numeric_limits<float>::max();
	// translation y
	model->pose_dofs[1].min = -numeric_limits<float>::max();
	model->pose_dofs[1].max = numeric_limits<float>::max();
	// translation z
	model->pose_dofs[2].min = -numeric_limits<float>::max();
	model->pose_dofs[2].max = numeric_limits<float>::max();
	// rotation x
	model->pose_dofs[3].min = -numeric_limits<float>::max();
	model->pose_dofs[3].max = numeric_limits<float>::max();
	// rotation y
	model->pose_dofs[4].min = -numeric_limits<float>::max();
	model->pose_dofs[4].max = numeric_limits<float>::max();
	// rotation z
	model->pose_dofs[5].min = -numeric_limits<float>::max();
	model->pose_dofs[5].max = numeric_limits<float>::max();

	// nothing
	model->pose_dofs[6].min = -numeric_limits<float>::max();
	model->pose_dofs[6].max = numeric_limits<float>::max();

	// wrist adbuction
	model->pose_dofs[7].min = -1;
	model->pose_dofs[7].max = 0.2;
	// wrist flexion
	model->pose_dofs[8].min = -1;
	model->pose_dofs[8].max = 1.6;

	// thumb abduction
	model->pose_dofs[9].min = -0.1;
	model->pose_dofs[9].max = 1.0;
	// thumb flexion 1
	model->pose_dofs[10].min = -0.5;
	model->pose_dofs[10].max = 1.0;
	// thumb flexion 2
	model->pose_dofs[11].min = -0.9;
	model->pose_dofs[11].max = 0.9;
	// thumb flexion 3
	model->pose_dofs[12].min = -1.4;
	model->pose_dofs[12].max = 0.7;

	// index abductions
	model->pose_dofs[13].min = -0.30;
	model->pose_dofs[13].max = 0.35;
	// index flexion 1
	model->pose_dofs[14].min = -1.30;
	model->pose_dofs[14].max = 1.60;
	// index flexion 2
	model->pose_dofs[15].min = -0.10;
	model->pose_dofs[15].max = 2.00;
	// index flexion 3
	model->pose_dofs[16].min = -0.10;
	model->pose_dofs[16].max = 2.00;

	// middle abduction
	model->pose_dofs[17].min = -0.30;
	model->pose_dofs[17].max = 0.40;
	// middle flexion 1
	model->pose_dofs[18].min = -1.30;
	model->pose_dofs[18].max = 1.60;
	// middle flexion 2
	model->pose_dofs[19].min = -0.10;
	model->pose_dofs[19].max = 2.00;
	// middle flexion 3
	model->pose_dofs[20].min = -0.10;
	model->pose_dofs[20].max = 2.00;

	// ring abduction
	model->pose_dofs[21].min = -0.10;
	model->pose_dofs[21].max = 0.30;
	// ring flexion 1
	model->pose_dofs[22].min = -1.30;
	model->pose_dofs[22].max = 1.60;
	// ring flexion 2
	model->pose_dofs[23].min = -0.10;
	model->pose_dofs[23].max = 2.00;
	// ring flexion 3
	model->pose_dofs[24].min = -0.10;
	model->pose_dofs[24].max = 2.00;

	// pinky abduction
	model->pose_dofs[25].min = -0.20;
	model->pose_dofs[25].max = 0.65;
	// pinky flexion 1
	model->pose_dofs[26].min = -1.30;
	model->pose_dofs[26].max = 1.60;
	// pinky flexion 2
	model->pose_dofs[27].min = -0.10;
	model->pose_dofs[27].max = 2.00;
	// pinky flexion 3
	model->pose_dofs[28].min = -0.10;
	model->pose_dofs[28].max = 2.00;

	// thumb twist
	model->pose_dofs[29].min = -0.40;
	model->pose_dofs[29].max = 0.40;
	// index twist
	model->pose_dofs[30].min = -0.45;
	model->pose_dofs[30].max = 0.05;
	// middle twist
	model->pose_dofs[31].min = -0.15;
	model->pose_dofs[31].max = 0.05;
	// ring twist
	model->pose_dofs[32].min = -0.10;
	model->pose_dofs[32].max = 0.05;
	// pinky twist
	model->pose_dofs[33].min = -0.10;
	model->pose_dofs[33].max = 0.05;

}

void ModelSemantics::setup_shape_dofs() {

	if (model->calibration_type == FINGERS_PARTIAL || model->calibration_type == FINGERS || model->calibration_type == FINGERS_AND_PALM || model->calibration_type == FULL) {

		model->shape_dofs[0].set("thumb_bottom_length", PHALANGE_LENGTH, 1 /*phalange_id*/, 19 /*center_id*/, 18 /*top_center_id*/, 33 /*attachment_center_id*/);
		model->shape_dofs[1].set("thumb_middle_length", PHALANGE_LENGTH, 2 /*phalange_id*/, 18 /*center_id*/, 17 /*top_center_id*/);
		model->shape_dofs[2].set("thumb_top_length", TOP_PHALANGE_LENGTH, 3 /*phalange_id*/, 17 /*center_id*/, 16 /*top_center_id*/, 32 /*attachment_center_id*/);

		model->shape_dofs[3].set("index_bottom_length", PHALANGE_LENGTH, 13 /*phalange_id*/, 15 /*center_id*/, 14 /*top_center_id*/, 31 /*attachment_center_id*/);
		model->shape_dofs[4].set("index_middle_length", PHALANGE_LENGTH, 14 /*phalange_id*/, 14 /*center_id*/, 13 /*top_center_id*/);
		model->shape_dofs[5].set("index_top_length", TOP_PHALANGE_LENGTH, 15 /*phalange_id*/, 13 /*center_id*/, 12 /*top_center_id*/);

		model->shape_dofs[6].set("middle_bottom_length", PHALANGE_LENGTH, 10 /*phalange_id*/, 11 /*center_id*/, 10 /*top_center_id*/, 30 /*attachment_center_id*/);
		model->shape_dofs[7].set("middle_middle_length", PHALANGE_LENGTH, 11 /*phalange_id*/, 10 /*center_id*/, 9 /*top_center_id*/);
		model->shape_dofs[8].set("middle_top_length", TOP_PHALANGE_LENGTH, 12 /*phalange_id*/, 9 /*center_id*/, 8 /*top_center_id*/);

		model->shape_dofs[9].set("ring_bottom_length", PHALANGE_LENGTH, 7 /*phalange_id*/, 7 /*center_id*/, 6 /*top_center_id*/, 29 /*attachment_center_id*/);
		model->shape_dofs[10].set("ring_middle_length", PHALANGE_LENGTH, 8 /*phalange_id*/, 6 /*center_id*/, 5 /*top_center_id*/);
		model->shape_dofs[11].set("ring_top_length", TOP_PHALANGE_LENGTH, 9 /*phalange_id*/, 5 /*center_id*/, 4 /*top_center_id*/);

		model->shape_dofs[12].set("pinky_bottom_length", PHALANGE_LENGTH, 4 /*phalange_id*/, 3 /*center_id*/, 2 /*top_center_id*/, 28 /*attachment_center_id*/);
		model->shape_dofs[13].set("pinky_middle_length", PHALANGE_LENGTH, 5 /*phalange_id*/, 2 /*center_id*/, 1 /*top_center_id*/);
		model->shape_dofs[14].set("pinky_top_length", TOP_PHALANGE_LENGTH, 6 /*phalange_id*/, 1 /*center_id*/, 0 /*top_center_id*/);
	}

	if (model->calibration_type == FINGERS || model->calibration_type == FINGERS_AND_PALM || model->calibration_type == FULL) {
		model->shape_dofs[15].set("thumb_base_x", FINGER_BASE_X, 1 /*phalange_id*/, 19 /*center_id*/, -1 /*top_center_id*/, 24 /*attachment_center_id*/);
		model->shape_dofs[16].set("thumb_base_y", FINGER_BASE_Y, 1 /*phalange_id*/, 19 /*center_id*/, -1 /*top_center_id*/, 24 /*attachment_center_id*/);
		model->shape_dofs[17].set("thumb_base_z", FINGER_BASE_Z, 1 /*phalange_id*/, 19 /*center_id*/, -1 /*top_center_id*/, 24 /*attachment_center_id*/);

		model->shape_dofs[18].set("index_base_x", FINGER_BASE_X, 13 /*phalange_id*/, 15 /*center_id*/, -1 /*top_center_id*/, 31 /*attachment_center_id*/);
		model->shape_dofs[19].set("index_base_y", FINGER_BASE_Y, 13 /*phalange_id*/, 15 /*center_id*/, -1 /*top_center_id*/, 31 /*attachment_center_id*/);
		model->shape_dofs[20].set("index_base_z", FINGER_BASE_Z, 13 /*phalange_id*/, 15 /*center_id*/, -1 /*top_center_id*/, 31 /*attachment_center_id*/);

		model->shape_dofs[21].set("middle_base_x", FINGER_BASE_X, 10 /*phalange_id*/, 11 /*center_id*/, -1 /*top_center_id*/, 30 /*attachment_center_id*/);
		model->shape_dofs[22].set("middle_base_y", FINGER_BASE_Y, 10 /*phalange_id*/, 11 /*center_id*/, -1 /*top_center_id*/, 30 /*attachment_center_id*/);
		model->shape_dofs[23].set("middle_base_z", FINGER_BASE_Z, 10 /*phalange_id*/, 11 /*center_id*/, -1 /*top_center_id*/, 30 /*attachment_center_id*/);

		model->shape_dofs[24].set("ring_base_x", FINGER_BASE_X, 7 /*phalange_id*/, 7 /*center_id*/, -1 /*top_center_id*/, 29 /*attachment_center_id*/);
		model->shape_dofs[25].set("ring_base_y", FINGER_BASE_Y, 7 /*phalange_id*/, 7 /*center_id*/, -1 /*top_center_id*/, 29 /*attachment_center_id*/);
		model->shape_dofs[26].set("ring_base_z", FINGER_BASE_Z, 7 /*phalange_id*/, 7 /*center_id*/, -1 /*top_center_id*/, 29 /*attachment_center_id*/);

		model->shape_dofs[27].set("pinky_base_x", FINGER_BASE_X, 4 /*phalange_id*/, 3 /*center_id*/, -1 /*top_center_id*/, 28 /*attachment_center_id*/);
		model->shape_dofs[28].set("pinky_base_y", FINGER_BASE_Y, 4 /*phalange_id*/, 3 /*center_id*/, -1 /*top_center_id*/, 28 /*attachment_center_id*/);
		model->shape_dofs[29].set("pinky_base_z", FINGER_BASE_Z, 4 /*phalange_id*/, 3 /*center_id*/, -1 /*top_center_id*/, 28 /*attachment_center_id*/);
	}

	if (model->calibration_type == FINGERS_AND_PALM || model->calibration_type == FULL) {

		model->shape_dofs[30].set("index_palm_center_x", PALM_CENTER_X, 0 /*phalange_id*/, 23 /*center_id*/);
		model->shape_dofs[31].set("index_palm_center_y", PALM_CENTER_Y, 0 /*phalange_id*/, 23 /*center_id*/);

		model->shape_dofs[32].set("pinky_palm_center_x", PALM_CENTER_X, 0 /*phalange_id*/, 20 /*center_id*/);
		model->shape_dofs[33].set("pinky_palm_center_y", PALM_CENTER_Y, 0 /*phalange_id*/, 20 /*center_id*/);

		model->shape_dofs[34].set("left_palm_center_x", PALM_CENTER_X, 0 /*phalange_id*/, 27 /*center_id*/);
		model->shape_dofs[35].set("left_palm_center_y", PALM_CENTER_Y, 0 /*phalange_id*/, 27 /*center_id*/);

		model->shape_dofs[36].set("right_palm_center_x", PALM_CENTER_X, 0 /*phalange_id*/, 26 /*center_id*/);
		model->shape_dofs[37].set("right_palm_center_y", PALM_CENTER_Y, 0 /*phalange_id*/, 26 /*center_id*/);

		model->shape_dofs[38].set("thumb_palm_center_x", PALM_CENTER_X, 0 /*phalange_id*/, 24 /*center_id*/);
		model->shape_dofs[39].set("thumb_palm_center_y", PALM_CENTER_Y, 0 /*phalange_id*/, 24 /*center_id*/);

		model->shape_dofs[40].set("wrist_width", PALM_CENTER_X, 16 /*phalange_id*/, 36 /*center_id*/, model->centers_name_to_id_map["wrist_top_right"] /*top_center_id*/);
		model->shape_dofs[41].set("wrist_length", PALM_CENTER_Y, 16 /*phalange_id*/, 36 /*center_id*/);
		model->shape_dofs[45].set("wrist_width_left", PALM_CENTER_X, 16 /*phalange_id*/, 34 /*center_id*/, model->centers_name_to_id_map["wrist_top_left"] /*top_center_id*/);

		model->shape_dofs[42].set("thumb_membrane_length", TOP_PHALANGE_LENGTH, 2 /*phalange_id*/, 18 /*center_id*/, 33 /*top_center_id*/, 33 /*attachment_center_id*/);
		model->shape_dofs[43].set("thumb_additional_y", FINGER_TOP_Y, 3 /*phalange_id*/, 16 /*center_id*/, 32 /*top_center_id*/, 32 /*attachment_center_id*/);
		model->shape_dofs[44].set("thumb_additional_z", FINGER_TOP_Z, 3 /*phalange_id*/, 16 /*center_id*/, 32 /*top_center_id*/, 32 /*attachment_center_id*/);
	}

	if (model->calibration_type == FULL) {
		size_t c = model->calibration_type_to_num_betas_map[FINGERS_AND_PALM];

		model->shape_dofs[c + 0].set("thumb_base_radius", RADIUS, 1 /*phalange_id*/, model->centers_name_to_id_map["thumb_base"]);
		model->shape_dofs[c + 1].set("thumb_bottom_radius", RADIUS, 2 /*phalange_id*/, model->centers_name_to_id_map["thumb_bottom"]);
		model->shape_dofs[c + 2].set("thumb_middle_radius", RADIUS, 3 /*phalange_id*/, model->centers_name_to_id_map["thumb_middle"]);
		model->shape_dofs[c + 3].set("thumb_top_radius", RADIUS, 3 /*phalange_id*/, model->centers_name_to_id_map["thumb_top"]);

		model->shape_dofs[c + 4].set("index_base_radius", RADIUS, 13 /*phalange_id*/, model->centers_name_to_id_map["index_base"]);
		model->shape_dofs[c + 5].set("index_bottom_radius", RADIUS, 14 /*phalange_id*/, model->centers_name_to_id_map["index_bottom"]);
		model->shape_dofs[c + 6].set("index_middle_radius", RADIUS, 15 /*phalange_id*/, model->centers_name_to_id_map["index_middle"]);
		model->shape_dofs[c + 7].set("index_top_radius", RADIUS, 15 /*phalange_id*/, model->centers_name_to_id_map["index_top"]);

		model->shape_dofs[c + 8].set("middle_base_radius", RADIUS, 10 /*phalange_id*/, model->centers_name_to_id_map["middle_base"]);
		model->shape_dofs[c + 9].set("middle_bottom_radius", RADIUS, 11 /*phalange_id*/, model->centers_name_to_id_map["middle_bottom"]);
		model->shape_dofs[c + 10].set("middle_middle_radius", RADIUS, 12 /*phalange_id*/, model->centers_name_to_id_map["middle_middle"]);
		model->shape_dofs[c + 11].set("middle_top_radius", RADIUS, 12 /*phalange_id*/, model->centers_name_to_id_map["middle_top"]);

		model->shape_dofs[c + 12].set("ring_base_radius", RADIUS, 7 /*phalange_id*/, model->centers_name_to_id_map["ring_base"]);
		model->shape_dofs[c + 13].set("ring_bottom_radius", RADIUS, 8 /*phalange_id*/, model->centers_name_to_id_map["ring_bottom"]);
		model->shape_dofs[c + 14].set("ring_middle_radius", RADIUS, 9 /*phalange_id*/, model->centers_name_to_id_map["ring_middle"]);
		model->shape_dofs[c + 15].set("ring_top_radius", RADIUS, 9 /*phalange_id*/, model->centers_name_to_id_map["ring_top"]);

		model->shape_dofs[c + 16].set("pinky_base_radius", RADIUS, 4 /*phalange_id*/, model->centers_name_to_id_map["pinky_base"]);
		model->shape_dofs[c + 17].set("pinky_bottom_radius", RADIUS, 5 /*phalange_id*/, model->centers_name_to_id_map["pinky_bottom"]);
		model->shape_dofs[c + 18].set("pinky_middle_radius", RADIUS, 6 /*phalange_id*/, model->centers_name_to_id_map["pinky_middle"]);
		model->shape_dofs[c + 19].set("pinky_top_radius", RADIUS, 6 /*phalange_id*/, model->centers_name_to_id_map["pinky_top"]);

		model->shape_dofs[c + 20].set("palm_index_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_index"]);
		model->shape_dofs[c + 21].set("palm_middle_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_middle"]);
		model->shape_dofs[c + 22].set("palm_ring_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_ring"]);
		model->shape_dofs[c + 23].set("palm_pinky_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_pinky"]);

		model->shape_dofs[c + 24].set("palm_left_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_left"]);
		model->shape_dofs[c + 25].set("palm_back_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_back"]);
		model->shape_dofs[c + 26].set("palm_right_radius", RADIUS, 0 /*phalange_id*/, model->centers_name_to_id_map["palm_right"]);

		model->shape_dofs[c + 27].set("wrist_top_left_radius", RADIUS, 16 /*phalange_id*/, model->centers_name_to_id_map["wrist_top_left"], model->centers_name_to_id_map["wrist_top_right"] /*top_center_id*/);
		model->shape_dofs[c + 28].set("wrist_bottom_left_radius", RADIUS, 16 /*phalange_id*/, model->centers_name_to_id_map["wrist_bottom_left"], model->centers_name_to_id_map["wrist_bottom_right"]  /*top_center_id*/);

	}

	for (size_t i = 0; i < model->shape_dofs.size(); i++) {
		std::string name = model->shape_dofs[i].name;
		model->shape_dofs_name_to_id_map[name] = i;
	}
}

void ModelSemantics::setup_betas_limits() {
	if (model->num_betas == 0) return;

	for (size_t i = 0; i < model->num_betas; i++) {
		BetaType beta_type = model->shape_dofs[i].type;

		model->shape_dofs[i].min = -std::numeric_limits<float>::max();
		model->shape_dofs[i].max = std::numeric_limits<float>::max();

		if (beta_type == PHALANGE_LENGTH) {
			model->shape_dofs[i].min = 10;
			model->shape_dofs[i].max = 60;
		}
		if (beta_type == TOP_PHALANGE_LENGTH) {
			model->shape_dofs[i].min = 4;
			model->shape_dofs[i].max = 30;
		}
		if (beta_type == RADIUS) {
			model->shape_dofs[i].min = 3.0;
			model->shape_dofs[i].max = 25;
		}
		if (beta_type == FINGER_TOP_Y) {
			model->shape_dofs[i].min = 3.0;
			model->shape_dofs[i].max = 15.0;
		}
		if (beta_type == FINGER_TOP_Z) {
			model->shape_dofs[i].min = 1.0;
			model->shape_dofs[i].max = 10.0;
		}

		/*if (beta_type == FINGER_BASE_X) {
			model->shape_dofs[i].min = -50;
			model->shape_dofs[i].max = 50;
		}
		if (beta_type == FINGER_BASE_Y) {
			model->shape_dofs[i].min = -20;
			model->shape_dofs[i].max = 75;
		}
		if (beta_type == FINGER_BASE_Z) {
			model->shape_dofs[i].min = -10;
			model->shape_dofs[i].max = 15;
		}
		if (beta_type == PALM_CENTER_X) {
			model->shape_dofs[i].min = -50;
			model->shape_dofs[i].max = 50;
		}
		if (beta_type == PALM_CENTER_Y) {
			model->shape_dofs[i].min = -5;
			model->shape_dofs[i].max = 80;
		}

		{ // process some limits separatedly
			if (model->shape_dofs[i].name == "thumb_base_x") {
				model->shape_dofs[i].min = 0;
				model->shape_dofs[i].max = 20;
			}
			if (model->shape_dofs[i].name == "thumb_base_y") {
				model->shape_dofs[i].min = 2;
				model->shape_dofs[i].max = 15;
			}
			if (model->shape_dofs[i].name == "thumb_base_z") {
				model->shape_dofs[i].min = -15;
				model->shape_dofs[i].max = 5;
			}
			if (model->shape_dofs[i].name == "index_palm_center_x") {
				model->shape_dofs[i].min = 15;
				model->shape_dofs[i].max = 40;
			}
			if (model->shape_dofs[i].name == "left_palm_center_x") {
				model->shape_dofs[i].min = 5;
				model->shape_dofs[i].max = 15;
			}
			if (model->shape_dofs[i].name == "pinky_palm_center_x") {
				model->shape_dofs[i].min = -40;
				model->shape_dofs[i].max = -15;
			}
			if (model->shape_dofs[i].name == "right_palm_center_x") {
				model->shape_dofs[i].min = -15;
				model->shape_dofs[i].max = -5;
			}
			if (model->shape_dofs[i].name == "index_palm_center_y" || model->shape_dofs[i].name == "pinky_palm_center_y") {
				model->shape_dofs[i].min = 30;
				model->shape_dofs[i].max = 60;
			}
			if (model->shape_dofs[i].name == "left_palm_center_y" || model->shape_dofs[i].name == "right_palm_center_y") {
				model->shape_dofs[i].min = 0;
				model->shape_dofs[i].max = 15;
			}
		}*/
	}
}

void ModelSemantics::setup_phalanges() {

	model->phalanges[0].name = "Hand";
	model->phalanges_name_to_id_map["Hand"] = 0;
	model->phalanges[0].parent_id = 17;
	model->phalanges[0].segment_id = 3;
	model->phalanges[0].children_ids = { 1, 4, 7, 10, 13, 16 };

	model->phalanges[1].name = "HandThumb1";
	model->phalanges_name_to_id_map["HandThumb1"] = 1;
	model->phalanges[1].parent_id = 0;
	model->phalanges[1].segment_id = 4;
	model->phalanges[1].children_ids = { 2 };

	model->phalanges[2].name = "HandThumb2";
	model->phalanges_name_to_id_map["HandThumb2"] = 2;
	model->phalanges[2].parent_id = 1;
	model->phalanges[2].segment_id = 5;
	model->phalanges[2].children_ids = { 3 };

	model->phalanges[3].name = "HandThumb3";
	model->phalanges_name_to_id_map["HandThumb3"] = 3;
	model->phalanges[3].parent_id = 2;
	model->phalanges[3].segment_id = 6;
	model->phalanges[3].children_ids = {};

	model->phalanges[4].name = "HandPinky1";
	model->phalanges_name_to_id_map["HandPinky1"] = 4;
	model->phalanges[4].parent_id = 0;
	model->phalanges[4].segment_id = 8;
	model->phalanges[4].children_ids = { 5 };

	model->phalanges[5].name = "HandPinky2";
	model->phalanges_name_to_id_map["HandPinky2"] = 5;
	model->phalanges[5].parent_id = 4;
	model->phalanges[5].segment_id = 9;
	model->phalanges[5].children_ids = { 6 };

	model->phalanges[6].name = "HandPinky3";
	model->phalanges_name_to_id_map["HandPinky3"] = 6;
	model->phalanges[6].parent_id = 5;
	model->phalanges[6].segment_id = 10;
	model->phalanges[6].children_ids = {};

	model->phalanges[7].name = "HandRing1";
	model->phalanges_name_to_id_map["HandRing1"] = 7;
	model->phalanges[7].parent_id = 0;
	model->phalanges[7].segment_id = 12;
	model->phalanges[7].children_ids = { 8 };

	model->phalanges[8].name = "HandRing2";
	model->phalanges_name_to_id_map["HandRing2"] = 8;
	model->phalanges[8].parent_id = 7;
	model->phalanges[8].segment_id = 13;
	model->phalanges[8].children_ids = { 9 };

	model->phalanges[9].name = "HandRing3";
	model->phalanges_name_to_id_map["HandRing3"] = 9;
	model->phalanges[9].parent_id = 8;
	model->phalanges[9].segment_id = 14;
	model->phalanges[9].children_ids = {};

	model->phalanges[10].name = "HandMiddle1";
	model->phalanges_name_to_id_map["HandMiddle1"] = 10;
	model->phalanges[10].parent_id = 0;
	model->phalanges[10].segment_id = 16;
	model->phalanges[10].children_ids = { 11 };

	model->phalanges[11].name = "HandMiddle2";
	model->phalanges_name_to_id_map["HandMiddle2"] = 11;
	model->phalanges[11].parent_id = 10;
	model->phalanges[11].segment_id = 17;
	model->phalanges[11].children_ids = { 12 };

	model->phalanges[12].name = "HandMiddle3";
	model->phalanges_name_to_id_map["HandMiddle3"] = 12;
	model->phalanges[12].parent_id = 11;
	model->phalanges[12].segment_id = 18;
	model->phalanges[12].children_ids = {};

	model->phalanges[13].name = "HandIndex1";
	model->phalanges_name_to_id_map["HandIndex1"] = 13;
	model->phalanges[13].parent_id = 0;
	model->phalanges[13].segment_id = 20;
	model->phalanges[13].children_ids = { 14 };

	model->phalanges[14].name = "HandIndex2";
	model->phalanges_name_to_id_map["HandIndex2"] = 14;
	model->phalanges[14].parent_id = 13;
	model->phalanges[14].segment_id = 21;
	model->phalanges[14].children_ids = { 15 };

	model->phalanges[15].name = "HandIndex3";
	model->phalanges_name_to_id_map["HandIndex3"] = 15;
	model->phalanges[15].parent_id = 14;
	model->phalanges[15].segment_id = 22;
	model->phalanges[15].children_ids = {};

	model->phalanges[16].name = "Wrist";
	model->phalanges_name_to_id_map["Wrist"] = 16;
	model->phalanges[16].parent_id = 0;
	model->phalanges[16].segment_id = -1;
	model->phalanges[16].children_ids = {};

	model->phalanges[17].name = "Position";
	model->phalanges_name_to_id_map["Position"] = 17;
	model->phalanges[17].parent_id = -1;
	model->phalanges[17].segment_id = 0;
	model->phalanges[17].children_ids = { 0 };

	/*for (size_t i = 0; i < num_phalanges + 1; i++) {
		Phalange phalange = model->phalanges[i];
		//cout << "id = " << i << endl;
		cout << "model->phalanges[" << i << "].name = " << "\"" << phalange.name << "\";" << endl;
		cout << "model->phalanges_name_to_id_map[" << model->phalanges[i].name << "] = " << i << ";" << endl;
		cout << "model->phalanges[" << i << "].parent_id = " << phalange.parent_id << ";" << endl;
		cout << "model->phalanges[" << i << "].length = " << phalange.length << ";" << endl;
		cout << "model->phalanges[" << i << "].radius1 = " << phalange.radius1 << ";" << endl;
		cout << "model->phalanges[" << i << "].radius2 = " << phalange.radius2 << ";" << endl;
		cout << "model->phalanges[" << i << "].segment_id = " << phalange.segment_id << ";" << endl;

		cout << "model->phalanges[" << i << "].kinematic_chain = {";
		for (size_t j = 0; j < phalange.kinematic_chain.size(); j++)
		cout << phalange.kinematic_chain[j] << ", ";
		cout << "};" << endl;
		cout << "model->phalanges[" << i << "].children = {";
		for (size_t j = 0; j < phalange.children_ids.size(); j++)
		cout << phalange.children_ids[j] << ", ";
		cout << "};" << endl << endl;
		}*/
}

void ModelSemantics::setup_semantic_limits() {
	if (model->num_betas == 0) return;

	if (model->calibration_type == FULL) {
		using tuple = std::tuple<int, float, bool>;
		std::vector<tuple> smaller, bigger;

		/// Prevent finger bases from switching places
		{
			// 0 < middle_base_x
			smaller = {};
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_x"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// ring_base_x < 0
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_base_x"], 1.0f, true) };
			bigger = {};
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// middle_base_x + middle_base_radius + 2 < index_base_x + index_base_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], -0.9f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// ring_base_x + ring_base_radius < middle_base_x + middle_base_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["ring_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_base_radius"], -0.6f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// pinky_base_x + pinky_base_radius < ring_base_x + ring_base_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["ring_base_radius"], -0.9f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent finger bases from sticking out of the palm in Z direction
		{
			// index_base_z - index_base_radius < palm_index_radius < index_base_z + index_base_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// middle_base_z + middle_base_radius < palm_middle_radius < middle_base_z + middle_base_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_base_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["palm_middle_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_middle_radius"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_base_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// ring_base_z - ring_base_radius  < palm_ring_radius < ring_base_z + ring_base_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_ring_radius"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["ring_base_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_ring_radius"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["ring_base_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// pinky_base_z - pinky_base_radius < palm_pinky_radius < pinky_base_z + pinky_base_radius 			
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["palm_pinky_radius"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_pinky_radius"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_base_z"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent palm top centers from sticking out in Y direction
		{
			// index_base_radius < index_base_y - index_palm_center_y < palm_index_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_palm_center_y"], -1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_palm_center_y"], -1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// pinky_base_radius < pinky_base_y - pinky_palm_center_y < palm_pinky_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_palm_center_y"], -1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_palm_center_y"], -1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["palm_pinky_radius"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent palm top centers from going too much down
		if (model->enable_shape_dofs_blockers == false) {
			// 0.6 * index_base_y + 0.6 * fraction * index_bottom_length < index_palm_center_y
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_y"], 0.6f, true), tuple(model->shape_dofs_name_to_id_map["index_bottom_length"], 0.6f * 0.36f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_y"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// 0.6 * pinky_base_y + 0.6 * fraction * pinky_bottom_length < pinky_palm_center_y
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_y"], 0.6f, true), tuple(model->shape_dofs_name_to_id_map["pinky_bottom_length"], 0.6f * 0.61f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_palm_center_y"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

		}

		/// Prevent palm top centers from sticking out in X direction
		{
			// new-one
			// index_palm_center_x + palm_index_radius < index_base_x + index_base_radius < index_palm_center_x + palm_index_radius + 0.5
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.4f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 1.0f, false), tuple(-1, 0.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// pinky_base_x < pinky_palm_center_x
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_palm_center_x"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent bottom palm centers from going lower than the wrist
		{
			// - palm_back_radius < thumb_base_y - thumb_base_radius < pinky_top_radius - palm_back_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["thumb_base_radius"], -1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["thumb_base_radius"], -1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_top_radius"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], -1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// thumb_base_z + thumb_base_radius < palm_back_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_base_z"], -1.0f, true), tuple(model->shape_dofs_name_to_id_map["thumb_base_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], 1.7f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// - palm_back_radius < right_palm_center_y - palm_right_radius < 0.1 * pinky_top_radius - palm_back_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["right_palm_center_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["palm_right_radius"], -1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["right_palm_center_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["palm_right_radius"], -1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_top_radius"], 0.1f, false), tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], -1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// - palm_back_radius < left_palm_center_y - palm_left_radius < 0.1 * pinky_top_radius - palm_back_radius 
			smaller = { tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["left_palm_center_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["palm_left_radius"], -1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["left_palm_center_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["palm_left_radius"], -1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_top_radius"], 0.1f, false), tuple(model->shape_dofs_name_to_id_map["palm_back_radius"], -1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent thumb base from going to the middle of the palm
		{
			// middle_base_x < thumb_base_x 
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_x"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_base_x"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// thumb_base_x - "thumb_base_radius" < left_palm_center_x
			//smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_base_x"], 1.0f, false),  tuple(model->shape_dofs_name_to_id_map["thumb_base_radius"], -0.4f, false) };
			//bigger = { tuple(model->shape_dofs_name_to_id_map["left_palm_center_x"], 1.0f, true) };
			//model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Bound palm_thumb with palm_index
		{
			// index_palm_center_x + palm_index_radius - 0.5 * pinky_top_radius < thumb_palm_center_x + thumb_palm_radius < index_palm_center_x + palm_index_radius +  0.5 * pinky_top_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 0.5f, false)};
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_palm_center_x"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_palm_center_x"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 0.8f, false)};
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// index_palm_center_y + 0.5 * palm_index_radius < thumb_palm_center_y < index_palm_center_y + 0.8 * palm_index_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_y"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 0.5f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_palm_center_y"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_palm_center_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_palm_center_y"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["palm_index_radius"], 1.0f, false) };			
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent finger bases from sticking out of the palm in X direction
		{
			// pinky_palm_center_x + palm_pinky_radius < pinky_base_x + pinky_base_radius  		
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_palm_center_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["palm_pinky_radius"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], 1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Prevent finger bases from sliding down while the palm is turned backwards
		if (model->enable_shape_dofs_blockers == false) {
			// (1 - f) * index_bottom_length + index_middle_length + index_top_length < index_base_y + f * index_bottom_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_bottom_length"], 1.0f - model->membranes_fractional_length[0], false), tuple(model->shape_dofs_name_to_id_map["index_middle_length"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["index_top_length"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_bottom_length"], model->membranes_fractional_length[0], false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// (1 - f) * middle_bottom_length + middle_middle_length + middle_top_length < middle_base_y + f * middle_bottom_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_bottom_length"], 1.0f - model->membranes_fractional_length[1], false), tuple(model->shape_dofs_name_to_id_map["middle_middle_length"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["middle_top_length"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_bottom_length"], model->membranes_fractional_length[1], false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// (1 - f) * ring_bottom_length + ring_middle_length + ring_top_length < ring_base_y + f * ring_bottom_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_bottom_length"], 1.0f - model->membranes_fractional_length[2], false), tuple(model->shape_dofs_name_to_id_map["ring_middle_length"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["ring_top_length"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["ring_bottom_length"], model->membranes_fractional_length[2], false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// (1 - f) * pinky_bottom_length + pinky_middle_length + pinky_top_length < pinky_base_y + f * pinky_bottom_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_bottom_length"], 1.0f - model->membranes_fractional_length[3], false), tuple(model->shape_dofs_name_to_id_map["pinky_middle_length"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["pinky_top_length"], 1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_base_y"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_bottom_length"], model->membranes_fractional_length[3], false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Control the radii
		{
			// index_base_x - pinky_base_x < index_base_radius + 2 * middle_base_radius + 2 * ring_base_radius + pinky_base_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_base_radius"], 1.8f, true), tuple(model->shape_dofs_name_to_id_map["ring_base_radius"], 1.8f, true), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// index_base_x - pinky_base_x < 2 * index_bottom_radius + 2 * middle_bottom_radius + 2 * ring_bottom_radius + 2 * pinky_bottom_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_bottom_radius"], 1.2f, true), tuple(model->shape_dofs_name_to_id_map["middle_bottom_radius"], 2.0f, true), tuple(model->shape_dofs_name_to_id_map["ring_bottom_radius"], 2.0f, true), tuple(model->shape_dofs_name_to_id_map["pinky_bottom_radius"], 1.2f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// index_bottom_radius + 2 * middle_bottom_radius + 2 * ring_bottom_radius + pinky_bottom_radius < index_base_x - pinky_base_x 			
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_bottom_radius"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["middle_bottom_radius"], 1.5f, true), tuple(model->shape_dofs_name_to_id_map["ring_bottom_radius"], 1.5f, true), tuple(model->shape_dofs_name_to_id_map["pinky_bottom_radius"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], -1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		// 1.1 * thumb_middle_radius < thumb_top_radius
		smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_radius"], 1.05f, true) };
		bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_top_radius"], 1.0f, true) };
		model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

		// 1.6 * thumb_top_radius < thumb_top_length
		smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_top_radius"], 1.6f, true) };
		bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_top_length"], 1.0f, true) };
		model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

		/// Finger bases
		{
			// index_base_y < middle_base_y < index_base_y + 3
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_y"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_y"], 1.0f, true), tuple(-1, 3.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// ring_base_y < middle_base_y < ring_base_y + 3
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_base_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_y"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_y"], 1.0f, true), tuple(-1, 3.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// pinky_base_y < ring_base_y < pinky_base_y + 3
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_y"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_base_y"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_base_y"], 1.0f, true), tuple(-1, 3.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// index_base_z < middle_base_z < index_base_z + 3
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_z"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_z"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_z"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_z"], 1.0f, true), tuple(-1, 6.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// ring_base_z < middle_base_z < ring_base_z + 3
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_base_z"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_base_z"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_base_z"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_z"], 1.0f, true), tuple(-1, 6.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// pinky_base_z < ring_base_z < pinky_base_z + 3
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_base_z"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_base_z"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_base_z"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_base_z"], 1.0f, true), tuple(-1, 10.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}
		/// Fingers lenght ratios
		if (model->enable_shape_dofs_blockers == false) {
			float smaller_factor = 1.15f;
			float smaller_top_factor = 1.45f;
			float bigger_factor = 1.75f;
			
			// smaller_factor * thumb_middle_length < x_bottom_length < bigger_factor * thumb_middle_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], 1.3, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_bottom_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_bottom_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], bigger_factor, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// smaller_factor * (thumb_top_length + thumb_additional_y) < x_middle_length < bigger_factor * (thumb_top_length + thumb_additional_y)
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_top_length"], 2, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_top_length"], bigger_factor, true), tuple(model->shape_dofs_name_to_id_map["thumb_additional_y"], bigger_factor, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// smaller_factor * index_middle_length < x_bottom_length < bigger_factor * index_middle_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_middle_length"],smaller_factor, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_bottom_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_bottom_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_middle_length"], bigger_factor, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// smaller_factor * index_top_length< x_middle_length < bigger_factor * index_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_top_length"], smaller_top_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_middle_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_middle_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// smaller_factor * middle_middle_length < x_bottom_length < bigger_factor * middle_middle_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_middle_length"], smaller_factor, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_bottom_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_bottom_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_middle_length"], bigger_factor, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// smaller_factor * middle_top_length< x_middle_length < bigger_factor * middle_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_top_length"], smaller_top_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_middle_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_middle_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// smaller_factor * ring_middle_length < x_bottom_length < bigger_factor * ring_middle_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_middle_length"], smaller_factor, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_bottom_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_bottom_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_middle_length"], bigger_factor, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// smaller_factor * ring_top_length< x_middle_length < bigger_factor * ring_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_top_length"], smaller_top_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_middle_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_middle_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// smaller_factor * pinky_middle_length < x_bottom_length < bigger_factor * pinky_middle_length 
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_middle_length"], smaller_factor, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_bottom_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_bottom_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_middle_length"], bigger_factor, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			// smaller_factor * pinky_top_length< x_middle_length < bigger_factor * pinky_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_top_length"], smaller_top_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_middle_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_middle_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_top_length"], 1.0, true), tuple(model->shape_dofs_name_to_id_map["thumb_additional_y"], 1.0, true) };
		bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], 0.95f, false) };
		model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

		smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_top_length"], 0.5, true),  };
		bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_additional_y"], 1.0, true) };
		model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

		/// Length of top phalanges is similar
		if (model->enable_shape_dofs_blockers == false) {
			float smaller_factor = 0.97f;
			float bigger_factor = 1.10f;
			// smaller_factor * pinky_top_length < ring_top_length < bigger_factor * pinky_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["pinky_top_length"], smaller_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_top_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_top_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["pinky_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// smaller_factor * ring_top_length < middle_top_length < bigger_factor * ring_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["ring_top_length"], smaller_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_top_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_top_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["ring_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// smaller_factor * index_top_length < middle_top_length < bigger_factor * index_top_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["middle_top_length"], smaller_factor , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_top_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["index_top_length"], 1.0f, true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["middle_top_length"], bigger_factor , true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}
		/// Wrist
		{
			float smaller_factor = 0.95f;
			float bigger_factor = 1.01f;
			//  smaller_factor * (wrist_width + wrist_width_left + wrist_top_left) < index_base_x + index_base_radius - (pinky_base_x + pinky_base_radius) < bigger_factor * 2 * (wrist_width + wrist_width_left + wrist_top_left) 
			smaller = { tuple(model->shape_dofs_name_to_id_map["wrist_width"], smaller_factor, true),  tuple(model->shape_dofs_name_to_id_map["wrist_width_left"], smaller_factor, true), 
				tuple(model->shape_dofs_name_to_id_map["wrist_top_left_radius"], smaller_factor * 2.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, false),
				tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], -1.0f, false), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], -1.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, false), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, false),
				tuple(model->shape_dofs_name_to_id_map["pinky_base_x"], -1.0f, false), tuple(model->shape_dofs_name_to_id_map["pinky_base_radius"], -1.0f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["wrist_width"], bigger_factor, true), tuple(model->shape_dofs_name_to_id_map["wrist_width_left"], bigger_factor, true),
				tuple(model->shape_dofs_name_to_id_map["wrist_top_left_radius"],bigger_factor * 2.0f, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// 30 < beta[wrist_length] < 45
			smaller = { tuple(-1, 20, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["wrist_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
			smaller = { tuple(model->shape_dofs_name_to_id_map["wrist_length"], 1.0f, true) };
			bigger = { tuple(-1, 50, false) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			smaller = { tuple(-1, 3.5, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["wrist_width_left"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// wrist_width + wrist_width_left + 2 * wrist_top_left_radius < 2 * wrist_width + 2 * wrist_bottom_left_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["wrist_width_left"], 1.0, false), tuple(model->shape_dofs_name_to_id_map["wrist_top_left_radius"], 2.0, false)};
			bigger = { tuple(model->shape_dofs_name_to_id_map["wrist_width"], 1.1, true), tuple(model->shape_dofs_name_to_id_map["wrist_bottom_left_radius"], 1.1 * 2.0, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// right_palm_center_x - palm_right_radius < - wrist_width - wrist_top_left_radius
			smaller = { tuple(model->shape_dofs_name_to_id_map["right_palm_center_x"], 1.0, false), tuple(model->shape_dofs_name_to_id_map["palm_right_radius"], -1.0, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["wrist_width"], -1.0, true), tuple(model->shape_dofs_name_to_id_map["wrist_top_left_radius"], -1.0, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/// Thumb additional and fold
		{
			// thumb_additional_z < 0.6 * thumb_additional_y
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_additional_z"], 1.0 , true) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_additional_y"], 0.6f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// 0.5 * thumb_middle_length < thumb_membrane_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], 0.55f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_membrane_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));

			// 0.5 * (thumb_bottom_length + thumb_middle_length + thumb_top_length + thumb_additional_y) < thumb_membrane_length
			smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_bottom_length"], -0.5f, false), tuple(model->shape_dofs_name_to_id_map["thumb_middle_length"], 0.5f, false), 
				tuple(model->shape_dofs_name_to_id_map["thumb_top_length"], 0.5f, false) , tuple(model->shape_dofs_name_to_id_map["thumb_additional_y"], 0.5f, false) };
			bigger = { tuple(model->shape_dofs_name_to_id_map["thumb_membrane_length"], 1.0f, true) };
			model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}

		/*/// Palm width
		{
		// thumb_base_x + thumb_base_radius < index_base_x
		smaller = { tuple(model->shape_dofs_name_to_id_map["thumb_base_x"], 2.6f, false)};
		bigger = { tuple(model->shape_dofs_name_to_id_map["index_base_x"], 1.0f, true), tuple(model->shape_dofs_name_to_id_map["index_base_radius"], 1.0f, false) };
		model->semantic_limits.push_back(SemanticLimit(smaller, bigger));
		}*/

	}
	else {
		model->semantic_limits = {};
	}
}