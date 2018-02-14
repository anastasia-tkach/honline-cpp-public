#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Energy/Energy.h"
#include "tracker/Types.h"
#include "tracker/HModel/Model.h"
#include <vector>
#include <queue>

#include "tracker/HModel/FingertipsDetector.h"

//namespace energy {
	class Fingertips {
		Model * model;
		Camera * camera;

		void moving_window_average(int current_frame_id);

	public:

		struct Settings {
			bool enable_fingertips_prior = true;		
			bool weight = 200;
			size_t history_size = 20;
			std::vector<size_t> fingertips_centers_indices = { 32, 12, 8, 4, 0 };
			std::vector<size_t> fingermiddles_centers_indices = { 17, 13, 9, 5, 1 };
			std::vector<size_t> fingertips_block_indices = { 27, 9, 6, 3, 0 };
		} _settings;
		Settings* settings = &_settings;

		//DetectionStream temp_detection;
		//FindFingers find_fingers;
		//FingertipsDetector fingertips_detector;

		//{ model->centers_name_to_id_map["thumb_additional"],
		//model->centers_name_to_id_map["index_top"],
		//model->centers_name_to_id_map["middle_top"],
		//model->centers_name_to_id_map["ring_top"],
		//model->centers_name_to_id_map["pinky_top"] };

		std::vector<std::queue<glm::vec3>> data_fingertips_history;
		std::vector<glm::vec3> data_fingertips_sum;
		std::vector<glm::vec3> averaged_data_fingertips;
		std::vector<int> last_update_frame_id;

		int last_frame_id = -1;

		std::vector<Vector2> data_fingertips_2d;
		std::vector<glm::vec3> data_fingertips;
		std::vector<glm::vec3> ordered_data_fingertips;
		std::vector<glm::vec3> model_fingertips;
		std::vector<glm::vec3> ordered_model_fingertips;
		std::vector<int> block_indices;
		std::vector<glm::vec3> axis_projections;
		std::vector<glm::ivec3> indices;

		void track(LinearSystem & system, DataFrame & current_frame);
		void init(Model * model, Camera * camera);
	};	
//}