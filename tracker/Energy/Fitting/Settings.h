#pragma once

/// @note this needs to be placed outside of Fitting.h as these settings
///       are parsed by the CUDA portions of the code

namespace energy {
	namespace fitting {
		struct Settings {

			///--- E_2D
			//bool fit2D_enable = true;
			bool fit2D_unproject = false;
			//bool fit2D_outline_normal_direction = false;
			bool fit2D_outline_enable = false;
			bool fit2D_silhouette2outline_enable = false;
			bool fit2D_silhouette_enable = false;			
			float fit2D_weight = 1;
			float fit2D_weight_palm = 1;
			float fit2D_weight_segment = 1;

			///--- E_3D
			bool  fit3D_enable = true;
			float fit3D_weight = 1.0f;
			bool  fit3D_point2plane = true;
			bool  fit3D_reweight = true;
			bool fit3D_reweight_rigid = false;

			bool write_jacobian_to_file = false;
			bool disable_finger_bases_fitting = false;

			bool undistort = false;
			bool fit_wrist_separately = false;
			int dataset_type = 0;
		};
	}
}

