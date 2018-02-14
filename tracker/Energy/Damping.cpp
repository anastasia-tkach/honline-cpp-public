#include "Damping.h"
#include "tracker/TwSettings.h"
#include "tracker/HModel/Model.h"

namespace energy {

	void Damping::init(Model * model) {
		this->model = model;
		tw_settings->tw_add(settings->translation_damping, "translations", "group=Damping");
		tw_settings->tw_add(settings->rotation_damping, "rotations", "group=Damping");
	}

	void Damping::track(LinearSystem &system) {

		Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> D = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Identity(system.lhs.rows(), system.lhs.cols());
		Eigen::Matrix<Scalar, Eigen::Dynamic, 1>  d = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Ones(system.lhs.rows(), 1);
		for (int i = 0; i < num_thetas; ++i) {
			if (model->pose_dofs[i].type == TRANSLATION_AXIS)
				d(i) = settings->translation_damping;
			if (model->pose_dofs[i].type == ROTATION_AXIS)
				d(i) = settings->rotation_damping * settings->beta_vs_theta_damping;
			if (i == 13 || i == 17 || i == 21 || i == 25) d(i) = settings->abduction_damping * settings->beta_vs_theta_damping;
			if (i == 16 || i == 20 || i == 24 || i == 28) d(i) = settings->top_phalange_damping * settings->beta_vs_theta_damping;
			if (i == 29 || i == 30 || i == 31 || i == 32 || i == 33) d(i) = settings->twist_damping * settings->beta_vs_theta_damping;
			if (model->calibration_type == FULL && (i == 7 || i == 8)) d(i) = 8000000 * settings->beta_vs_theta_damping;
		}

		for (size_t i = num_thetas; i < model->num_parameters; i++) {
			size_t beta_id = i - num_thetas;
			BetaType beta_type = model->shape_dofs[beta_id].type;
			if (beta_type == BetaType::PHALANGE_LENGTH || beta_type == BetaType::TOP_PHALANGE_LENGTH)
				d(i) = settings->beta_phalange_length_damping / settings->beta_vs_theta_damping;
			if (beta_type == BetaType::FINGER_BASE_X || beta_type == BetaType::FINGER_BASE_Y || beta_type == BetaType::FINGER_BASE_Z || beta_type == BetaType::FINGER_TOP_Y || beta_type == BetaType::FINGER_TOP_Z)
				d(i) = settings->beta_finger_base_damping / settings->beta_vs_theta_damping;
			if (beta_type == BetaType::PALM_CENTER_X || beta_type == BetaType::PALM_CENTER_Y)
				d(i) = settings->beta_palm_center_damping / settings->beta_vs_theta_damping;
			if (beta_type == BetaType::RADIUS)
				d(i) = settings->beta_radius_damping / settings->beta_vs_theta_damping;
			if (i == 42) 
				d(i) = 1000 / settings->beta_vs_theta_damping;
		}

		if (system.has_nan()) cout << "of before the Damping term" << endl;


		D.diagonal() = d;
		system.lhs = system.lhs + D;

		if (system.has_nan()) cout << "of the Damping term" << endl;
	}

} 
