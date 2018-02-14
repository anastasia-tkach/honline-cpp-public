#pragma once
#include "tracker/ForwardDeclarations.h"
#include "Energy.h"
#include "tracker/Types.h"
#include <vector>

namespace energy{

class Damping : public Energy{
	Model * model = NULL;

public:
    struct Settings {
		Scalar translation_damping = 1;
		Scalar rotation_damping = 3000;
		Scalar abduction_damping = 1500000; 
		Scalar twist_damping = 3000000;
		Scalar top_phalange_damping = 10000;

		Scalar beta_phalange_length_damping = 15000;
		Scalar beta_finger_base_damping = 30000;
		Scalar beta_palm_center_damping = 50000;
		Scalar beta_radius_damping = 50000;

		float beta_vs_theta_damping = 1.0f;

    } _settings;
    Settings*const settings = &_settings;


    void init(Model * model);
	void track(LinearSystem& system);
};

}
