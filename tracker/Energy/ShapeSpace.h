#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Energy/Energy.h"
#include "tracker/Types.h"
#include "tracker/HModel/Model.h"
#include "tracker/Energy/PoseSpace.h"
#include <vector>

namespace energy {
	class ShapeSpace : public Energy {
		std::vector<float> beta_template;
		std::vector<float> beta_latent;
		bool enable_pose_prior;
		Model * model;
		PoseSpace * E_pose;
		std::string logs_path;

		std::vector<bool> shape_dofs_blockers_initialization_indicator;
		std::vector<float> shape_dofs_blockers_values;
		std::vector<float> shape_dofs_blockers_thresholds;
		std::vector<int> shape_dofs_blockers_beta_indices;
		std::vector<int> shape_dofs_blockers_theta_indices;

	public:
		/// passing information to kalman
		std::vector<bool> shape_dofs_blockers_conditions;
		std::vector<std::vector<int>> shape_dofs_blockers_beta_sets;

		struct Settings {
			float weight_uniform = 1;
			float weight_palm_height = 1;
			float weight_palm_width = 1;
			float weight_radii = 1;
			float enable_shape_prior = true;

			size_t num_additional_constraints;
			size_t num_semantic_limits;
			size_t num_shape_dofs_blockers;

			bool enable_additional_constraints = false;
			bool enable_semantic_limits = false;
			bool enable_shape_dofs_blockers = false;

			float weight_additional_constraints = 1;
			float weight_semantic_limits = 1;
			float weight_shape_dofs_blockers = 1;

		} _settings;
		Settings* settings = &_settings;

		void init(Model * model, PoseSpace * E_pose, std::string logs_path);

		void update_beta_template(const std::vector<float> & new_beta_template);

		void update_beta_latent(const std::vector<float> & delta_beta_latent);

		void update_beta_latent(const VectorN & solution);

		void print_beta_latent();

		std::vector<float> get_beta_latent();

		Eigen::Matrix<double, Eigen::Dynamic, 1> energy::ShapeSpace::compute_objective_double(const std::vector<double> & beta_double, int beta_index, const std::vector<double> & beta_latent_double, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & F);

		void verify_jacobian(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> & F, const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & J);

		void track(LinearSystem & system, std::vector<float> beta, int iter);
	};
}
