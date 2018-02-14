#include "JointLimits.h"
#include "tracker/Hmodel/Model.h"

void energy::JointLimits::init(Model * model) {
	this->model = model;
}

void energy::JointLimits::track(LinearSystem& system, const std::vector<Scalar>& theta, const std::vector<Scalar>& beta) {
	if (!jointlimits_enable) return;

	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> LHS = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(system.lhs.rows(), system.lhs.cols());
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1>  rhs = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(system.lhs.rows(), 1);
	
	for (size_t i = 0; i < num_thetas; ++i) {

		float qmin = model->pose_dofs[i].min;
		float qmax = model->pose_dofs[i].max;

		if (theta[i] > qmax) {
			//if (i == 13 || i == 14 || i == 15 || i == 16) cout << "theta[" << i << "] = " << theta[i] << endl;
			rhs(i) = (qmax - theta[i]) - std::numeric_limits<Scalar>::epsilon();
			if (isnan(rhs(i)) || isinf(rhs(i))) cout << "nan in joint limits" << endl;
			LHS(i, i) = 1;
		}
		else if (theta[i] < qmin) {
			//if (i == 13 || i == 14 || i == 15 || i == 16) cout << "theta[" << i << "] = " << theta[i] << endl;
			rhs(i) = (qmin - theta[i]) + std::numeric_limits<Scalar>::epsilon();
			if (isnan(rhs(i)) || isinf(rhs(i))) cout << "nan in joint limits" << endl;
			LHS(i, i) = 1;
		}
		else {
			LHS(i, i) = 0;
			rhs(i) = 0;
		}
	}

	if (model->num_betas > 0) {
		for (size_t i = 0; i < model->num_betas; ++i) {

			float qmin = model->shape_dofs[i].min;
			float qmax = model->shape_dofs[i].max;

			if (beta[i] > qmax) {
				//cout << "beta[" << i << "] = " << beta[i] << endl;
				rhs(num_thetas + i) = (qmax - beta[i]) - std::numeric_limits<Scalar>::epsilon();
				if (isnan(rhs(num_thetas + i)) || isinf(rhs(num_thetas + i))) cout << "nan in joint limits" << endl;
				LHS(num_thetas + i, num_thetas + i) = 1;
			}
			if (beta[i] < qmin) {
				//cout << "beta[" << i << "] = " << beta[i] << endl;
				rhs(num_thetas + i) = (qmin - beta[i]) + std::numeric_limits<Scalar>::epsilon();
				if (isnan(rhs(num_thetas + i)) || isinf(rhs(num_thetas + i))) cout << "nan in joint limits" << endl;
				LHS(num_thetas + i, num_thetas + i) = 1;
			}
		}
	}

	if (system.has_nan()) cout << "of the before Limits term" << endl;

	///--- Add constraints to the solve
	Scalar omega = jointlimits_weight;
	system.lhs.noalias() += omega * LHS.transpose() * LHS;
	system.rhs.noalias() += omega * LHS.transpose() * rhs;

	///--- Check
	if (system.has_nan()) cout << "of the Limits term" << endl;
}
