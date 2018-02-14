#include "Energy.h"
#include "util/mylogger.h"
namespace energy {

	static bool debug_print_delta_theta = false;

	void Energy::rigid_only(LinearSystem &system) {
		int start = 6; ///< fully rigid
		// int start = 8; ///< wrist DOF enabled
		for (int c = start; c < system.lhs.rows(); ++c)
			system.lhs.col(c).setZero();
		for (int r = start; r < system.lhs.rows(); ++r) {
			system.lhs.row(r).setZero();
			system.rhs.row(r).setZero();
		}
	}

	bool Energy::vector_has_nan(const VectorN & input) {
		for (int i = 0; i < input.size(); i++) {
			if (isnan(input[i]) || isinf(input[i])) {
				return true;
			}
		}
		return false;
	}
	bool Energy::matrix_has_nan(const Matrix_MxN & input) {
		for (int i = 0; i < input.rows(); i++) {
			for (size_t j = 0; j < input.cols(); j++) {
				if (isnan(input(i, j)) || isinf(input(i, j))) {
					cout << input(i, j) << endl;
					return true;
				}
			}			
		}
		return false;
	}
	VectorN Energy::solve(LinearSystem &system) {

		//VectorN solution = system.lhs.colPivHouseholderQr().solve(system.rhs);

		/// Compute Cholesky factorization
		Eigen::LDLT< Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> > ldltOfLHS(system.lhs);
		/// Check if LHS is SDP
		if (ldltOfLHS.info() == Eigen::NumericalIssue) //ltltOfLHS
			std::cout << "LHS is not SDP, thus solution not reliable" << std::endl;	
		VectorN solution = ldltOfLHS.solve(system.rhs);

		/// Check for NaN
		for (int i = 0; i < solution.size(); i++) {
			if (isnan(solution[i]) || isinf(solution[i])) {
				cout << "NaN or Inf detected in the solution" << endl << endl;
				return solution;
			}
		}

		return solution;
	}

} /// energy::
