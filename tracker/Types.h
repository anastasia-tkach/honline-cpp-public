#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <limits>

using namespace std;

typedef float Scalar;
typedef Scalar Real;
typedef std::string String;
typedef unsigned int uint;
typedef int Integer;
typedef Eigen::Matrix4f Matrix4;
typedef Eigen::Matrix3f Matrix3;
typedef Eigen::Vector2f Vector2;
typedef Eigen::Vector3f Vector3;
typedef Eigen::Vector4f Vector4;
typedef Eigen::VectorXf VectorN;

const int d = 3;

/// Parameters vector
const int num_thetas = 34;
const int num_thetas_ignore = 9;
const int num_thetas_translation = 3;
const int num_thetas_rotation = 26;
const int num_thetas_rigid_motion = 6;
const int num_thetas_latent = 4;
const int num_thetas_thumb = 4;
const int num_thetas_fingers = 16;
const int num_thetas_pose = 20;
const int num_fingers = 5;
const int num_joints = 21;
const int num_temporal = 37;
const int num_phalanges = 17;

const int num_betas_latent = 5;

const int upper_bound_num_sensor_points = 76800;
const int upper_bound_num_rendered_outline_points = 5000;
const int upper_bound_num_rendered_silhouette_points = 5000;

#define M_PI 3.14159265358979323846

/// More complex matrixes
typedef Eigen::Matrix<Scalar, 2, 3> Matrix_2x3;
typedef Eigen::Matrix<Scalar, 1, Eigen::Dynamic> Matrix_1xN;
typedef Eigen::Matrix<Scalar, 2, Eigen::Dynamic> Matrix_2xN;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Matrix_3xN;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, 2> Matrix_Nx2;
typedef Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> Matrix_MxN;

typedef Eigen::Hyperplane<Scalar,3> Plane3;
typedef Eigen::ParametrizedLine<Scalar,3> Ray3;
typedef Eigen::AlignedBox3f BBox3;
typedef Eigen::Vector2i Vector2i;
typedef Eigen::AlignedBox2i BBox2i;
typedef Eigen::Quaternion<Real> Quaternion;

/// Nan for the default type
inline Real nan(){ return std::numeric_limits<Real>::quiet_NaN(); }
inline Real inf(){ return std::numeric_limits<Real>::max(); }

/// Linear system lhs*x=rhs
struct LinearSystem {
    Matrix_MxN lhs; // J^T*J
	VectorN rhs; // J^T*r
    LinearSystem() {}
    LinearSystem(int n) {
        lhs = Matrix_MxN::Zero(n,n);
        rhs = VectorN::Zero(n);
	}
	bool has_nan() {
		for (int i = 0; i < lhs.rows(); i++) {
			for (int j = 0; j < lhs.cols(); j++) {
				if (isnan(lhs(i, j))) {
					cout << "NaN detected in the LHS ";
					return true;
				}
				if (isinf(lhs(i, j))) {				
					cout << "Inf detected in the LHS ";
					return true;
				}
			}
			if (isnan(rhs(i))) {				
				cout << "NaN detected in the RHS ";
				return true;
			}
			if (isinf(rhs(i))) {				
				cout << "Inf detected in the RHS ";
				return true;
			}
		}
		return false;
	}
};

enum CalibrationType {
	NONE = 0,
	FINGERS_PARTIAL = 1,
	FINGERS = 2,
	FINGERS_AND_PALM = 3,
	FULL = 4,
};

enum CalibrationStage {
	DEFAULT = 0,
	D1 = 1,
	D4 = 2,
	DN = 3,
};

enum DatasetType {
	TKACH = 0,
	TOMPSON = 1,
	SHARP = 2,
	SRIDHAR = 3,
	CHEN = 4,
	INTEL = 5,
	VGA = 6
}; 

enum Mode {
	LIVE,
	BENCHMARK,
	PLAYBACK
};

enum KalmanType {
	DIAGONAL = 0,
	STANDARD = 1,
	UNIFORM = 2,
	EXTENDED = 3,
	HYBRID = 4
};

