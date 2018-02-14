#pragma once
#include <vector>
#include <iostream>

#ifdef __CUDACC__
#include "cudax/cuda_glm.h"
#else 
#include "tracker/Types.h"
#endif

struct BetaInfo {
#ifdef __CUDACC__
	glm::mat4x4 mat;
	glm::vec3 axis;
#else
	float mat[16];
	float axis[3];
#endif    
	int jacobian_column;
	float beta;
	int type;
	int center_id;
	int top_center_id;
	int attachment_center_id;
};

#define SHAPE_MAX_LENGTH 13
struct ShapeChain { 
	int data[SHAPE_MAX_LENGTH]; 
	void set_data(const std::vector<size_t> & input) {
		for (size_t i = 0; i < SHAPE_MAX_LENGTH; i++) {
			if (input.size() > i)
				this->data[i] = input[i];
			else
				this->data[i] = -1;
		}
	}
};

struct ThetaInfo {
#ifdef __CUDACC__
	glm::mat4x4 mat;
	glm::vec3 axis;
#else
	float mat[16];
	float axis[3];
#endif    
	int type;
	int jacobian_column;
};

#define KINEMATIC_MAX_LENGTH 11
struct KinematicChain { 
	int data[KINEMATIC_MAX_LENGTH]; 
	void set_data(const std::vector<size_t> & input) {
		for (size_t i = 0; i < KINEMATIC_MAX_LENGTH; i++) {
			if (input.size() > i)
				this->data[i] = input[i];
			else
				this->data[i] = -1;
		}
	}
};

inline std::ostream& operator<< (std::ostream& d, KinematicChain chain) {
	for (int j = 0; j < KINEMATIC_MAX_LENGTH; j++)
		d << chain.data[j] << " ";
	return d;
}
inline std::ostream& operator<< (std::ostream& d, std::vector<KinematicChain> kinematic_chains) {
	for (int i = 0; i < kinematic_chains.size(); ++i)
		d << kinematic_chains[i] << std::endl;
	return d;
}

inline std::ostream& operator<< (std::ostream& d, ThetaInfo val) {
	d << "index: " << val.jacobian_column << std::endl;
	d << "type: " << val.type << std::endl;
	d << "matrix: " << std::endl;
#ifndef __CUDACC__
	Eigen::Map<Matrix4> mat(val.mat);
#endif

	for (int row = 0; row < 4; ++row) {
		for (int col = 0; col < 4; ++col) {
#ifdef __CUDACC__			
			d << "  " << val.mat[col][row] << " "; // column major
#else
			d << "  " << mat(row, col) << " ";
#endif
		}
		d << std::endl;
	}
	d << "axis: " << std::endl << val.axis[0] << " " << val.axis[1] << " " << val.axis[2] << std::endl;
	return d;
}
