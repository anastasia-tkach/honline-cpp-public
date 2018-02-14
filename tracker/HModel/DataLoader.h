#pragma once
#include <fstream>
#include <vector>
#include "cudax/cuda_glm.h"

void read_model_centers(std::string data_path, std::string name, std::vector<glm::vec3> & input) {
	FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		glm::vec3 v;
		fscanf(fp, "%f%f%f", &v[0], &v[1], &v[2]);
		if (input.size() > i) {
			input[i] = v;
		}
		else {
			cout << "file size exceeed number of model centers" << endl;
		}
	}
	fclose(fp);
}

void read_model_radii(std::string data_path, std::string name, std::vector<float> & input) {
	FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		float a;
		fscanf(fp, "%f", &a);
		if (input.size() > i) {
			input[i] = a;
		}
		else {
			cout << "file size exceeed number of model radii" << endl;
		}
	}
	fclose(fp);
}

void read_model_blocks(std::string data_path, std::string name, std::vector<glm::ivec3> & input) {
	FILE *fp = fopen((data_path + name + ".txt").c_str(), "r");
	int N;
	fscanf(fp, "%d", &N);
	for (int i = 0; i < N; ++i) {
		glm::ivec3 v;
		fscanf(fp, "%d%d%d", &v[0], &v[1], &v[2]);
		if (input.size() > i) {
			input[i] = v;
		}
		else {
			cout << "file size exceeed number of model blocks" << endl;
		}
	}
	fclose(fp);
}

void write_floats(std::string data_path, std::string name, float * output, size_t size) {
	std::ofstream output_file;
	output_file.open(data_path + name + ".txt");
	for (size_t i = 0; i < size; i++) {
		output_file << output[i] << " ";
	}
	output_file.close();
}

