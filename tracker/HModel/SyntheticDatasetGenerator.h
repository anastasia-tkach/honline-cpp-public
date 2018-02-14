#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Worker.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <vector>
#include <random>
#include <numeric>

class SyntheticDatasetGenerator {

	Worker * worker;
	std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> tracking_parameters;
	bool done = false;
	bool wrong_rotation = false;
	int epoch = 0;
	int skip = 4;
	float step_theta = M_PI / 4;
	float min_theta0 = -M_PI;
	float min_theta1 = 0;
	float min_theta2 = 0;
	float max_theta0 = M_PI / 4;//M_PI/2;
	float max_theta1 = 2 * M_PI - step_theta;// / M_PI - step_theta;
	float max_theta2 = M_PI / 4;// M_PI / 2 - step_theta;
	std::vector<float> theta_global_rotation = { min_theta0, min_theta1, min_theta2 };
	int sequence_length = 2260;
	string sequence_path;
	int data_count = 0;
public:
	SyntheticDatasetGenerator(Worker * worker, string sequence_path) :worker(worker) {
		this->sequence_path = sequence_path;		
		tracking_parameters.push_back(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(34, 1));
	}

	void set_sequence_length(int sequence_length) {
		this->sequence_length = sequence_length;
	}

	void load_recorded_parameters(std::string solution_path) {
		std::cout << "loading solutions function" << endl;
		std::ifstream solutions_file(solution_path);
		if (!solutions_file.is_open()) { std::cout << "cannot open solution file at " + solution_path << endl; exit(0); }
		int row = 0;
		for (std::string line; std::getline(solutions_file, line); row++) {
			//cout << row << endl;
			tracking_parameters.push_back(Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(num_thetas + worker->model->num_betas, 1));
			stringstream string_stream(line);
			std::string elem; size_t col = 0;

			while (string_stream >> elem) {
				if (col >= num_thetas + worker->model->num_betas) cout << "col = " << col << endl;
				tracking_parameters[row](col++) = std::stof(elem);
			}
		}
		cout << "number of loaded solutions = " << tracking_parameters.size() << endl;
		solutions_file.close();
	}

	void write_rendered_depth() {
		static cv::Mat rendered_model, rendered_silhouette;
		if (rendered_model.empty()) rendered_model = cv::Mat(worker->camera->height(), worker->camera->width(), CV_16UC1, cv::Scalar(0));
		glBindTexture(GL_TEXTURE_2D, worker->offscreen_renderer.frame_buffer->depth_texture_id());
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_SHORT, rendered_model.data);
		glBindTexture(GL_TEXTURE_2D, 0);
		cv::flip(rendered_model, rendered_model, 0);

		if (rendered_silhouette.empty()) rendered_silhouette = cv::Mat(worker->camera->height(), worker->camera->width(), CV_8UC1, cv::Scalar(0));
		glBindTexture(GL_TEXTURE_2D, worker->offscreen_renderer.frame_buffer->silhouette_texture_id());
		glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_UNSIGNED_BYTE, rendered_silhouette.data);
		glBindTexture(GL_TEXTURE_2D, 0);
		cv::flip(rendered_silhouette, rendered_silhouette, 0);

		std::string path = worker->settings->sequence_path + "/" + worker->settings->sequence_name + "/synthetic/";
		CreateDirectory(path.c_str(), NULL);
		std::ostringstream stringstream;
		stringstream << std::setfill('0') << std::setw(7) << data_count++;
		string filename = path + "depth-" + stringstream.str() + ".png";
		cv::imwrite(filename, rendered_model);

		filename = path + "mask-" + stringstream.str() + ".png";
		cv::imwrite(filename, rendered_silhouette);

		bool display = false;
		if (display) {
			cv::Mat normalized_silhouette = cv::Mat::ones(worker->camera->height(), worker->camera->width(), CV_16UC3);
			vector<cv::Mat> channels(3);
			split(normalized_silhouette, channels);
			channels[0] = rendered_model;
			merge(channels, normalized_silhouette);
			for (size_t i = 0; i < worker->model->centers.size(); i++) {
				if (i > 20 && i != 25) continue;
				Vector3 world_point = Vector3(worker->model->centers[i][0], worker->model->centers[i][1], worker->model->centers[i][2]);
				Mat3f projection_matrix = worker->camera->projection_matrix();
				Scalar x = world_point[0] / world_point[2];
				Scalar y = world_point[1] / world_point[2];
				x = x * projection_matrix(0, 0) + projection_matrix(0, 2);
				y = y * projection_matrix(1, 1) + projection_matrix(1, 2);
				Vector2 image_point = Vector2(x, y);

				cv::Point center = cv::Point(image_point[0], worker->camera->height() - image_point[1]);
				cv::Scalar color = cv::Scalar(143 * 255, 154 * 255, 217 * 255);
				cv::circle(normalized_silhouette, center, 0.5, color, 3, 8);
			}
			cv::imshow("silhouette", normalized_silhouette);
			cv::waitKey(3);
		}
	}

	void write_joint_angles() {
		static ofstream joint_angles_file(sequence_path + "synthetic/results_joint_angles.txt");
		for (size_t i = 0; i < num_thetas; i++) {
			joint_angles_file << worker->model->theta[i] << " ";
		}
		joint_angles_file << endl;
	}

	void write_joint_locations() {
		static ofstream joint_locations_file(sequence_path + "synthetic/results_joint_locations.txt");
		for (size_t i = 0; i < worker->model->centers.size(); i++) {
			if (i < 20 || i == 25)
				joint_locations_file << worker->model->centers[i][0] << " " << worker->model->centers[i][1] << " " << worker->model->centers[i][2] << " ";
		}
		joint_locations_file << endl;
	}

	void rotate_and_render_main() {
		if (done) {
			worker->glarea->close();
			return;
		}
		
		if ((worker->current_frame.id >= sequence_length - skip || wrong_rotation) && !done) {
			worker->current_frame.id = 0;
			wrong_rotation = false;
			epoch++;
			cout << "epoch " << epoch << endl;
			theta_global_rotation[0] += step_theta;

			if (theta_global_rotation[2] >= max_theta2 && theta_global_rotation[1] >= max_theta1) {
				done = true;
				return;
			}
			if (theta_global_rotation[1] >= max_theta1 && theta_global_rotation[0] > max_theta0) {
				theta_global_rotation[0] = min_theta0;
				theta_global_rotation[1] = min_theta1;
				theta_global_rotation[2] += step_theta;
				return;
			}
			if (theta_global_rotation[0] > max_theta0) {
				theta_global_rotation[0] = min_theta0;
				theta_global_rotation[1] += step_theta;
				return;
			}
		}
		//cout << std::setprecision(3) << epoch << ": " << theta_global_rotation[0] << "\t\t" << theta_global_rotation[1] << "\t\t" << theta_global_rotation[2] << endl;
		cout << "epoch = " << epoch << ", frame = " << worker->current_frame.id << endl;
		if (worker->settings->calibration_type == NONE) worker->calibration_finished = true;

		if (worker->settings->pause_tracking == true) { worker->offscreen_renderer.render_offscreen(true, false, false); worker->updateGL(); return; }

		if (worker->current_frame.id < 0) {
			worker->updateGL(); worker->current_frame.id = 0;
			if (worker->settings->show_iteration_calls) cout << "loading solutions" << endl;
			load_recorded_parameters(sequence_path + "solutions.txt");
		}

		/// Loading solutions 
		{
			Eigen::Matrix<Scalar, Eigen::Dynamic, 1> parameters = tracking_parameters[worker->current_frame.id];
			std::vector<float> theta = std::vector<float>(parameters.data(), parameters.data() + num_thetas);
			std::vector<float> beta = std::vector<float>(parameters.data() + num_thetas, parameters.data() + num_thetas + worker->model->num_betas);

			if (std::accumulate(beta.begin(), beta.end(), 0.0f) > 0) {
				worker->model->update_beta(beta);
			}
			std::copy(theta_global_rotation.begin(), theta_global_rotation.end(), theta.begin() + 3);
			static std::default_random_engine generator(worker->settings->random_seed);
			static std::uniform_real_distribution<> distribution(-M_PI / 8, M_PI / 8);
			if (worker->current_frame.id > 0) theta[3] += distribution(generator); theta[4] += distribution(generator); theta[5] += distribution(generator);
			//cout << std::setprecision(3) << epoch << ": " << theta[3] << "\t\t" << theta[4] << "\t\t" << theta[5] << endl;
			theta[1] += 30;
			theta[2] += 75;
			worker->model->update_theta(theta);
			worker->model->update_centers();
			worker->model->compute_outline();
		}

		/// Get global rotation
		{
			if (worker->current_frame.id == 0) {
				Mat3f R = worker->model->phalanges[0].global.block(0, 0, 3, 3);
				Vec3f a = R * Vec3f(1, 0, 0); Vec3f b = R * Vec3f(0, 1, 0); Vec3f c = R * Vec3f(0, 0, 1);
				cout << setprecision(2) << b[0] << "\t" << b[1] << "\t" << b[2] << endl;
				if (b[1] < 0) {
					worker->current_frame.id = sequence_length;
					epoch--;
					cout << "wrong rotation" << endl;
					return;
				}
			}
		}

		/// Rendering
		{
			if (worker->settings->show_iteration_calls) cout << "rendering" << endl;
			worker->offscreen_renderer.render_offscreen(true, false, false);
			worker->updateGL();
		}

		/// Write results
		{
			write_rendered_depth();
			write_joint_angles();
			write_joint_locations();
		}

		worker->current_frame.id += skip;

	}
};