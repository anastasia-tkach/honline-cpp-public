#include "tracker/Worker.h"

#include <iostream>
#include <windows.h>
#include <conio.h>
#include <map>
#include <string>
#include <algorithm>
#include <iterator>
#include <cstdlib>
#include <chrono>
#include <random>

std::string path_to_exe = "C:\\Developer\\builds\\honline-cpp-experiments-build\\honline_atb.exe";
std::string source_path = "C:\\Data\\sensor-sequences\\";
std::string log_path = "C:\\Users\\tkach\\Desktop\\Test\\";
std::string full_metric_filename = "online_continuous_metrics_exp.txt";
std::string weighted_metric_filename = "online_weighted_metrics_exp.txt";
std::string marker_based_metrics_filename = "marker_based_metrics_exp.txt";
std::string rendered_metric_filename = "online_rendered_metrics.txt";

std::string solutions_filename = "solutions.txt";
std::string full_metric_iter_filename = "online_continuous_metrics_iter.txt";

std::string betas_estimated_filename = "betas_estimated.txt";

std::string target_path = "C:\\Data\\honline-results\\teaser_pert0.2_s7000_m0.1_w200_i_30_prior10\\";
std::string target_series = "";

bool synthetic_depth = false;

unsigned random_seed = 0;
std::string sequence_name = "";
float kalman_filter_weight = 0;
float perturb_template_std = 0;
float uniform_scaling_mean = 1.0f;
int calibration_sequence_length = -1;

void move_evaluation_files(std::string target_filename_prefix, bool copy_calibrated_model) {
	std::string source_address, target_address;

	std::string target_filename = target_filename_prefix + "_" + std::to_string(perturb_template_std) + "_" + sequence_name + "_" + std::to_string(random_seed) + "_" + target_series;

	
	{ /// Full metrics
		source_address = source_path + sequence_name + "\\" + full_metric_filename;
		target_address = target_path + target_filename + ".txt";
		//cout << "source_address = " << source_address << endl;
		//cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}

	{ /// Rendered metrics
		source_address = source_path + sequence_name + "\\" + rendered_metric_filename;
		target_address = target_path + target_filename + "_rendered.txt";
		//cout << "source_address = " << source_address << endl;
		//cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}

	/*{ /// Weighted metrics
		source_address = source_path + sequence_name + "\\" + weighted_metric_filename;
		target_address = target_path + target_filename + "_weighted.txt";
		//cout << "source_address = " << source_address << endl;
		//cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Marker based metrics
		source_address = source_path + sequence_name + "\\" + marker_based_metrics_filename;
		target_address = target_path + target_filename + "_markers.txt";
		cout << "source_address = " << source_address << endl;
		cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Full metrics each iteration
		source_address = source_path + sequence_name + "\\" + "online_continuous_metrics_iter.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\online_continuous_metrics_iter.txt";
		target_address = target_path + target_filename +  "_metrics-iter.txt";
		cout << "source_address = " << source_address << endl;
		cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Solutions iter
		source_address = source_path + sequence_name + "\\" + "solutions_iter.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\solutions_iter.txt";
		target_address = target_path + target_filename + "_solutions_iter.txt";
		cout << "source_address = " << source_address << endl;
		cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Solutions kalman iter
		source_address = source_path + sequence_name + "\\" + "solutions_kalman_iter.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\solutions_kalman_iter.txt";
		target_address = target_path + target_filename + "_solutions-kalman-iter.txt";
		cout << "source_address = " << source_address << endl;
		cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Solutions 
		source_address = source_path + sequence_name + "\\" + solutions_filename;
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\" + solutions_filename;
		target_address = target_path + target_filename + "_solutions.txt";
		cout << "source_address = " << source_address << endl;
		cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Betas estimated
		source_address = source_path + sequence_name + "\\" + betas_estimated_filename;
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\" + betas_estimated_filename;
		target_address = target_path + target_filename + "_beta-estimated.txt";
		cout << "source_address = " << source_address << endl;
		cout << "target_address = " << target_address << endl;
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	/*{ /// Measured and estimated
		source_address = source_path + sequence_name + "\\estimated_certainties.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\estimated_certainties.txt";
		target_address = target_path + target_filename + "_estimated-certainties.txt";
		std::system(("move " + source_address + " " + target_address).c_str());

		source_address = source_path + sequence_name + "\\measured_certainties.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\measured_certainties.txt";
		target_address = target_path + target_filename + "_measured-certainties.txt";
		std::system(("move " + source_address + " " + target_address).c_str());

		source_address = source_path + sequence_name + "\\estimated_values.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\estimated_values.txt";
		target_address = target_path + target_filename + "_estimated-values.txt";
		std::system(("move " + source_address + " " + target_address).c_str());

		source_address = source_path + sequence_name + "\\measured_values.txt";
		if (synthetic_depth) source_address = source_path + sequence_name + "\\synthetic\\measured_values.txt";
		target_address = target_path + target_filename + "_measured-values.txt";
		std::system(("move " + source_address + " " + target_address).c_str());
	}*/

	if (copy_calibrated_model) {
		CreateDirectory((target_path + target_filename).c_str(), NULL);
		std::string calibrated_model_path = "E:\\Data\\honline-results\\calibrated_models\\experiments\\";
		source_address = calibrated_model_path + "C.txt";
		target_address = target_path + target_filename + "\\C.txt";
		std::system(("copy " + source_address + " " + target_address).c_str());

		source_address = calibrated_model_path + "B.txt";
		target_address = target_path + target_filename + "\\B.txt";
		std::system(("copy " + source_address + " " + target_address).c_str());

		source_address = calibrated_model_path + "R.txt";
		target_address = target_path + target_filename + "\\R.txt";
		std::system(("copy " + source_address + " " + target_address).c_str());

		source_address = calibrated_model_path + "I.txt";
		target_address = target_path + target_filename + "\\I.txt";
		std::system(("copy " + source_address + " " + target_address).c_str());
	}	
}

std::map<std::string, double> get_default_calibration_settings() {
	std::map<std::string, double> worker_settings;
	worker_settings["calibration_type"] = 4;
	worker_settings["perturb_template"] = true; 
	worker_settings["load_calibrated_model"] = false;
	worker_settings["random_seed"] = random_seed;
	worker_settings["uniform_scaling_mean"] = uniform_scaling_mean;
	worker_settings["sequence_length"] = calibration_sequence_length;

	worker_settings["run_batch_solver"] = false;
	worker_settings["use_online_betas_for_batch_solver"] = false;
	worker_settings["run_kalman_filter"] = false;
	worker_settings["kalman_filter_type"] = 0;
	worker_settings["kalman_filter_weight"] = kalman_filter_weight;
	worker_settings["perturb_template_std"] = perturb_template_std;
	worker_settings["use_online_beta"] = false;

	return worker_settings;
}

std::map<std::string, double> get_default_evaluation_settings() {
	std::map<std::string, double> worker_settings;
	worker_settings["calibration_type"] = 0;
	worker_settings["perturb_template"] = false;
	worker_settings["load_calibrated_model"] = true;
	worker_settings["random_seed"] = 0.0f;
	worker_settings["uniform_scaling_mean"] = 1.0f;
	worker_settings["sequence_length"] = -1.0f;

	worker_settings["run_batch_solver"] = false;
	worker_settings["use_online_betas_for_batch_solver"] = false;
	worker_settings["run_kalman_filter"] = false;
	worker_settings["kalman_filter_type"] = 0;
	worker_settings["kalman_filter_weight"] = kalman_filter_weight;
	worker_settings["perturb_template_std"] = 0;
	worker_settings["use_online_beta"] = false;

	return worker_settings;
}

void run_experiment(const std::map<std::string, double> & worker_settings, std::string experiment_stage, std::string experiment_type) {
	std::string command_line = path_to_exe + " " + sequence_name + "\\";
	for (auto iterator : worker_settings) command_line += " " + iterator.first + " " + to_string(iterator.second);
	cout << "   " + experiment_stage << endl;
	/// Add logging
	//std::string log_filename = log_path + experiment_type + "_" + experiment_stage + "_" + std::to_string((int)kalman_filter_weight) + "_" + sequence_name + "_" + std::to_string(random_seed) + ".txt";
	//command_line += " > " + log_filename;
	cout << command_line << endl;
	system(command_line.c_str());
}

void run_honline_and_move_files(const std::map<std::string, double> & calibration_settings, std::string experiment_type) {

	run_experiment(calibration_settings, "CALIBRATION", experiment_type);
	move_evaluation_files(experiment_type + "_CALIBRATION", false);
	run_experiment(get_default_evaluation_settings(), "EVALUATION", experiment_type);
	move_evaluation_files(experiment_type + "_EVALUATION", false);
}

void repetivite_task_on_several_sequences() {
	std::vector<std::string> sequence_names = { "andrii", "edoardo", "fabrice", "filippe", "jacomo", "jan", "luca", "madeleine", "mina", "pier", "stefano", "timur"};
	for (size_t sequence_index = 0; sequence_index < sequence_names.size(); sequence_index++) {
		sequence_name = sequence_names[sequence_index];
		std::cout << "sequence_name = " << sequence_name << endl;
		std::map<std::string, double> worker_settings;

		// find model
		/*worker_settings = get_default_calibration_settings();
		worker_settings["perturb_template"] = false;
		worker_settings["run_kalman_filter"] = true;
		worker_settings["kalman_filter_type"] = STANDARD;
		worker_settings["sequence_length"] = 300;
		run_experiment(worker_settings, "CALIBRATION", "");*/

		// find solutions
		worker_settings = get_default_evaluation_settings();
		run_experiment(worker_settings, "EVALUATION", "");
	}
}

void main(int argc, char* argv[]) {

	repetivite_task_on_several_sequences();
	return;

	std::map<std::string, double> worker_settings;

	target_series = "";

	std::vector<float> kalman_filter_weights = { 200.0f };
	//std::vector<float> perturb_template_stds = { 0.025f, 0.05f, 0.075f, 0.10f, 0.125f, 0.15f, 0.175f, 0.20f, 0.225f, 0.25f, 0.275f, 0.30f, 0.325f, 0.35f, 0.375f, 0.40f};
	//std::vector<float> perturb_template_stds = { 0.025f, 0.075f, 0.125f, 0.175f, 0.225f, 0.275f, 0.325f, 0.375f };
	//std::vector<float> perturb_template_stds = { 0.05f, 0.10f, 0.15f, 0.20f, 0.25f, 0.30f, 0.35f, 0.40f };
	//std::vector<float> perturb_template_stds = { 0.10f, 0.20f, 0.30f, 0.40f};
	std::vector<float> perturb_template_stds = { 0.20f };
	std::vector<std::string> sequence_names = { "teaser"};
	std::vector<float> uniform_scaling_means = { 1.0f };
	std::vector<unsigned> random_seeds = { 5001 /*0.8*/, 1556 /*1.5*/, 9247 /*0.97*/, 24917 /*0.15*/, 3804/*1.26*/, 25565/*1.12*/, 14995/*0.96*/, 23874/*1.2*/, 29237/*1.43*/, 18269/*1.7*/ };
	//std::vector<unsigned> random_seeds = { 23874/*1.27*/, 5001 /*0.74*/, 9247 /*0.968*/, 24917 /*0.3*/, 3804/*1.35*/, 25565/*1.16*/, 14995/*0.95*/, 29237/*1.58*/, 98743/*0.77*/, 79086 /*0.99*/, 6537 /*0.3*/, 78907/*1.2*/, 45675/*1.14*/, 34652 /*1.11*/, 46787 /*0.9*/ };

	for (size_t sequence_index = 0; sequence_index < sequence_names.size(); sequence_index++) {
		sequence_name = sequence_names[sequence_index];
		uniform_scaling_mean = uniform_scaling_means[sequence_index];

		for (size_t random_seed_index = 0; random_seed_index < 1; random_seed_index++) {

			//random_seed = std::chrono::system_clock::now().time_since_epoch().count() % RAND_MAX;
			random_seed = random_seeds[random_seed_index];
			kalman_filter_weight = 0;

			for (size_t perturb_template_std_index = 0; perturb_template_std_index < perturb_template_stds.size(); perturb_template_std_index++) {
				perturb_template_std = perturb_template_stds[perturb_template_std_index];

				std::cout << endl << "random_seed = " << random_seed << ", perturb_template_std = " << perturb_template_std << endl;

				/*{ /// Template
					std::string experiment_type = "TEMPLATE";
					cout << endl << experiment_type << endl;
					worker_settings = get_default_evaluation_settings();
					worker_settings["perturb_template"] = false;
					worker_settings["load_calibrated_model"] = false;
					run_experiment(worker_settings, "EVALUATION", experiment_type);
					move_evaluation_files(experiment_type, false);
				*/

				/*{ /// Original
					std::string experiment_type = "ORIGINAL";
					cout << endl << experiment_type << endl;
					worker_settings = get_default_evaluation_settings();
					worker_settings["perturb_template"] = true;
					worker_settings["perturb_template_std"] = perturb_template_std;
					worker_settings["load_calibrated_model"] = false;
					run_experiment(worker_settings, "EVALUATION", experiment_type);
					move_evaluation_files(experiment_type, false);
				}*/

				{ /// Kalman Standart
					std::string experiment_type = "KALMAN_STANDARD";
					cout << endl << experiment_type << endl;
					worker_settings = get_default_calibration_settings();
					worker_settings["run_kalman_filter"] = true;
					worker_settings["kalman_filter_type"] = STANDARD;
					run_honline_and_move_files(worker_settings, experiment_type);
				}

				/*{ /// Online
					std::string experiment_type = "ONLINE";
					cout << endl << experiment_type << endl;
					worker_settings = get_default_calibration_settings();
					worker_settings["use_online_beta"] = true;
					run_honline_and_move_files(worker_settings, experiment_type);
				}*/

				for (size_t kalman_filter_weight_index = 0; kalman_filter_weight_index < kalman_filter_weights.size(); kalman_filter_weight_index++) {
					kalman_filter_weight = kalman_filter_weights[kalman_filter_weight_index];
					cout << "kalman_filter_weight = " << kalman_filter_weight << endl;

					{ /// Kalman Extended
						std::string experiment_type = "KALMAN_EXTENDED";
						cout << endl << experiment_type << endl;
						worker_settings = get_default_calibration_settings();
						worker_settings["run_kalman_filter"] = true;
						worker_settings["kalman_filter_type"] = EXTENDED;
						run_honline_and_move_files(worker_settings, experiment_type);
					}
				}

				kalman_filter_weight = 0;

				{ /// Batch offline
					std::string experiment_type = "BATCH_OFFLINE";
					cout << endl << experiment_type << endl;
					worker_settings = get_default_calibration_settings();
					worker_settings["run_batch_solver"] = true;
					worker_settings["calibration_type"] = 0;
					run_honline_and_move_files(worker_settings, experiment_type);
				}

				{ /// Batch online
					std::string experiment_type = "BATCH_ONLINE";
					cout << endl << experiment_type << endl;
					worker_settings = get_default_calibration_settings();
					worker_settings["run_batch_solver"] = true;
					worker_settings["use_online_betas_for_batch_solver"] = true;
					run_honline_and_move_files(worker_settings, experiment_type);
				}

				/*{ /// Kalman Uniform
				std::string experiment_type = "KALMAN_UNIFORM";
				cout << endl << experiment_type << endl;
				worker_settings = get_default_calibration_settings();
				worker_settings["run_kalman_filter"] = true;
				worker_settings["kalman_filter_type"] = UNIFORM;
				run_honline_and_move_files(worker_settings, experiment_type);
				}*/

				/*{ /// Kalman Diagonal
				std::string experiment_type = "KALMAN_DIAGONAL";
				cout << endl << experiment_type << endl;
				worker_settings = get_default_calibration_settings();
				worker_settings["run_kalman_filter"] = true;
				worker_settings["kalman_filter_type"] = DIAGONAL;
				run_honline_and_move_files(worker_settings, experiment_type);
				}*/

				/*for (size_t perturb_template_std_index = 0; perturb_template_std_index < perturb_template_stds.size(); perturb_template_std_index++) {
				perturb_template_std = perturb_template_stds[perturb_template_std_index];
				cout <<<< endl;
				{ /// Online calibration
				std::string experiment_type = "ONLINE";
				cout << endl << experiment_type << endl;
				worker_settings = get_default_calibration_settings();
				worker_settings["use_online_beta"] = true;
				run_experiment(worker_settings, "CALIBRATION", experiment_type);
				move_evaluation_files(experiment_type, false);
				}
				}*/
			}
			
		}
	}
	
}

