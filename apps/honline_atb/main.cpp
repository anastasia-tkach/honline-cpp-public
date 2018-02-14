#include <iostream>
#include <QApplication>

#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/Camera.h"

#include "tracker/Tracker.h"
#include "tracker/GLWidget.h"
//#include <vld.h>

#include <map>
#include <string>  
#include <chrono>
#include <random>

void parse_command_line(int argc, char* argv[], Worker & worker) {
	bool verbose = true;

	//for (size_t i = 0; i < argc; i++) cout << i << ": " << argv[i] << endl; 
	worker.settings->sequence_name = argv[1];
	if (verbose) cout << "sequence_name = " << worker.settings->sequence_name << endl;

	std::map<std::string, float> settings_map;
	for (size_t i = 2; i < argc; i += 2) {
		settings_map[std::string(argv[i])] = std::stof(argv[i + 1]);
	}

	//for (auto iterator : settings_map) cout << iterator.first + " " + to_string(iterator.second) << endl;

	if (settings_map.find("calibration_type") != settings_map.end()) {
		worker.settings->calibration_type = (CalibrationType)(int)settings_map["calibration_type"];
		if (verbose) cout << "calibration_type = " << worker.settings->calibration_type << endl;
	}
	if (settings_map.find("perturb_template") != settings_map.end()) {
		worker.settings->perturb_template = settings_map["perturb_template"];
		if (verbose) cout << "perturb_template = " << worker.settings->perturb_template << endl;
	}
	if (settings_map.find("load_calibrated_model") != settings_map.end()) {
		worker.settings->load_calibrated_model = settings_map["load_calibrated_model"];
		if (verbose) cout << "load_calibrated_model = " << worker.settings->load_calibrated_model << endl;
	}
	if (settings_map.find("random_seed") != settings_map.end()) {
		worker.settings->random_seed = settings_map["random_seed"];
		if (verbose) cout << "random_seed = " << worker.settings->random_seed << endl;
	}
	if (settings_map.find("uniform_scaling_mean") != settings_map.end()) {
		worker.settings->uniform_scaling_mean = settings_map["uniform_scaling_mean"];
		if (verbose) cout << "uniform_scaling_mean = " << worker.settings->uniform_scaling_mean << endl;
	}
	if (settings_map.find("sequence_length") != settings_map.end()) {
		worker.settings->sequence_length = settings_map["sequence_length"];
		if (verbose) cout << "sequence_length = " << worker.settings->sequence_length << endl;
	}

	if (settings_map.find("run_kalman_filter") != settings_map.end()) {
		worker.settings->run_kalman_filter = settings_map["run_kalman_filter"];
		if (verbose) cout << "run_kalman_filter = " << worker.settings->run_kalman_filter << endl;
	}
	if (settings_map.find("kalman_filter_type") != settings_map.end()) {
		worker.settings->kalman_filter_type = (KalmanType)(int)settings_map["kalman_filter_type"];
		if (verbose) cout << "kalman_filter_type = " << worker.settings->kalman_filter_type << endl;
	}
	if (settings_map.find("kalman_filter_weight") != settings_map.end()) {
		worker.settings->kalman_filter_weight = settings_map["kalman_filter_weight"];
		if (verbose) cout << "kalman_filter_weight = " << worker.settings->kalman_filter_weight << endl;
	}
	if (settings_map.find("perturb_template_std") != settings_map.end()) {
		worker.settings->perturb_template_std = settings_map["perturb_template_std"];
		if (verbose) cout << "perturb_template_std = " << worker.settings->perturb_template_std << endl;
	}
	if (settings_map.find("run_batch_solver") != settings_map.end()) {
		worker.settings->run_batch_solver = settings_map["run_batch_solver"];
		if (verbose) cout << "run_batch_solver = " << worker.settings->run_batch_solver << endl;
	}
	if (settings_map.find("use_online_betas_for_batch_solver") != settings_map.end()) {
		worker.settings->use_online_betas_for_batch_solver = settings_map["use_online_betas_for_batch_solver"];
		if (verbose) cout << "use_online_betas_for_batch_solver = " << worker.settings->use_online_betas_for_batch_solver << endl;
	}
	if (settings_map.find("use_online_beta") != settings_map.end()) {
		worker.settings->use_online_beta = settings_map["use_online_beta"];
		if (verbose) cout << "use_online_beta = " << worker.settings->use_online_beta << endl;
	}
}

int main(int argc, char* argv[]) {

	bool benchmark, playback, real_color;
	Worker worker;

	worker.settings->sequence_path = "C:/Data/sensor-sequences/";
	worker.settings->data_path = "C:/Developer/honline-cpp/data/";
	worker.settings->calibrated_model_path = "C:/Data/honline-results/calibrated-models/experiments/";
	worker.settings->sequence_name = "experiment/";
	worker.settings->logs_path = "C:/Users/tkach/Desktop/Test/";

	/// Algorithm 		
	{
		benchmark = false;
		playback = false;
		real_color = false;
		worker.settings->model_path = "";
		worker.settings->sequence_length = -1;
		worker.settings->display_estimated_certainty = true;
		worker.settings->display_measured_certainty = false;
		worker.settings->downsampling_factor = 2;
		worker.settings->report_times = false;
		worker.settings->show_initialization_calls = false;
		worker.settings->show_iteration_calls = false;
		worker.settings->stop_tracking_without_wristband = true;
		worker.settings->fit_wrist_separately = true;
		worker.settings->random_seed = 5001;
		// std::chrono::system_clock::now().time_since_epoch().count() % RAND_MAX;
		worker.settings->dataset_type = TKACH;
		worker.settings->calibration_type = FULL;
	}
	/// Estimations 
	{
		worker.settings->kalman_filter_type = STANDARD;
		worker.settings->run_kalman_filter = true;
		worker.settings->kalman_filter_weight = 10;
		worker.settings->frames_interval_between_measurements = 20;// 20;

		worker.settings->run_batch_solver = false;
		worker.settings->use_online_betas_for_batch_solver = false;

		worker.settings->use_online_beta = false;

		worker.settings->perturb_template = false;
		worker.settings->perturb_template_std = 0.15;
		worker.settings->load_calibrated_model = false;
	}
	/// Evaluation
	{
		worker.settings->write_synthetic_depth = false; // comment "break" in FB_fshader
		worker.settings->write_marker_positions = false; /// write number of frames at the beginning
		worker.settings->write_estimated_certainties = false;
		worker.settings->write_every_iteration = false;
		worker.settings->compute_rendered_metrics = false;
		worker.settings->compute_marker_based_metrics = false;
		worker.settings->compute_weighted_metrics = false;
		worker.settings->use_synthetic_depth = false;
	}
	/// Debug
	{
		worker.settings->verify_jacobian = false;
		worker.settings->multiply_on_cpu = false;
		worker.E_fitting.settings->write_jacobian_to_file = false;
	}
	/// Optimization 	 
	{
		worker.settings->termination_max_iters = 7;

		worker.E_fitting.settings->fit2D_outline_enable = true;
		worker.E_fitting.settings->fit2D_silhouette2outline_enable = false;
		worker.E_fitting.settings->fit2D_silhouette_enable = false;
		worker.E_fitting.settings->fit2D_unproject = true;
		worker.E_fitting.settings->fit2D_weight = 1.5;

		worker.E_fitting.settings->fit2D_weight_segment = 1.2;
		worker.E_fitting.settings->fit2D_weight_palm = 2.0;
		worker.E_fitting.settings->disable_finger_bases_fitting = false;

		worker.E_fitting.settings->fit3D_enable = true;

		worker.E_limits.jointlimits_enable = true;

		worker.E_pose._settings.enable_split_pca = true;
		worker.E_pose._settings.weight_proj = 4 * 10e2;

		worker.E_shape._settings.enable_shape_prior = true;
		worker.E_shape._settings.weight_uniform = 5;
		worker.E_shape._settings.weight_palm_width = 0;
		worker.E_shape._settings.weight_palm_height = 0;
		worker.E_shape._settings.weight_radii = 1000;
		worker.E_shape._settings.enable_additional_constraints = true;
		worker.E_shape._settings.enable_semantic_limits = true;
		worker.E_shape._settings.enable_shape_dofs_blockers = false;

		worker.E_collision._settings.collision_enable = false;
		worker.E_collision._settings.collision_weight = 1e3;

		worker.E_fingertips._settings.enable_fingertips_prior = true;

		worker.settings->termination_max_rigid_iters = 1;

	}

	{ /// If the application is run externaly
		if (argc > 1) {
			parse_command_line(argc, argv, worker);
			worker.settings->run_experiments = false;
			//worker.settings->sequence_length = 300;
			benchmark = true;
		}

		if (worker.settings->dataset_type == TOMPSON || worker.settings->dataset_type == SHARP) worker.settings->downsampling_factor = 1;
		if (worker.settings->calibration_type == NONE) 	worker.E_fitting.settings->fit2D_weight = 0.4;
		if (worker.settings->calibration_type == NONE && worker.settings->dataset_type == TKACH) worker.E_fingertips._settings.enable_fingertips_prior = false;
		else worker.E_fitting.settings->fit2D_weight = 1.5;
		if (worker.settings->sequence_name.compare("teaser_short/") == 0 && worker.settings->calibration_type == NONE) {
			worker.settings->termination_max_iters = 8;
		}
		if (worker.settings->dataset_type == SHARP) worker.E_fitting.settings->undistort = true;
		if (worker.settings->fit_wrist_separately) worker.E_fitting.settings->fit_wrist_separately = true;
		if (worker.settings->dataset_type != TKACH) worker.E_fitting.settings->dataset_type = worker.settings->dataset_type;

		worker.E_temporal._settings.temporal_coherence1_enable = true;
		worker.E_temporal._settings.temporal_coherence2_enable = true;
		if (worker.settings->dataset_type == TKACH && worker.settings->calibration_type == NONE) {
			worker.E_temporal._settings.temporal_coherence1_weight = 0.05f; worker.E_temporal._settings.temporal_coherence2_weight = 0.05f;
		}
		if (worker.settings->dataset_type == TKACH && worker.settings->calibration_type == FULL) {
			worker.E_temporal._settings.temporal_coherence1_weight = 0.2f; worker.E_temporal._settings.temporal_coherence2_weight = 0.2f;
		}
		if (worker.settings->dataset_type == TOMPSON) {
			worker.E_temporal._settings.temporal_coherence1_weight = 5.0f; worker.E_temporal._settings.temporal_coherence2_weight = 5.0f;
			worker.E_pose._settings.weight_proj = 7000;
			worker.settings->termination_max_iters = 15;
			worker.E_shape._settings.enable_shape_dofs_blockers = true;
			worker.E_shape._settings.weight_uniform = 10;
			worker.E_shape._settings.weight_radii = 3000;
			worker.settings->frames_interval_between_measurements = 30;
		}
		if (worker.settings->dataset_type == SRIDHAR) {
			worker.settings->termination_max_iters = 30;
			worker.E_fitting.settings->fit2D_weight = 0.4;
			worker.E_fitting.settings->fit2D_outline_enable = true;
			worker.E_fingertips._settings.enable_fingertips_prior = true;
			worker.E_temporal._settings.temporal_coherence1_weight = 0.05f;
			worker.E_temporal._settings.temporal_coherence2_weight = 0.05f;
			worker.E_pose._settings.weight_proj = 10000;
			worker.E_damping._settings.abduction_damping = 5000000;
			worker.E_shape._settings.weight_uniform = 50;
		}
	}

	Q_INIT_RESOURCE(shaders);
	QApplication app(argc, argv);

	Camera camera(worker.settings->dataset_type, 60);
	SensorRealSense sensor(&camera, real_color, worker.settings->downsampling_factor, worker.settings->fit_wrist_separately);

	DataStream datastream(&camera);
	SolutionStream solutions;
	GroundTruthLoader ground_truth_loader(worker.settings->dataset_type, worker.settings->fit_wrist_separately, worker.settings->sequence_path + worker.settings->sequence_name + "/");

	worker.initialize(&camera, &ground_truth_loader, worker.settings->data_path);

	worker.handfinder->settings->wrist_band_length = 10;
	worker.model->membranes_fractional_length = { 0.40f, 0.45f, 0.51f, 0.58f };

	GLWidget glwidget(&worker, &datastream, &solutions, playback, real_color, worker.settings->data_path, worker.settings->sequence_path + worker.settings->sequence_name + "/");

	worker.bind_glwidget(&glwidget);
	if (!worker.settings->run_experiments)
		glwidget.show();

	if (worker.settings->use_synthetic_depth) worker.settings->sequence_name = worker.settings->sequence_name + "synthetic/";
	Tracker tracker(&worker, &glwidget, camera.FPS(), worker.settings->sequence_path + worker.settings->sequence_name + "/", real_color);
	tracker.sensor = &sensor;
	tracker.datastream = &datastream;
	tracker.solutions = &solutions;

	tracker.toggle_tracking(!benchmark && !playback);
	tracker.toggle_benchmark(benchmark);
	tracker.toggle_playback(playback);

	if (worker.model->calibration_type == FULL) {
		worker.model->pose_dofs[7].min = -0.25;
		worker.model->pose_dofs[8].min = -0.4;
	}

	//worker.model->set_initial_pose(); worker.model->write_model(worker.settings->sequence_path + worker.settings->sequence_name);

	//if (worker.settings->show_initialization_calls) cout << "before app.exec" << endl;
	return app.exec();
}

