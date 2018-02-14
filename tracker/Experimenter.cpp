#include "tracker/Experimenter.h"
#include "tracker/Worker.h"
#include "tracker/HModel/BatchSolver.h"
#include "tracker/HModel/KalmanFilter.h"

#include "cudax/CudaHelper.h"
#include "cudax/CublasHelper.h"

#include <chrono>
#include <random>
#include <fstream>

/*Experimenter::Experimenter(Worker * worker, std::string sequence_path, ofstream * online_metrics_file, ofstream * marker_metrics_file) {
	this->worker = worker;
	this->sequence_path = sequence_path;
	this->online_metrics_file = online_metrics_file;
	this->marker_metrics_file = marker_metrics_file;

	estimation_type = (ESTIMATION_TYPE) 0;
	estimation_phase = CALIBRATION;

	std::cout << "loading sequence length" << endl;
	std::ifstream sequence_length_file(sequence_path + "sequence_length.txt");
	if (!sequence_length_file.is_open()) {
		cout << "cannot open sequence length file" << endl; exit(0);
	}
	std::string line;
	std::getline(sequence_length_file, line);
	sequence_length = std::stoi(line);
	sequence_length_file.close();	
	cout << "sequence_length = " << sequence_length << endl;
	sequence_length = 200;
}

void Experimenter::change_metrics_file_name(std::string & filename) {

	std::size_t found = filename.find(".txt");
	filename = filename.substr(0, found);
	if (found == std::string::npos) {
		std::cout << "file name does not contain .txt" << std::endl; exit(0);
	}

	if (estimation_type == BATCH_OFFLINE) {
		filename += "_batch_offline.txt";
	}
	if (estimation_type == BATCH_ONLINE) {
		filename += "_batch_online.txt";
	}
	if (estimation_type == KALMAN_STANDARD) {
		filename += "_kalman_standard.txt";
	}
	if (estimation_type == KALMAN_EXTENDED) {
		filename += "_kalman_extended.txt";		
	}
	if (estimation_type == INDEPENDENT) {
		filename += "_independent.txt";
	}
}

void Experimenter::run_sequence_for_evaluation() {
	worker->set_calibration_type(NONE);	

	std::vector<float> theta = std::vector<float>(num_thetas, 0);
	theta[1] = -50; theta[2] = 375; theta[9] = 0.7; theta[10] = 0.6;
	worker->model->update_theta(theta); worker->model->update_centers();

	worker->settings->run_batch_solver = false;
	worker->settings->run_kalman_filter = false;
	worker->E_fitting.settings->fit2D_weight = 0.4;
	worker->current_frame.id = 0;
	worker->settings->run_sequence_again_with_calibrated_model = false;

	worker->offscreen_renderer.render_offscreen(true, false, false); worker->updateGL(); Sleep(500);
}

void Experimenter::reinitialize_model() {
	worker->model->load_model_from_file(false);
	worker->model->update_parameters(std::vector<float>(num_thetas, 0));
	worker->model->initialize_offsets();
	worker->model->beta_template = worker->model->calibration_type_to_beta_template_map[worker->model->calibration_type];
	worker->E_shape.update_beta_template(worker->model->beta_template);
	worker->set_calibration_type(FULL);

	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[1] = -50; theta_initial[2] = 375; theta_initial[9] = 0.7; theta_initial[10] = 0.6;
	worker->model->update_theta(theta_initial);
	worker->model->update_centers();

	worker->model->perturb_parameters(worker->settings->random_seed, 0.5, 3.0);
	worker->model->beta_template = worker->model->get_beta();
	worker->E_shape.update_beta_template(worker->model->beta_template);
	worker->model->update_theta(theta_initial);
	worker->model->update_centers();

	if (worker->settings->use_online_betas_for_batch_solver == false) {
		worker->set_calibration_type(NONE);
	}

	worker->offscreen_renderer.render_offscreen(true, false, false); worker->updateGL(); Sleep(500);
}

void Experimenter::iterate_batch_solver() {
	size_t num_iterations;
	if (worker->settings->use_online_betas_for_batch_solver) num_iterations = 10;
	else num_iterations = 10;

	if (worker->model->calibration_type == NONE) worker->set_calibration_type(FULL);
	for (size_t i = 0; i < num_iterations; i++) worker->batch_solver->batch_solve_iteration();
}

void Experimenter::run_sequence_again_with_calibrated_model(Mode mode) {
	if (mode == LIVE)  return;

	if (worker->current_frame.id != 0 && worker->current_frame.id != sequence_length - 1) return;

	if (worker->current_frame.id == 0 && worker->settings->run_sequence_again_with_calibrated_model) {
		reinitialize_model();

		if (worker->settings->run_kalman_filter) {
			worker->kalman_filter = new KalmanFilter(worker->model->get_beta(), worker->settings->use_covariance, worker->settings->run_iekf,
				worker->settings->attract_to_kalman_parameters, worker->settings->attract_to_kalman_parameters_weight);
		}
		worker->offscreen_renderer.render_offscreen(true, false, false); worker->updateGL(); Sleep(500);
		return;
	}

	if (worker->current_frame.id == sequence_length - 1) {	
		if (worker->settings->run_batch_solver) iterate_batch_solver();
		this->online_metrics_file->close();
		this->marker_metrics_file->close();
		run_sequence_for_evaluation();
	}
}

void Experimenter::run_calibration_experiments(Mode mode) {
	if (mode == LIVE)  return;
	if (worker->current_frame.id != 0 && worker->current_frame.id != sequence_length - 1) return;

	if (worker->current_frame.id == sequence_length - 1) {

		if (estimation_type == last && estimation_phase == EVALUATION) {
			worker->settings->run_calibration_experiments = false;
			exit(0);
		}

		if (worker->settings->run_batch_solver && estimation_phase == CALIBRATION) iterate_batch_solver();

		if (estimation_phase == CALIBRATION) {
			estimation_phase = EVALUATION;
		}
		else {
			estimation_phase = CALIBRATION;			
			estimation_type = (ESTIMATION_TYPE) (estimation_type + 1);
		}
		this->online_metrics_file->close();
		this->marker_metrics_file->close();
		worker->current_frame.id = 0;
	}	

	std::cout << endl << "ESTIMATION_TYPE = " << estimation_type << ", ESTIMATION_PHASE = " << estimation_phase << endl << endl;

	if (worker->current_frame.id == 0) {		
		if (estimation_phase == EVALUATION) {
			run_sequence_for_evaluation();
		}
		if (estimation_phase == CALIBRATION) {	

			worker->E_fitting.settings->fit2D_weight = 1.5;
			worker->settings->run_batch_solver = false;
			worker->settings->use_online_betas_for_batch_solver = false;
			worker->settings->run_kalman_filter = false;
			worker->settings->run_iekf = false;
			worker->settings->use_online_beta = false;

			reinitialize_model();
			
			switch (estimation_type) {
			case BATCH_OFFLINE: {
				worker->settings->run_batch_solver = true;
				worker->settings->use_online_betas_for_batch_solver = false;
				worker->set_calibration_type(NONE);
				worker->batch_solver = new BatchSolver(worker);
				break;
			}
			case BATCH_ONLINE: {
				worker->settings->run_batch_solver = true;
				worker->settings->use_online_betas_for_batch_solver = true;
				worker->set_calibration_type(FULL);
				worker->batch_solver = new BatchSolver(worker);
				break;
			}
			case KALMAN_STANDARD: {
				worker->settings->run_kalman_filter = true;
				worker->settings->run_iekf = false;
				worker->set_calibration_type(FULL);
				worker->kalman_filter = new KalmanFilter(worker->model->get_beta(), worker->settings->use_covariance, worker->settings->run_iekf, 
					worker->settings->attract_to_kalman_parameters, worker->settings->attract_to_kalman_parameters_weight);		
				break;
			}
			case KALMAN_EXTENDED: {
				worker->settings->run_kalman_filter = true;
				worker->settings->run_iekf = true;
				worker->set_calibration_type(FULL);
				worker->kalman_filter = new KalmanFilter(worker->model->get_beta(), worker->settings->use_covariance, worker->settings->run_iekf,
					worker->settings->attract_to_kalman_parameters, worker->settings->attract_to_kalman_parameters_weight);
				break;
			}
			case INDEPENDENT: {
				worker->settings->use_online_beta = true;
				worker->set_calibration_type(FULL);
				break;
			}
			}
		}
	}
}*/