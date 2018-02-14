#include "Worker.h"
#include "util/gl_wrapper.h"
#include "util/tictoc.h"

#include <QElapsedTimer>
#include <QGLWidget>
#include <QOpenGLWidget>

#include "tracker/ForwardDeclarations.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/TextureColor8UC3.h"
#include "tracker/Data/TextureDepth16UC1.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Energy/Energy.h"
#include "tracker/TwSettings.h"
#include "tracker/HModel/Model.h"

#include "tracker/HModel/JacobianVerifier.h"
#include "tracker/HModel/KalmanFilter.h"
#include "tracker/HModel/BatchSolver.h"
#include "tracker/HModel/GroundTruthLoader.h"

#include <map>

void Worker::updateGL() { 
	if (glarea != NULL) {
		//glarea->update();
		glarea->updateGL();
	}
}

void Worker::initialize(Camera *camera, GroundTruthLoader * ground_truth_loader, std::string data_path) {
	this->camera = camera;
	this->ground_truth_loader = ground_truth_loader;
	this->data_path = data_path;

	this->model = new Model();
	this->model->init(data_path, this->settings->sequence_path, this->settings->sequence_name, this->settings->calibrated_model_path, this->settings->calibration_type, this->settings->calibration_stage, this->settings->load_calibrated_model,
		this->E_shape._settings.enable_shape_prior, this->E_shape._settings.enable_shape_dofs_blockers, settings->fit_wrist_separately, settings->show_initialization_calls);
	if (settings->model_path.length() > 0) model->calibrated_model_path = settings->model_path;

	std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
	theta_initial[1] = -50; theta_initial[2] = 375; theta_initial[9] = 0.7; theta_initial[10] = 0.6;
	model->update_theta(theta_initial);
	model->update_centers();
	model->compute_outline();
	//model->manually_adjust_initial_transformations();	
	
	set_calibration_type(this->settings->calibration_type);
	set_calibration_stage(this->settings->calibration_stage);

	handfinder = new HandFinder(camera, settings->downsampling_factor, settings->fit_wrist_separately);
	ground_truth_loader->model = this->model;
	ground_truth_loader->camera = this->camera;

	if (settings->show_initialization_calls) cout << "finished initializing worker" << endl;

}

void Worker::init_graphic_resources() {
	offscreen_renderer.init(camera, model, data_path, E_fitting.settings->fit2D_silhouette_enable || E_fitting.settings->fit2D_outline_enable || settings->compute_rendered_metrics || settings->write_synthetic_depth);
	sensor_color_texture = new ColorTexture8UC3(camera->width(), camera->height());
	sensor_depth_texture = new DepthTexture16UC1(camera->width(), camera->height());

	using namespace energy;
	trivial_detector = new TrivialDetector(camera, &offscreen_renderer);

	jacobian_verifier = new JacobianVerifier(settings->verify_jacobian, model, settings->logs_path);

	E_fitting.init(this);

	E_limits.init(model);
	E_collision.init(model);
	E_pose.init(this);
	E_shape.init(model, &E_pose, settings->logs_path);
	E_temporal.init(model);
	E_fingertips.init(model, camera);
	E_damping.init(model);

	//cout << "finished initilizing worker" << endl;

	if (settings->perturb_template) {
		CalibrationType calibration_type = model->calibration_type;
		if (model->calibration_type == NONE) set_calibration_type(FULL);
		
		if (calibration_type == NONE) {
			for (size_t i = 0; i < num_phalanges; i++) if (i == 4 || i == 7 || i == 10 || i == 13) model->phalanges[i].init_local(2, 3) -= 3;
			for (size_t j = 0; j < model->phalanges[0].attachments.size(); j++) model->phalanges[0].offsets[j][2] = 0;
		}		
		
		{ /// A sequence of Anastasia or SomeoneElse
			if (settings->uniform_scaling_mean != 1.0f) {
				model->perturb_parameters(settings->random_seed, settings->uniform_scaling_mean, settings->perturb_template_std, 0.0);
			}
			else {
				model->perturb_parameters(settings->random_seed, settings->uniform_scaling_mean, settings->perturb_template_std, 0.0);//4.0
			}
		}
		
		model->beta_template = model->get_beta();
		model->phalanges_template = model->phalanges;
		E_shape.update_beta_template(model->beta_template);

		std::vector<float> theta_initial = std::vector<float>(num_thetas, 0);
		theta_initial[1] = -50; theta_initial[2] = 375; theta_initial[9] = 0.7; theta_initial[10] = 0.6;		
		model->update_theta(theta_initial);
		model->update_centers();
				
		//for (size_t i = 0; i < 72; i++) cout << model->beta_template[i] << " "; cout << endl;

		if (calibration_type == NONE) set_calibration_type(NONE);
	}

	if (settings->load_calibrated_model == false && settings->dataset_type == SRIDHAR) model->resize_model(1.45, 1.1, 1.1);	
	if (settings->load_calibrated_model == false && settings->dataset_type == TOMPSON) model->resize_model(1.45, 1.2, 1.2);	 

	if (settings->run_batch_solver) batch_solver = new BatchSolver(this);
	if (settings->run_kalman_filter) 
		kalman_filter = new KalmanFilter(model->calibration_type_to_beta_template_map[model->calibration_type], settings->kalman_filter_type, settings->kalman_filter_weight, E_shape.settings->enable_shape_dofs_blockers);

	if (settings->show_initialization_calls) cout << "finished initializing graphic resources" << endl;
}

void Worker::cleanup_graphic_resources() {
	delete sensor_color_texture;
	delete sensor_depth_texture;
	E_fitting.cleanup();
}

void write_linear_system(const LinearSystem & system, size_t iter, size_t frame_id, string energy_name, std::string logs_path) {
	cout << "system contains nans, writing to file" << endl;
	std::ostringstream stringstream;
	stringstream << "frame-" << frame_id << "-iter-" << iter;
	std::ofstream lhs_file(logs_path + "lhs-" + stringstream.str() + "-" + energy_name + ".txt"); if (lhs_file.is_open()) lhs_file << system.lhs;
	std::ofstream rhs_file(logs_path + "rhs-" + stringstream.str() + "-" + energy_name + ".txt"); if (rhs_file.is_open()) rhs_file << system.rhs;
}

void Worker::write_metrics_and_solutions() {
	if (!settings->write_every_iteration) return;

	static ofstream tracking_metrics_file(settings->sequence_path + settings->sequence_name + "online_continuous_metrics_iter.txt");
	if (tracking_metrics_file.is_open()) 
		tracking_metrics_file << tracking_error.pull_error << " " << tracking_error.push_error << endl;

	static ofstream weighted_metrics_file(settings->sequence_path + settings->sequence_name + "online_weighted_metrics_iter.txt");
	if (weighted_metrics_file.is_open())
		weighted_metrics_file << tracking_error.weighted_error << endl;


	static ofstream solutions_file(settings->sequence_path + settings->sequence_name + "solutions_iter.txt");
	if (solutions_file.is_open()) {		
		for (size_t i = 0; i < model->num_betas; i++) solutions_file << model->beta[i] << " ";
		solutions_file << endl;
	}

	if (settings->run_kalman_filter == true) {
		static ofstream solutions_kalman_file(settings->sequence_path + settings->sequence_name + "solutions_kalman_iter.txt");
		if (solutions_kalman_file.is_open()) {
			std::vector<float> beta_kalman = kalman_filter->get_estimate();
			for (size_t i = 0; i < model->num_betas; i++) solutions_kalman_file << beta_kalman[i] << " ";
			solutions_kalman_file << endl;
		}
	}	
}

Worker::~Worker() {
	delete trivial_detector;
	delete handfinder;
	delete model;
}

bool Worker::track_till_convergence() {	
	for (int i = 0; i < settings->termination_max_iters; ++i) {
		track(i);
	}	
	return monitor.is_failure_frame(tracking_error.pull_error, tracking_error.push_error, settings->dataset_type);
}

void Worker::track(int iter) {

	bool eval_error = (iter == settings->termination_max_iters - 1) || settings->write_every_iteration;
	bool rigid_only = (iter < settings->termination_max_rigid_iters);
	std::vector<float> theta = model->get_theta();
	std::vector<float> beta;
	if (model->num_betas > 0) beta = model->get_beta();
	else beta = std::vector<float>();

	model->serializer.serialize_model();

	if (E_fitting.settings->fit2D_silhouette_enable == true)
		offscreen_renderer.render_offscreen(iter == settings->termination_max_iters - 1, false, false);

	if (settings->verify_jacobian) {
		jacobian_verifier->theta_double = std::vector<double>(theta.begin(), theta.end());
		jacobian_verifier->beta_double = std::vector<double>(beta.begin(), beta.end());
	}	

	system.lhs = Matrix_MxN::Zero(model->num_parameters, model->num_parameters);
	system.rhs = VectorN::Zero(model->num_parameters);

	/// Build linear system

	E_fitting.track(current_frame, system, rigid_only, eval_error, settings->calibrate, tracking_error.push_error, tracking_error.pull_error, tracking_error.weighted_error, iter); 

	if (system.has_nan()) write_linear_system(system, iter, current_frame.id, "fitting", settings->logs_path);

	if (settings->run_kalman_filter && iter == settings->termination_max_iters - 1) {
		kalman_filter->set_measured_hessian(system.lhs.block(num_thetas, num_thetas, model->num_betas, model->num_betas),
			E_shape.shape_dofs_blockers_conditions, E_shape.shape_dofs_blockers_beta_sets, theta, E_pose.fingers_bending_latent_variable);
	}

	//E_collision.track(system);
	E_temporal.track(system, current_frame);
	E_limits.track(system, theta, beta);
	E_fingertips.track(system, current_frame);	
	E_damping.track(system);

	if (rigid_only) {
		energy::Energy::rigid_only(system);
	}
	else {
		ground_truth_loader->track(system, current_frame);
		E_pose.track(system, theta);
		E_shape.track(system, beta, iter);		
	}

	if (settings->run_kalman_filter && (settings->kalman_filter_type == EXTENDED || settings->kalman_filter_type == HYBRID) && settings->calibrate) {
		if (current_frame.id > 2 * settings->frames_interval_between_measurements) kalman_filter->track(system, beta);
	}

	if (settings->solve_linear_system == false) return;

	/// Solve 
	VectorN solution = energy::Energy::solve(system);
	if (system.has_nan() || energy::Energy::vector_has_nan(solution)) {
		write_linear_system(system, iter, current_frame.id, "all", settings->logs_path);
		return;
	}

	write_metrics_and_solutions();
	
	///  Update
	if (settings->calibrate && !rigid_only) {
		model->update_beta(solution);
		E_shape.update_beta_latent(solution);
		//E_shape.print_beta_latent();
	}
	theta = model->get_updated_parameters(theta, vector<float>(solution.data(), solution.data() + num_thetas));
	model->update_theta(theta);	
	model->update_centers();
	model->compute_outline();
	E_temporal.update(current_frame.id, theta);

	/// > Make a measurement
	if (iter != settings->termination_max_iters - 1) return;	
	if (current_frame.id % settings->frames_interval_between_measurements != 0) return;
	if (current_frame.id < 2 * settings->frames_interval_between_measurements) return;

	if (settings->run_kalman_filter && settings->calibrate) {
		//write_linear_system(system, iter, current_frame.id, "all");
		kalman_filter->update_estimate(beta, system.lhs.block(num_thetas, num_thetas, model->num_betas, model->num_betas));
		if (!settings->run_experiments) {
			//kalman_filter->display_several_measure_histories({3, 4, 5, 19});
			//kalman_filter->display_several_measure_histories({0, 1, 2, 40});
			//kalman_filter->display_estimate_history(2);
			//kalman_filter->display_covariances();
		}

		if (iter == settings->termination_max_iters - 1 && (settings->display_measured_certainty || settings->display_estimated_certainty)) {
			for (size_t i = 0; i < model->num_betas; i++) {
				if (settings->display_measured_certainty) model->beta_certainty[i] = kalman_filter->measured_hessian(i, i);
				if (settings->display_estimated_certainty) model->beta_certainty[i] = kalman_filter->estimated_hessian(i, i);
			}
		}
		
	}	
	if (settings->run_batch_solver) batch_solver->add_frame();
}

void Worker::set_calibration_type(CalibrationType new_calibration_type) { 
	if (settings->show_initialization_calls) cout << "setting calibration type to " << new_calibration_type << endl;
	
	if (new_calibration_type == NONE) {
		settings->calibrate = false;
	}
	else {
		settings->calibrate = true;
	}

	// TODO: remove this
	if (settings->calibrate == true) {		
		E_damping._settings.beta_phalange_length_damping = 8000;
		E_damping._settings.beta_finger_base_damping = 8000;
		E_damping._settings.beta_palm_center_damping = 50000;
		E_damping._settings.beta_radius_damping = 100000;

		E_damping._settings.abduction_damping = 150000;
		E_damping._settings.twist_damping = 1500000;
		E_damping._settings.top_phalange_damping = 3000;
		E_pose._settings.weight_proj = 7000;		
	}
	else { 
		E_damping._settings.abduction_damping = 1500000;
		E_damping._settings.top_phalange_damping = 10000;
		E_damping._settings.twist_damping = 3000000;
	}

	if (model->calibration_type == new_calibration_type) {
		if (settings->show_initialization_calls) cout << "calibration_type already was " << new_calibration_type << endl;
		return;
	}

	{
		if (settings->run_batch_solver) {
			if (batch_solver != NULL) model->update_beta(batch_solver->get_beta());
		}
		if (settings->run_kalman_filter) {
			model->update_beta(kalman_filter->get_estimate());
		}
		if (settings->use_online_beta) {
			model->update_beta(model->beta);
		}
		if (!settings->run_batch_solver && !settings->run_kalman_filter && !settings->use_online_beta && model->calibration_type != NONE) {
			model->update_beta(model->beta_template);
		}
		model->beta_template = model->calibration_type_to_beta_template_map[new_calibration_type];		
	}
	model->change_calibration_type(new_calibration_type);
	E_shape.update_beta_template(model->beta_template);	
}

void Worker::set_calibration_stage(CalibrationStage calibration_stage) {
	
	if (model->calibration_type == NONE) return;

	model->calibration_stage = calibration_stage;
	settings->calibration_stage = calibration_stage;
}