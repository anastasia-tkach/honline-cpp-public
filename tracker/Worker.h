#pragma once
#include "util/gl_wrapper.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "OpenGL/OffscreenRenderer.h"
#include "Data/DataFrame.h"
#include "Energy/JointLimits.h"
#include "Energy/Damping.h"
#include "Energy/Collision.h"
#include "Energy/PoseSpace.h"
#include "Energy/ShapeSpace.h"
#include "Energy/Fingertips.h"
#include "Energy/Fitting.h"
#include "Energy/Fitting/TrackingMonitor.h"
#include "Energy/Temporal.h"

#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow

/// @note do not construct more than one instance of this class

class Worker {

public:
	struct Settings {	
		
		bool report_times = false;
		bool restart_clock = false;
		bool show_initialization_calls = false;
		bool show_iteration_calls = false;
		bool stop_tracking_without_wristband = false;

		bool calibrate = false;
		bool load_calibrated_model = false;
		bool perturb_template = false;
		float perturb_template_std = 0.0;
		bool run_experiments = false;
		bool fit_wrist_separately = false;
		unsigned random_seed = 0;
		float uniform_scaling_mean = 1.0;

		DatasetType dataset_type = TKACH;
		CalibrationType calibration_type = NONE;
		CalibrationStage calibration_stage = DEFAULT;
		
		bool verify_jacobian = false;
		bool use_synthetic_depth = false;
		bool use_online_beta = false;
		bool multiply_on_cpu = false;
		bool display_measured_certainty = false;
		bool display_estimated_certainty = false;

		bool run_batch_solver = false;
		bool use_online_betas_for_batch_solver = false;

		KalmanType kalman_filter_type = STANDARD;
		bool run_kalman_filter = false;
		float kalman_filter_weight = 10;
		int frames_interval_between_measurements = 60;

		bool solve_linear_system = true;

		bool write_synthetic_depth = false;
		bool write_marker_positions = false;
		bool write_estimated_certainties = false;
		bool write_every_iteration = false;
		bool compute_rendered_metrics = false;
		bool compute_marker_based_metrics = false;
		bool compute_weighted_metrics = false;

		bool pause_tracking = false;
		bool pause_current_frame = false;

		int termination_max_iters = 6;
		int termination_max_rigid_iters = 1;
		int downsampling_factor = 2;
		int sequence_length = -1;

		std::string model_path = "";
		std::string calibrated_model_path = "";
		std::string sequence_path = "";
		std::string data_path = "";
		std::string sequence_name = "";
		std::string logs_path = "";

	} _settings;
	Settings*const settings = &_settings;

public:
	//QOpenGLWidget* glarea = NULL; void bind_glwidget(QOpenGLWidget* glarea) { this->glarea = glarea; }

	QGLWidget* glarea = NULL; void bind_glwidget(QGLWidget* glarea) { this->glarea = glarea; }

	void updateGL();

public:
	std::string data_path;
	bool calibration_finished = false;

	Camera* camera = NULL;
	Model * model;
	DataFrame current_frame = DataFrame(-1);
	TrackingError tracking_error;

	DepthTexture16UC1* sensor_depth_texture = NULL;
	ColorTexture8UC3* sensor_color_texture = NULL;

	LinearSystem system;

	energy::Fitting E_fitting;
	energy::Temporal E_temporal;
	energy::Damping E_damping;
	energy::JointLimits E_limits;
	energy::Collision E_collision;
	energy::PoseSpace E_pose;
	energy::ShapeSpace E_shape;
	//energy::Fingertips E_fingertips;
	Fingertips E_fingertips;

	HandFinder* handfinder = NULL;
	TrivialDetector* trivial_detector = NULL;
	OffscreenRenderer offscreen_renderer;
	TrackingMonitor monitor;
	JacobianVerifier * jacobian_verifier = NULL;
	KalmanFilter * kalman_filter = NULL;
	BatchSolver * batch_solver = NULL;
	GroundTruthLoader * ground_truth_loader = NULL;

public:
	Worker() {}
	~Worker();
	// the two below functions are needed because worker has a pointer to glarea and glarea has a pointer to worker
	void initialize(Camera *camera, GroundTruthLoader * ground_truth_loader, std::string data_path);
	void init_graphic_resources(); 
	void cleanup_graphic_resources();

	void write_metrics_and_solutions();
	void track(int iter);
	bool track_till_convergence();

	void set_calibration_type(CalibrationType new_calibration_type);
	void set_calibration_stage(CalibrationStage new_calibration_stage);
};
