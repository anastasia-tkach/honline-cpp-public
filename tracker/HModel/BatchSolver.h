#include "tracker/Types.h"
#include "tracker/ForwardDeclarations.h"
#include <vector>
#include "util/opencv_wrapper.h"

class BatchSolver {
	Worker * worker;
	std::vector<cv::Mat> data_images;
	std::vector<std::vector<float>> thetas;
	std::vector<std::vector<float>> betas;
	std::vector<std::vector<float>> betas_latent;
	std::vector<float> beta;
	std::vector<float> beta_common;
	std::vector<float> beta_common_latent;
	std::vector<cv::Mat> sensor_silhoettes;
	std::vector<cv::Mat> sensor_silhoettes_closed;
	std::vector<cv::Mat> sensor_outlines;
	std::vector<std::vector<Vector2>> data_fingertips_2d;
	cv::Mat pose_icons;
	string window_name = "poses";
	int displayed_pose_index = -1;
	int previous_pose_index = -1;
	size_t num_poses = 0;
	int counter = 0;

	bool use_online_betas = false;
	float weight = 10000;

	void apply_pose(int pose_index);

	cv::Mat get_current_pose_icon();	

	void parse_parameters_for_offline_beta(const VectorN & solution);

	void parse_parameters_for_online_betas(const VectorN & solution);

	LinearSystem build_batch_system_for_offline_beta(const std::vector<LinearSystem> & systems);

	LinearSystem build_batch_system_for_online_betas(const std::vector<LinearSystem> & systems);

	void solve_batch_system();

public:
	bool first_batch_solve_iteration = true;

	BatchSolver(Worker * worker);

	void add_frame();

	void display_frames();

	void update_pose_icons();

	void batch_solve_iteration();

	void batch_solve(size_t num_iterations);

	std::vector<float> get_beta() { 
		if (use_online_betas)
			return beta_common;
		else return beta;
	};
};
