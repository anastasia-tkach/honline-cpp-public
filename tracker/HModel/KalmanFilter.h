#include "tracker/Types.h"
#include "util/opencv_wrapper.h"

class KalmanFilter {

public:
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> estimated_hessian;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> estimated_values;
	Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> measured_hessian;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> measured_values;
	Eigen::Matrix<Scalar, Eigen::Dynamic, 1> ground_truth_values;

	struct Settings {		
		KalmanType kalman_filter_type;
		float weight;		
		float measurement_noise_value = 1000.0f;//7000
		float system_noise_value = 0.0f;//0.1
		bool enable_shape_dofs_blockers = false;

	} _settings;
	Settings * settings = &_settings;

private:
	int num_betas;
	bool record_history = true;

	std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> measured_values_history;
	std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> estimated_values_history;
	std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> measured_certainties_history;
	std::vector<Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> estimated_certainties_history;
	float max_measured_certainty_so_far = 0;
	float max_estimated_certainty_so_far = 0;

	// for display
	int thickness = 2;
	int line_type = CV_AA;
	int font_face = cv::FONT_HERSHEY_COMPLEX;
	float font_scale = 0.7;
	int font_thickness = 1;
	int baseline = 0;
	cv::Scalar dark_grey = cv::Scalar(100, 100, 100);
	cv::Scalar light_grey = cv::Scalar(200, 200, 200);
	cv::Scalar orange = cv::Scalar(38, 127, 255);
	cv::Scalar crimson = cv::Scalar(109, 81, 179);
	cv::Scalar pink = cv::Scalar(143, 154, 217);
	cv::Scalar green = cv::Scalar(119, 187, 136);
	cv::Scalar light_green = cv::Scalar(196, 212, 176);

	void find_elipse_parameters(const Eigen::Matrix<Scalar, 2, 2> & covariance, float & a, float & b, float & phi);

	void display_covariance(std::vector<size_t> beta_ids, std::vector<std::string> beta_names, size_t i, size_t j, cv::Mat & image);

public:

	KalmanFilter(const std::vector<float> & beta, KalmanType kalman_filter_type, float kalman_filter_weight, bool enable_shape_dofs_blockers);

	std::vector<float> get_estimate();

	void set_measured_hessian(const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & measured_hessian,
		const std::vector<bool> & shape_dofs_blockers_conditions, const std::vector<std::vector<int>> & shape_dofs_blockers_beta_sets,
		const std::vector<float> & theta, const float fingers_bending_latent_variable);

	void update_estimate(const std::vector<float>  & beta, const Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> & hessian);

	void write_estimate(size_t frame_id, std::string logs_path);

	void display_covariances();

	void display_estimate_history(size_t beta_id);

	void display_several_estimate_histories(vector<size_t> beta_ids);

	void display_several_measure_histories(vector<size_t> beta_ids);

	void track(LinearSystem& system, const std::vector<Scalar>& beta);
};