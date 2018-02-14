#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"
#include "util/opencv_wrapper.h"
#include "tracker/Detection/TrivialDetector.h"

class HandFinder{
private:
    Camera*const camera=NULL;
    TrivialDetector*const trivial_detector=NULL;
public:
    HandFinder(Camera * camera, int downsamping_factor, bool fit_wrist_separately);
	~HandFinder() {
		delete[] sensor_indicator;
	}

/// @{ Settings
public:
    struct Settings{
        bool show_hand = false;
        bool show_wband = false;
		bool fit_wrist_separately = false;
		int downsampling_factor = 1;
        float depth_range = 150;
        float wrist_band_length = 10;
		float crop_radius = 150;
        cv::Scalar hsv_min = cv::Scalar( 94, 111,  37); ///< potentially read from file
        cv::Scalar hsv_max = cv::Scalar(120, 255, 255); ///< potentially read from file
    } _settings;
    Settings*settings=&_settings;
/// @}

public:
    bool _has_useful_data = false;
    bool _wristband_found;
    Vector3 _wband_center;
    Vector3 _wband_dir;
public:
    cv::Mat sensor_silhouette;
	cv::Mat sensor_silhouette_closed;
	cv::Mat sensor_outline;
	cv::Mat sensor_silhouette_wrist;
	cv::Mat mask_wristband;
	int * sensor_indicator;
	int num_sensor_points;

public:
    bool has_useful_data(){ return _has_useful_data; }
    bool wristband_found(){ return _wristband_found; }
    Vector3 wristband_center(){ return _wband_center; }
    Vector3 wristband_direction(){ return _wband_dir; }
    void wristband_direction_flip(){ _wband_dir=-_wband_dir; }
public:
	void binary_classification(cv::Mat & depth, cv::Mat& color);

	void binary_classification_wrist(const cv::Mat & depth);

	void get_sensor_indicator();
};
