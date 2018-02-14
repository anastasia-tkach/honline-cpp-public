#include "HandFinder.h"

#include <numeric> ///< std::iota
#include <fstream> ///< ifstream
#include "util/mylogger.h"
#include "util/opencv_wrapper.h"
#include "util/qfile_helper.h"
#include "tracker/Worker.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/DataStream.h"
#include "tracker/Detection/TrivialDetector.h"
//#include "tracker/Legacy/util/Util.h"
#include "./connectedComponents.h" ///< only declared in opencv3

#include "tracker/TwSettings.h"

HandFinder::HandFinder(Camera *camera, int downsampling_factor, bool fit_wrist_separately) : camera(camera){
    //CHECK_NOTNULL(camera);

	settings->fit_wrist_separately = fit_wrist_separately;
	settings->downsampling_factor = downsampling_factor;

	sensor_indicator = new int[upper_bound_num_sensor_points];

	settings->hsv_min[0] = 94;
	settings->hsv_min[1] = 111;
	settings->hsv_min[2] = 37;
	settings->hsv_max[0] = 120;
	settings->hsv_max[1] = 255;
	settings->hsv_max[2] = 255;
    
}

Vector3 point_at_depth_pixel(const cv::Mat & depth, int x, int y, Camera* camera) {
	Integer z = depth.at<unsigned short>(y, x);
	return camera->depth_to_world(x, y, z);
}

void HandFinder::get_sensor_indicator() {

	num_sensor_points = 0; int count = 0;
	for (int row = 0; row < sensor_silhouette.rows; ++row) {
		for (int col = 0; col < sensor_silhouette.cols; ++col) {
			if (sensor_silhouette.at<uchar>(row, col) != 255) continue;
			if (count % settings->downsampling_factor == 0) {
				sensor_indicator[num_sensor_points] = row * camera->width() + col;

				if (settings->fit_wrist_separately) {
					if (sensor_silhouette_wrist.at<uchar>(row, col) == 255) {
						sensor_indicator[num_sensor_points] = -sensor_indicator[num_sensor_points];
					}
				}

				num_sensor_points++;
			}
			count++;
		}
	}
}

void HandFinder::binary_classification_wrist(const cv::Mat & depth) {

	auto project_point_on_line = [](const Vector3 & p, const Vector3 & c1, const Vector3 & c2) {
		Vector3 u = c2 - c1; Vector3 v = p - c1;
		float alpha = u.dot(v) / u.dot(u);
		return Vector3(c1 + alpha * u);
	};

	if (settings->fit_wrist_separately) {
		Scalar crop_radius_sq = 30 * 30;
		Vector3 crop_center = _wband_center;

		sensor_silhouette_wrist = cv::Mat(camera->height(), camera->width(), CV_8UC1, cv::Scalar(0));
		for (int row = 0; row < camera->height(); row++) {
			for (int col = 0; col < camera->width(); col++) {
				if (sensor_silhouette.at<unsigned char>(row, col) != 255) continue;
				Vector3 point = point_at_depth_pixel(depth, col, row, camera);
				Vector3 projection = project_point_on_line(point, _wband_center , _wband_center + _wband_dir);

				if ((_wband_center - projection).squaredNorm() < crop_radius_sq)
					sensor_silhouette_wrist.at<unsigned char>(row, col) = 255;
				else
					sensor_silhouette_wrist.at<unsigned char>(row, col) = 0;
			}
		}		
	}
}

void HandFinder::binary_classification(cv::Mat& depth, cv::Mat& color) {    
    _wristband_found = false;

    TIMED_SCOPE(timer, "Worker::binary_classification");

    /// Fetch from settings
    cv::Scalar hsv_min = settings->hsv_min;
    cv::Scalar hsv_max = settings->hsv_max;
    Scalar depth_range= _settings.depth_range;

    /// We look for wristband up to here
    Scalar depth_farplane = camera->zFar();

	/// Allocated once
    static cv::Mat color_hsv;
    static cv::Mat in_z_range;

    /// Convert to HSV
    {
        cv::cvtColor(color, color_hsv, CV_RGB2HSV);
        cv::inRange(color_hsv, hsv_min, hsv_max, /*=*/ mask_wristband);
        cv::inRange(depth, camera->zNear(), depth_farplane /*mm*/, /*=*/ in_z_range);
        cv::bitwise_and(mask_wristband, in_z_range, mask_wristband);
		//cv::imshow("mask_wristband (pre)", mask_wristband); cv::waitKey(1);
    }

    /// Robust wrist
    {
        cv::Mat labels, stats, centroids;
        int num_components = cv::connectedComponentsWithStats(mask_wristband, labels, stats, centroids, 4 /*connectivity={4,8}*/);       

        /// Generate array to sort
        std::vector< int > to_sort(num_components);
        std::iota(to_sort.begin(), to_sort.end(), 0 /*start from*/);       

        /// Sort accoding to area
        auto lambda = [stats](int i1, int i2){
            int area1 = stats.at<int>(i1,cv::CC_STAT_AREA);
            int area2 = stats.at<int>(i2,cv::CC_STAT_AREA);
            return area1>area2;
        };
        std::sort(to_sort.begin(), to_sort.end(), lambda);

        if(num_components < 2 /*not found anything beyond background*/){            		
            _has_useful_data = false;
        }
        else {            
            /// Select 2nd biggest component
            mask_wristband = (labels==to_sort[1]);
            _wristband_found = true;
        }
    }

	if (_settings.show_wband) {
		cv::imshow("show_wband", mask_wristband);
		cv::waitKey(1);
	}		

    /// Crop at wrist depth
    {
        /// Extract wristband average depth
        std::pair<float, int> avg;
        for (int row = 0; row < mask_wristband.rows; ++row) {
            for (int col = 0; col < mask_wristband.cols; ++col) {
                float depth_wrist = depth.at<ushort>(row,col);
                if(mask_wristband.at<uchar>(row,col)==255){
                     if(camera->is_valid(depth_wrist)){
                         avg.first += depth_wrist;
                         avg.second++;
                     }
                 }
            }
        }
        ushort depth_wrist = (avg.second==0) ? camera->zNear() : avg.first / avg.second; 

        /// First just extract pixels at the depth range of the wrist
        cv::inRange(depth, depth_wrist-depth_range, /*mm*/
                           depth_wrist+depth_range, /*mm*/
                           sensor_silhouette /*=*/);
    }

    _wband_center = Vector3(0,0,0);
    _wband_dir = Vector3(0,0,-1);
    /// PCA
    {
        /// Compute MEAN
        int counter = 0;
        for (int row = 0; row < mask_wristband.rows; ++row){
            for (int col = 0; col < mask_wristband.cols; ++col){
                if(mask_wristband.at<uchar>(row,col)!=255) continue;
				_wband_center += point_at_depth_pixel(depth, col, row, camera);
                counter ++;
            }
        }
        _wband_center /= counter;

		if (counter < 100) _wristband_found = false;

        std::vector<Vector3> pts; pts.push_back(_wband_center);

        /// Compute Covariance
        static std::vector<Vector3> points_pca;
        points_pca.reserve(100000);
        points_pca.clear();		
        for (int row = 0; row < sensor_silhouette.rows; ++row){
            for (int col = 0; col < sensor_silhouette.cols; ++col){
                if(sensor_silhouette.at<uchar>(row,col)!=255) continue;
				Vector3 p_pixel = point_at_depth_pixel(depth, col, row, camera);
                if((p_pixel-_wband_center).norm()<100){
                    // sensor_silhouette.at<uchar>(row,col) = 255;
                    points_pca.push_back(p_pixel);
                } else {
                    // sensor_silhouette.at<uchar>(row,col) = 0;
                }
            }
        }
        if (points_pca.size() == 0) return;
        /// Compute PCA
        Eigen::Map<Matrix_3xN> points_mat(points_pca[0].data(), 3, points_pca.size() );       
        for(int i : {0,1,2})
            points_mat.row(i).array() -= _wband_center(i);
        Matrix3 cov = points_mat * points_mat.adjoint();
        Eigen::SelfAdjointEigenSolver<Matrix3> eig(cov);
        _wband_dir = eig.eigenvectors().col(2);

        /// Allow wrist to point downward
        if(_wband_dir.y() < 0) _wband_dir = -_wband_dir;
    }

    /// Set sensor_silhouette to zero outside of hand sphere
    {
        Scalar crop_radius_sq = settings->crop_radius * settings->crop_radius;
		Vector3 crop_center = _wband_center + _wband_dir * (settings->crop_radius - settings->wrist_band_length);

        for (int row = 0; row < sensor_silhouette.rows; row++){
            for (int col = 0; col < sensor_silhouette.cols; col++){
                if(sensor_silhouette.at<unsigned char>(row,col) != 255) continue;

				Vector3 p_pixel = point_at_depth_pixel(depth, col, row, camera);
                if((p_pixel-crop_center).squaredNorm() < crop_radius_sq)
                    sensor_silhouette.at<unsigned char>(row,col) = 255;
                else
                    sensor_silhouette.at<unsigned char>(row,col) = 0;
            }
        }
    }

    if(_settings.show_hand){
        cv::imshow("show_hand", sensor_silhouette);
    } else {
        cv::destroyWindow("show_hand");
    }
}

