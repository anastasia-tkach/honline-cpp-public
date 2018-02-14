#pragma once
#include "tracker/Detection/FindFingers.h"
#include "tracker/Detection/matlab_helpers.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/HandFinder/HandFinder.h"

class FingertipsDetector {
	//std::vector<Eigen::Matrix<Scalar, 2, 2>> covariances;
	//std::vector<Vector2> means;
	float max_fraction_of_pixels_in_finger = 0.15f;
	float finger_ratio_threshold = 1.8;

	//size_t min_column;
	//size_t max_column;
	//size_t min_row;
	//size_t max_row;

	//std::vector<Matrix_MxN> xy_finger_segments;
	//std::vector<Matrix_MxN> z_finger_segments;
	//std::vector<Vector2> xy_finger_tips;
	//std::vector<Vector2> z_finger_tips;
	//std::vector<Vector2> xy_finger_roots;
	//std::vector<Vector2> z_finger_roots;
	//Vector3 palm_center;

public:
	FindFingers * find_fingers;

	FingertipsDetector(FindFingers * find_fingers) {
		this->find_fingers = find_fingers;
	}

	void find_elipse_parameters(const Eigen::Matrix<Scalar, 2, 2> & covariance, float & a, float & b, float & phi) {
		Eigen::EigenSolver<Eigen::Matrix<Scalar, 2, 2>> eigen_solver(covariance);
		Eigen::Matrix<Scalar, 2, 1> eigenvalues = eigen_solver.eigenvalues().real();
		Eigen::Matrix<Scalar, 2, 2> eigenvectors = eigen_solver.eigenvectors().real();

		float smallest_eigenvalue, largest_eigenvalue;
		Eigen::Matrix<Scalar, 2, 1> largest_eigenvector, smallest_eigenvector;
		if (eigenvalues(0) < eigenvalues(1)) {
			smallest_eigenvalue = eigenvalues(0);
			largest_eigenvalue = eigenvalues(1);
			smallest_eigenvector = eigenvectors.col(0);
			largest_eigenvector = eigenvectors.col(1);
		}
		else {
			smallest_eigenvalue = eigenvalues(1);
			largest_eigenvalue = eigenvalues(0);
			smallest_eigenvector = eigenvectors.col(1);
			largest_eigenvector = eigenvectors.col(0);
		}

		phi = atan2(largest_eigenvector(1), largest_eigenvector(0));
		if (phi < 0) phi = phi + 2 * M_PI;
		a = sqrt(largest_eigenvalue);
		b = sqrt(smallest_eigenvalue);
	}

	void display_all_fingers(const Matrix_MxN &depth) {

		size_t scale = 3;
		size_t u, v;

		std::vector<cv::Scalar> colors;
		colors.push_back(cv::Scalar(0.4, 0.0, 0.9));
		colors.push_back(cv::Scalar(0.0, 0.5, 1.0));
		colors.push_back(cv::Scalar(0.0, 0.8, 1.0));
		colors.push_back(cv::Scalar(0.3, 0.8, 0.0));
		colors.push_back(cv::Scalar(1.0, 0.6, 0.0));
		size_t count = 0;

		cv::Mat image = cv::Mat::zeros(depth.rows(), depth.cols(), CV_32FC3);

		for (size_t i = find_fingers->min_row; i <= find_fingers->max_row; i++) {
			for (size_t j = find_fingers->min_column; j <= find_fingers->max_column; j++) {
				if (depth(i, j) > 0) image.at<cv::Vec3f>(i, j) = cv::Vec3f(1.0, 1.0, 1.0);

				if (find_fingers->downsampled) {
					u = (i - find_fingers->min_row) / 2;
					v = (j - find_fingers->min_column) / 2;
					if (u >= find_fingers->M || v >= find_fingers->N) continue;
					for (size_t p = 0; p < find_fingers->xy_finger_segments.size(); p++)
						if (find_fingers->xy_finger_segments[p](u, v) > 0) image.at<cv::Vec3f>(i, j) = 0.7 * cv::Vec3f(colors[p][0], colors[p][1], colors[p][2]);//cv::Vec3f(0.7, 0.7, 0.6);
				}

				else {
					for (size_t p = 0; p < find_fingers->xy_finger_segments.size(); p++)
						if (find_fingers->xy_finger_segments[p](i - find_fingers->min_row, j - find_fingers->min_column) > 0) image.at<cv::Vec3f>(i, j) = 0.7 * cv::Vec3f(colors[p][0], colors[p][1], colors[p][2]);//cv::Vec3f(0.7, 0.7, 0.6);
				}
			}
		}
		cv::resize(image, image, cv::Size(depth.cols() * scale, depth.rows() * scale));
		for (size_t p = 0; p < find_fingers->xy_finger_tips.size(); p++) {
			cv::Scalar color = colors[count]; 
			cv::circle(image, cv::Point(scale * find_fingers->xy_finger_tips[p](1), scale * find_fingers->xy_finger_tips[p](0)), 7.0, color, -1, 16);
			//cv::line(image, cv::Point(scale * find_fingers->xy_finger_tips[p](1), scale * find_fingers->xy_finger_tips[p](0)),
				//cv::Point(scale * find_fingers->xy_finger_roots[p](1), scale * find_fingers->xy_finger_roots[p](0)), color, 5);
			count++;
		}
		
		//cv::circle(image, cv::Point(scale * find_fingers->palm_center(1), scale * find_fingers->palm_center(0)), 10.0, cv::Scalar(0.0, 0.0, 1.0), -1, 16);

		cv::resize(image, image, cv::Size(image.cols / 2, image.rows / 2));

		/// Display covariance
		//int thickness = 2;
		//int line_type = CV_AA;
		//float a, b, phi, rotation_angle;
		//for (size_t p = 0; p < find_fingers->xy_finger_tips.size(); p++) {
			//cv::Scalar color = 0.4 * cv::Vec3f(colors[p][0], colors[p][1], colors[p][2]);

			//cv::Point center = cv::Point(2 * means[p][0] + 2 * find_fingers->min_row, 2 * means[p][1] + 2 * find_fingers->min_column);
			//find_elipse_parameters(covariances[p], a, b, phi);
			//cv::Size axes = cv::Size(3 * 2 * a, 3 * 2 * b);
			//rotation_angle = 360 - phi * 180 / M_PI;

			//cv::ellipse(image, center, axes, rotation_angle, 0, 360, color, thickness, line_type);

			//cv::circle(image, cv::Point(scale * means[p][1] + 1.5 * find_fingers->min_column, scale * means[p][0] + 1.5 * find_fingers->min_row), 4.0, color, -1, 16);
		//}



		cv::imshow("FindFingers", image);
	}

	bool verify_finger_shape(const Matrix_MxN &finger, Vector2s v) {
		
		Eigen::JacobiSVD<Eigen::Matrix<Scalar, 2, 2>> svd;
		Eigen::Matrix<Scalar, Eigen::Dynamic, 2> data;
		float mean_rows, mean_columns; Eigen::Matrix<Scalar, 2, 2> covariance;
		find_fingers->compute_finger_pixels_svd(finger, data, svd, mean_rows, mean_columns, covariance);
		VectorN s = svd.singularValues().array().pow(0.5);
		
		float ratio = s(0) / s(1);

		if (ratio < finger_ratio_threshold) return false;
		else return true;
	
	}

	bool find_xy_fingers(Matrix_MxN &depth) {

		Matrix_MxN_b mask = depth.array() > 0;

		Matrix_MxN distance = -1 * Matrix_MxN::Ones(find_fingers->M, find_fingers->N);
		VectorNs R, C; matlab::find<Matrix_MxN_b>(mask, R, C);
		if (R.cols() == 0 || R.rows() == 0 || C.rows() == 0 || C.cols() == 0) return false;

		Vector2s v = Vector2s(round(R.mean()), round(C.mean()));

		for (size_t t = 0; t < num_fingers + 1; t++) {
			/// Find geodesic extremum
			find_fingers->dijkstra_vector(depth, distance, v, numeric_limits<float>::max(), find_fingers->XY);
			distance.maxCoeff(&v(0), &v(1));
			distance(v(0), v(1)) = 0;

			/// Grow finger segment
			Matrix_MxN finger = -1 * Matrix_MxN::Ones(find_fingers->M, find_fingers->N);
			find_fingers->dijkstra_vector(depth, finger, v, find_fingers->model_finger_length, find_fingers->XY);

			/// Crop the segment at its base
			find_fingers->crop_finger(depth, finger);

			/// Discard too big or too small fingers
			VectorNs I; matlab::find<Matrix_MxN_b>(finger.array() > -1, I);
			if (I.size() < find_fingers->min_number_of_pixels_in_finger) continue;
			if (I.size() > max_fraction_of_pixels_in_finger * find_fingers->num_of_pixels_in_hand) continue;

			/// Discard too short segments
			if (!verify_finger_shape(finger, v)) continue;

			/// Discard fingers with tip not at the end
			int min_column = numeric_limits<size_t>::max();
			int max_column = 0;
			int min_row = numeric_limits<size_t>::max();
			int max_row = 0;
			for (size_t i = 0; i < finger.rows(); i++) {
				for (size_t j = 0; j < finger.cols(); j++) {
					if (finger(i, j) > 0) {
						if (i < min_row) min_row = i;
						if (i > max_row) max_row = i;
						if (j < min_column) min_column = j;
						if (j > max_column) max_column = j;
					}
				}
			}
			int rows_range = max_row - min_row;
			int columns_range = max_column - min_column;
			if (abs(v[0] - 0.5 * (max_row + min_row)) < rows_range / 4) {
				//cout << "v[0] = " << v[0] << ", mean_row = " << 0.5 * (max_row + min_row) << ", rows_range = " << rows_range << endl;
				continue;
			}
			if (abs(v[1] - 0.5 * (max_column + min_column)) < columns_range / 4) {
				//cout << "v[1] = " << v[1] << ", mean_column = " << 0.5 * (max_column + min_column) << ", columns_range = " << columns_range << endl;
				continue;
			}

			/// Remove processed fingers
			mask = mask.array() * (finger.array() == -1);
			distance = distance.array() * mask.array().cast<float>();
			depth = depth.array() * mask.array().cast<float>();

			find_fingers->xy_finger_segments.push_back(finger);
			find_fingers->xy_finger_tips.push_back(v.cast<float>());
		}
		return true;
	}	
	
	Matrix_MxN process_input() {

		size_t blur_kernel_size = 1;
		size_t erosion_kernel_size = 2;
		float threshold_value = 100;
		size_t threshold_type = 0;
		size_t const max_binary_value = 255;

		cv::Mat mask = find_fingers->handfinder->sensor_silhouette.clone();

		size_t kernel_width = 2;
		cv::Mat cv_depth = find_fingers->current_frame->depth.clone();
		cv::medianBlur(cv_depth, cv_depth, 1 + kernel_width * 2);

		Matrix_MxN depth = Matrix_MxN::Zero(mask.rows, mask.cols);

		for (size_t i = 0; i < mask.rows; i++) {
			for (size_t j = 0; j < mask.cols; j++) {
				if (mask.at<unsigned char>(i, j) == 255 && cv_depth.at<unsigned short>(i, j) < 2000)
					depth(i, j) = cv_depth.at<unsigned short>(i, j);
			}
		}
		return depth;
	}

	void find_fingers_main(bool display) {

		/// clean arrays
		if (find_fingers->xy_finger_segments.size()) find_fingers->xy_finger_segments.clear();
		if (find_fingers->z_finger_segments.size()) find_fingers->z_finger_segments.clear();
		if (find_fingers->xy_finger_tips.size()) find_fingers->xy_finger_tips.clear();
		if (find_fingers->z_finger_tips.size()) find_fingers->z_finger_tips.clear();
		if (find_fingers->xy_finger_roots.size()) find_fingers->xy_finger_roots.clear();
		if (find_fingers->z_finger_roots.size()) find_fingers->z_finger_roots.clear();

		Matrix_MxN original_depth = process_input();

		Matrix_MxN depth = find_fingers->crop_image(original_depth, find_fingers->min_row, find_fingers->max_row, find_fingers->min_column, find_fingers->max_column);

		depth = find_fingers->downsample_image(depth);
		
		find_fingers->M = depth.rows();
		find_fingers->N = depth.cols();
		find_fingers->num_of_pixels_in_hand = matlab::nnz(depth);
		Matrix_MxN xy_depth = depth;

		find_xy_fingers(xy_depth);
		
		find_fingers->restore_original_image(find_fingers->min_row, find_fingers->max_row, find_fingers->min_column, find_fingers->max_column, 
			find_fingers->xy_finger_tips, find_fingers->z_finger_tips, find_fingers->xy_finger_roots, find_fingers->z_finger_roots, find_fingers->palm_center);

		if (display) display_all_fingers(original_depth);
	}
};
