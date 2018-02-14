#pragma once
#include <vector>
#include <iostream>
#include <fstream>
#include <windows.h>
#include "OutlineFinder.h"
#include "GeometryHelpers.h"
#include "Model.h"

#include "util/opencv_wrapper.h"

enum Wise {
	clockwise,
	counterclockwise
};

struct OutlinePoint {
	glm::dvec2 value;
	bool type1;
	bool type2;
	int i1;
	int i2;
};

struct OutlineCircle {
	glm::dvec2 center;
	double radius;
	std::vector<int> points;
	size_t block;
	OutlineCircle() {
		points = std::vector<int>();
		radius = -1;
	}
	bool isempty() {
		if (radius >= 0) return false;
		return true;
	}
};

struct OutlineSegment {
	glm::dvec2 t1;
	glm::dvec2 t2;
	glm::ivec2 indices;
	std::vector<int> points;
	size_t block;
};

struct OutlineTraverser {
	Model * const model;

	std::vector<OutlinePoint> points;
	std::vector<OutlineSegment> segments;
	std::vector<OutlineCircle> circles;
	std::vector<Outline> outline;

	const::std::vector<int> & block_indices;

	bool debug = false;

	/*OutlineTraverser(const std::vector<glm::dvec3> & _centers, const std::vector<double> & _radii, const::std::vector<glm::ivec3> & _blocks, const::std::vector<int> & _block_indices) :
		centers(_centers), radii(_radii), blocks(_blocks), block_indices(_block_indices) {
		}*/
	OutlineTraverser(Model * _model, const::std::vector<int> & _block_indices) : model(_model), block_indices(_block_indices) {}

	void draw_outline() {
		cv::Scalar dark_grey = cv::Scalar(100, 100, 100);
		cv::Scalar orange = cv::Scalar(38, 127, 255);
		cv::Scalar crimson = cv::Scalar(109, 81, 179);
		cv::Scalar pink = cv::Scalar(143, 154, 217);
		cv::Scalar green = cv::Scalar(119, 187, 136);
		cv::Scalar light_green = cv::Scalar(196, 212, 176);

		int thickness = 2;
		int line_type = CV_AA;
		size_t image_size = 960;
		cv::Mat image = cv::Mat::zeros(image_size, image_size, CV_8UC3);
		image = cv::Scalar(255, 255, 255);

		float rotation_angle = 0;
		float lower_bound = std::numeric_limits<float>::max();
		float upper_bound = -std::numeric_limits<float>::max();
		for (size_t i = 0; i < circles.size(); i++) {
			for (size_t d = 0; d < 2; d++) {
				if (circles[i].center[d] - circles[i].radius < lower_bound)
					lower_bound = circles[i].center[d] - circles[i].radius;
				if (circles[i].center[d] + circles[i].radius > upper_bound)
					upper_bound = circles[i].center[d] + circles[i].radius;
			}
		}
		float factor = image_size / (upper_bound - lower_bound);

		auto convert_world_to_image = [lower_bound, factor, image_size](glm::vec2 world_point) {
			cv::Point image_point = cv::Point(factor * world_point[0] - factor * lower_bound, factor * world_point[1] - factor * lower_bound);
			image_point.y = image_size - image_point.y;
			return image_point;
		};

		for (size_t i = 0; i < segments.size(); i++) {
			cv::Point start = convert_world_to_image(segments[i].t1);
			cv::Point end = convert_world_to_image(segments[i].t2);
			cv::line(image, start, end, light_green, thickness, line_type);

			if (i == 51) cv::line(image, start, end, crimson, thickness, line_type);
		}

		for (size_t i = 0; i < circles.size(); i++) {
			if (circles[i].radius < 0) continue;
			cv::Point center = convert_world_to_image(circles[i].center);		
			cv::Size axes = cv::Size(factor * circles[i].radius, factor * circles[i].radius);
			cv::ellipse(image, center, axes, rotation_angle, 0, 360, light_green, thickness, line_type);

			if (i == 26) cv::ellipse(image, center, axes, rotation_angle, 0, 360, crimson, thickness, line_type);
		}

		auto reverse_angle_counting_direction = [](float start_angle, float end_angle) {
			if (end_angle < start_angle) end_angle = end_angle + 360;
			float difference = end_angle - start_angle;

			float reversed_start_angle = 360 - end_angle;
			float reversed_end_angle = 360 - start_angle;
			if (reversed_end_angle < reversed_start_angle) reversed_end_angle = reversed_end_angle + 360;
			float reversed_difference = reversed_end_angle - reversed_start_angle;
			if (difference > 180 && reversed_difference < 180) std::swap(reversed_start_angle, reversed_end_angle);
			if (difference < 180 && reversed_difference > 180) std::swap(reversed_start_angle, reversed_end_angle);
			return std::pair<float, float>(reversed_start_angle, reversed_end_angle);
		};
		for (size_t i = 0; i < min((int) outline.size(), 50); i++) {
			if (outline[i].indices[1] == RAND_MAX) {
				cv::Point center = convert_world_to_image(glm::vec2(model->centers[outline[i].indices[0]]));
				cv::Size axes = cv::Size(factor * model->radii[outline[i].indices[0]], factor * model->radii[outline[i].indices[0]]);

				glm::vec2 start_direction = outline[i].start - glm::vec2(model->centers[outline[i].indices[0]]);
				float start_angle = 180 / M_PI * atan2(start_direction[1], start_direction[0]);

				glm::vec2 end_direction = outline[i].end - glm::vec2(model->centers[outline[i].indices[0]]);
				float end_angle = 180 / M_PI * atan2(end_direction[1], end_direction[0]);

				std::pair<float, float> angles = reverse_angle_counting_direction(start_angle, end_angle);
				 
				cv::ellipse(image, center, axes, rotation_angle, angles.first, angles.second, crimson, thickness, line_type);
			}
			else {
				cv::Point start = convert_world_to_image(outline[i].start);
				cv::Point end = convert_world_to_image(outline[i].end);
				cv::line(image, start, end, crimson, thickness, line_type);
			}
		}

		for (size_t i = 0; i < points.size(); i++) {
			cv::Point center = convert_world_to_image(points[i].value);
			cv::Size axes = cv::Size(1, 1);
			cv::ellipse(image, center, axes, rotation_angle, 0, 360, pink, thickness, line_type);
		}
		cv::imshow("outline", image);
	}

	void print_points() {
		std::cout << "POINTS" << std::endl;
		for (size_t i = 0; i < points.size(); i++) {
			std::cout << "points[" << i << "]:" << std::endl;
			std::cout << "	value = " << points[i].value[0] << " " << points[i].value[1] << std::endl;
			std::cout << "	type1 = " << points[i].type1 << std::endl;
			std::cout << "	type2 = " << points[i].type2 << std::endl;
			std::cout << "	i1 = " << points[i].i1 << std::endl;
			std::cout << "	i2 = " << points[i].i2 << std::endl;
		}
		std::cout << std::endl << std::endl;
	}

	void print_point(size_t i) {
		std::cout << "POINT" << std::endl;
		std::cout << "points[" << i << "]:" << std::endl;
		std::cout << "	value = " << points[i].value[0] << " " << points[i].value[1] << std::endl;
		std::cout << "	type1 = " << points[i].type1 << std::endl;
		std::cout << "	type2 = " << points[i].type2 << std::endl;
		std::cout << "	i1 = " << points[i].i1 << std::endl;
		std::cout << "	i2 = " << points[i].i2 << std::endl;
		std::cout << std::endl << std::endl;
	}

	void print_segments() {
		std::cout << "SEGMENTS" << std::endl;
		for (size_t i = 0; i < segments.size(); i++) {
			std::cout << "segments[" << i << "]:" << std::endl;
			std::cout << "	t1 = " << segments[i].t1[0] << " " << segments[i].t1[1] << std::endl;
			std::cout << "	t2 = " << segments[i].t2[0] << " " << segments[i].t2[1] << std::endl;
			std::cout << "	indices = " << segments[i].indices[0] << " " << segments[i].indices[1] << std::endl;
			std::cout << "	points = ";
			for (size_t j = 0; j < segments[i].points.size(); j++) {
				std::cout << segments[i].points[j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl << std::endl;
	}

	void print_circles() {
		std::cout << "CIRCLES" << std::endl;
		for (size_t i = 0; i < circles.size(); i++) {
			std::cout << "circles[" << i << "]:" << std::endl;
			std::cout << "	center = " << circles[i].center[0] << " " << circles[i].center[1] << std::endl;
			std::cout << "	radius = " << circles[i].radius << std::endl;
			std::cout << "	points = ";
			for (size_t j = 0; j < circles[i].points.size(); j++) {
				std::cout << circles[i].points[j] << " ";
			}
			std::cout << std::endl;
		}
		std::cout << std::endl << std::endl;
	}

	void print_outline(size_t i) {
		//std::cout << "OUTLINE" << std::endl;
		//for (size_t i = 0; i < outline.size(); i++) {
		std::cout << "outline[" << i << "]:" << std::endl;
		if (outline[i].indices[1] < RAND_MAX) {
			std::cout << "	t1 = " << outline[i].t1[0] << " " << outline[i].t1[1] << std::endl;
			std::cout << "	t2 = " << outline[i].t2[0] << " " << outline[i].t2[1] << std::endl;
		}
		std::cout << "	indices = " << outline[i].indices[0] << " " << outline[i].indices[1] << std::endl;
		std::cout << "	start = " << outline[i].start[0] << " " << outline[i].start[1] << std::endl;
		std::cout << "	end = " << outline[i].end[0] << " " << outline[i].end[1] << std::endl;
		//}
		std::cout << std::endl << std::endl;
	}

	void store_tangents(const int i1, const int i2, const glm::dvec2 & t1, const glm::dvec2 & t2, const size_t b, size_t & p) {
		OutlinePoint point;

		point.value = t1;
		point.i1 = i1;
		point.type1 = 0;
		point.i2 = segments.size();
		point.type2 = 1;
		points.push_back(point);
		circles[i1].points.push_back(p++);
		circles[i1].center = glm::dvec2(model->centers[i1][0], model->centers[i1][1]);
		circles[i1].radius = model->radii[i1];
		circles[i1].block = b;

		point.value = t2;
		point.i1 = i2;
		point.type1 = 0;
		point.i2 = segments.size();
		point.type2 = 1;
		points.push_back(point);
		circles[i2].points.push_back(p++);
		circles[i2].center = glm::dvec2(model->centers[i2][0], model->centers[i2][1]);
		circles[i2].radius = model->radii[i2];
		circles[i2].block = b;

		OutlineSegment segment;
		segment.t1 = t1;
		segment.t2 = t2;
		segment.indices = glm::dvec2(i1, i2);
		segment.points.push_back(p - 2);
		segment.points.push_back(p - 1);
		segment.block = b;
		segments.push_back(segment);
	}

	void find_outline_intersections() {
		debug = false;
		circles = std::vector<OutlineCircle>(model->centers.size(), OutlineCircle());
		glm::dvec2 lt1, lt2, rt1, rt2;
		double d1, d2;
		
		// circle - segment tangency
		size_t count = 0;
		for (size_t b = 0; b <block_indices.size(); b++) {
			glm::ivec3 block = model->blocks[block_indices[b]];
			if (block_size(block) == 2) {
				if (get_tangents(model->centers[block[0]], model->centers[block[1]], model->radii[block[0]], model->radii[block[1]], lt1, lt2, rt1, rt2)) {
					store_tangents(block[0], block[1], lt1, lt2, block_indices[b], count);
					store_tangents(block[0], block[1], rt1, rt2, block_indices[b], count);
				}
			}
			else {
				if (get_tangents(model->centers[block[0]], model->centers[block[1]], model->radii[block[0]], model->radii[block[1]], lt1, lt2, rt1, rt2)) {
					d1 = distance_point_segment(glm::dvec2(model->centers[block[2]][0], model->centers[block[2]][1]), lt1, lt2);
					d2 = distance_point_segment(glm::dvec2(model->centers[block[2]][0], model->centers[block[2]][1]), rt1, rt2);
					if (d1 > d2)  store_tangents(block[0], block[1], lt1, lt2, block_indices[b], count);
					else store_tangents(block[0], block[1], rt1, rt2, block_indices[b], count);
				}

				if (get_tangents(model->centers[block[0]], model->centers[block[2]], model->radii[block[0]], model->radii[block[2]], lt1, lt2, rt1, rt2)) {
					d1 = distance_point_segment(glm::dvec2(model->centers[block[1]][0], model->centers[block[1]][1]), lt1, lt2);
					d2 = distance_point_segment(glm::dvec2(model->centers[block[1]][0], model->centers[block[1]][1]), rt1, rt2);
					if (d1 > d2)  store_tangents(block[0], block[2], lt1, lt2, block_indices[b], count);
					else store_tangents(block[0], block[2], rt1, rt2, block_indices[b], count);
				}

				if (get_tangents(model->centers[block[1]], model->centers[block[2]], model->radii[block[1]], model->radii[block[2]], lt1, lt2, rt1, rt2)) {
					d1 = distance_point_segment(glm::dvec2(model->centers[block[0]][0], model->centers[block[0]][1]), lt1, lt2);
					d2 = distance_point_segment(glm::dvec2(model->centers[block[0]][0], model->centers[block[0]][1]), rt1, rt2);
					if (d1 > d2)  store_tangents(block[1], block[2], lt1, lt2, block_indices[b], count);
					else store_tangents(block[1], block[2], rt1, rt2, block_indices[b], count);
				}
			}
		}

		if (debug) {
			print_points();
			print_segments();
			print_circles();
		}

		// circle - circle intersections
		glm::dvec2 t1, t2;
		OutlinePoint point;
		for (size_t i = 0; i < circles.size(); i++) {
			if (circles[i].isempty()) continue;
			for (size_t j = i + 1; j < circles.size(); j++) {
				if (circles[j].isempty()) continue;
				if (!intersect_circle_circle(circles[i].center, circles[j].center, circles[i].radius, circles[j].radius, t1, t2)) continue;

				point.value = t1;
				point.i1 = i;
				point.type1 = 0;
				point.i2 = j;
				point.type2 = 0;
				points.push_back(point);

				circles[i].points.push_back(count);
				circles[j].points.push_back(count);
				count++;

				point.value = t2;
				point.i1 = i;
				point.type1 = 0;
				point.i2 = j;
				point.type2 = 0;
				points.push_back(point);

				circles[i].points.push_back(count);
				circles[j].points.push_back(count);
				count++;
			}
		}
		if (debug) {
			print_points();
			print_segments();
			print_circles();
		}

		// circle - segment intersecitons
		bool i1, i2;
		for (size_t i = 0; i < circles.size(); i++) {
			if (circles[i].isempty()) continue;
			for (size_t j = 0; j < segments.size(); j++) {
				if (segments[j].indices[0] == i || segments[j].indices[1] == i) continue;
				intersect_circle_segment(circles[i].center, circles[i].radius, segments[j].t1, segments[j].t2, t1, t2, i1, i2);

				/*if (i == 26 && j == 51) {
					cout << "i1 = " << i1 << ", i2 = " << i2 << endl;
					glm::dvec2 c = circles[i].center;
					double r = circles[i].radius;
					const glm::dvec2 p = segments[j].t1;
					const glm::dvec2 q = segments[j].t2;
					double k = (p[1] - q[1]) / (p[0] - q[0]);
					double b = p[1] - k * p[0];
					if (!intersect_circle_line(c, r, k, b, t1, t2)) {
						cout << "!intersect_circle_line" << endl;
						cout << "c = " << c[0] << " " << c[1] << endl;
						cout << "p = " << p[0] << " " << p[1] << endl;
						cout << "q = " << q[0] << " " << q[1] << endl;
						cout << "r = " << r << endl;
						cout << "k = " << k << endl;
						cout << "b = " << b << endl;
					}
					if (!is_point_on_segment(t1, p, q)) {
						cout << "!is_point_on_segment(t1, p, q)" << endl;
						i1 = false;
					}
					if (!is_point_on_segment(t2, p, q)) {
						cout << "!is_point_on_segment(t2, p, q)" << endl;
						i2 = false;
					}
				}*/
				if (i1) {
					point.value = t1;
					point.i1 = i;
					point.type1 = 0;
					point.i2 = j;
					point.type2 = 1;
					points.push_back(point);
					circles[i].points.push_back(count);
					segments[j].points.push_back(count);
					if (debug) print_point(count);
					count++;
				}
				if (i2) {
					point.value = t2;
					point.i1 = i;
					point.type1 = 0;
					point.i2 = j;
					point.type2 = 1;
					points.push_back(point);
					circles[i].points.push_back(count);
					segments[j].points.push_back(count);
					if (debug) print_point(count);
					count++;
				}

			}
		}
		if (debug) {
			print_points();
			print_segments();
			print_circles();
		}

		// segment-segment intersection
		glm::dvec2 t;
		for (size_t i = 0; i < segments.size(); i++) {
			for (size_t j = i + 1; j < segments.size(); j++) {
				if (!intersect_segment_segment(segments[i].t1, segments[i].t2, segments[j].t1, segments[j].t2, t)) continue;
				point.value = t;
				point.i1 = i;
				point.type1 = 1;
				point.i2 = j;
				point.type2 = 1;
				points.push_back(point);
				segments[i].points.push_back(count);
				segments[j].points.push_back(count);
				count++;
			}
		}
		if (debug) {
			print_points();
			print_segments();
			print_circles();
		}
	}

	glm::dvec2 pick_closest_direction(glm::dvec2 u, glm::dvec2 v, Wise wise) {
		double alpha = myatan2(u);
		double beta1 = myatan2(v);
		double beta2 = myatan2(-v);
		double delta1, delta2, alpha_before;
		switch (wise) {
		case clockwise:
			alpha_before = alpha;
			while (alpha < beta1) alpha = alpha + 2 * M_PI;
			delta1 = alpha - beta1;
			alpha = alpha_before;
			while (alpha < beta2) alpha = alpha + 2 * M_PI;
			delta2 = alpha - beta2;
			if (delta2 < delta1) v = -v;
			break;
		case counterclockwise:
			while (beta1 < alpha) beta1 = beta1 + 2 * M_PI;
			while (beta2 < alpha) beta2 = beta2 + 2 * M_PI;
			delta1 = beta1 - alpha;
			delta2 = beta2 - alpha;
			if (delta2 < delta1) v = -v;
		}
		return v;
	}

	void find_closest_on_circle(size_t i, size_t k, size_t & k_next, bool & type_next, size_t & i_next, glm::dvec2 & v_next) {	

		// find next point
		double min_delta = std::numeric_limits<double>::max();
		glm::dvec2 v = points[k].value - circles[i].center;
		double alpha = myatan2(v);
		glm::dvec2 u, u_next; double beta, delta;

		/*std::cout << std::endl << "next on circle" << std::endl;
		std::cout << "i = " << i << std::endl;
		std::cout << "k = " << k << std::endl;
		std::cout << "alpha = " << alpha << std::endl;
		std::cout << "v = " << v[0] << ", " << v[1] << std::endl;
		std::cout << "point = " << points[k].value[0] << ", " << points[k].value[1] << std::endl;*/

		for (size_t j = 0; j < circles[i].points.size(); j++) {
			glm::dvec2 u = points[circles[i].points[j]].value - circles[i].center;
			beta = myatan2(u);
			if (abs(alpha - beta) < 1e-7) continue;
			if (beta < alpha) beta = beta + 2 * M_PI;
			delta = beta - alpha;
			//std::cout << "	j = " << j << ", alpha = " << alpha << ", beta = " << beta << ", delta = " << delta << std::endl;
			if (abs(delta) < 1e-7) continue;
			if (delta < min_delta) {
				min_delta = delta;
				k_next = circles[i].points[j];
				u_next = u;
			}
		}
		// find the type of the next primitive
		if (points[k_next].i1 == i && points[k_next].type1 == 0) {
			type_next = points[k_next].type2;
			i_next = points[k_next].i2;
		}
		else {
			type_next = points[k_next].type1;
			i_next = points[k_next].i1;
		}
		// find direction
		if (type_next == 0) v_next = u_next;
		if (type_next == 1) {
			// if it is a tangency point
			if (segments[i_next].indices[0] == i || segments[i_next].indices[1] == i) {
				glm::dvec2 p1 = points[k_next].value; glm::dvec2 p2;
				if (length(segments[i_next].t1 - p1) < std::numeric_limits<double>::min())
					p2 = segments[i_next].t2;
				else
					p2 = segments[i_next].t1;
				v_next = p2 - p1;

			}
			// if it is an intersection point
			else {
				beta = myatan2(u_next);
				beta = beta + 1e-5;
				glm::dvec2 w = glm::dvec2(cos(beta), sin(beta));
				glm::dvec2 tangent = w - u_next / length(u_next);
				glm::dvec2 v = segments[i_next].t2 - segments[i_next].t1;
				v_next = pick_closest_direction(tangent, v, clockwise);
			}
		}
	}

	void find_closest_on_segment(size_t i, size_t k, glm::dvec2 v, size_t & k_next, bool & type_next, size_t & i_next, glm::dvec2 & v_next) {
		// find next point
		double min_delta = std::numeric_limits<double>::max();
		glm::dvec2 w, u; double delta;
		for (size_t j = 0; j < segments[i].points.size(); j++) {
			w = points[segments[i].points[j]].value - points[k].value;
			delta = length(w);
			if (dot(w, v) < 0) continue;
			if (abs(delta) < 1e-4) continue;
			if (delta < min_delta) {
				min_delta = delta;
				k_next = segments[i].points[j];
				u = w;
			}
		}
		// find the type of the next primitive
		if (points[k_next].i1 == i && points[k_next].type1 == 1) {
			type_next = points[k_next].type2;
			i_next = points[k_next].i2;
		}
		else {
			type_next = points[k_next].type1;
			i_next = points[k_next].i1;
		}
		// find direction
		if (type_next == 0) v_next = u;
		if (type_next == 1) {
			v = segments[i_next].t2 - segments[i_next].t1;
			v_next = pick_closest_direction(u, v, clockwise);
		}
	}

	size_t find_starting_point(size_t & k, size_t & i) {
		glm::dvec2 up = glm::dvec2(0, 1);
		double max_y = -std::numeric_limits<double>::max();
		glm::dvec2 p;
		for (size_t j = 0; j < circles.size(); j++) {
			if (circles[j].isempty()) continue;
			if (circles[j].center[1] + circles[j].radius > max_y) {
				max_y = circles[j].center[1] + circles[j].radius;
				p = circles[j].center + circles[j].radius * up;
				i = j;
			}
		}

		if (max_y == -std::numeric_limits<double>::max()) {				
			float max_radius = 0;
			for (size_t j = 0; j < block_indices.size(); j++) {
				glm::ivec3 block = model->blocks[block_indices[j]];
				for (size_t k = 0; k < d; k++) {
					if (block[k] == RAND_MAX) break;
					if (model->radii[block[k]] > max_radius) {
						i = block[k];
						max_radius = model->radii[i];
						circles[i].center = glm::dvec2(model->centers[i][0], model->centers[i][1]);
						circles[i].radius = model->radii[i];
						circles[i].block = block_indices[j];
						p = circles[i].center + circles[i].radius * up;
					}
				}
			}				
		}

		double min_delta = std::numeric_limits<double>::max();
		glm::dvec2 v = p - circles[i].center;
		double alpha_before = myatan2(v);
		double alpha, beta, delta; glm::dvec2 u;
		for (size_t j = 0; j < circles[i].points.size(); j++) {
			alpha = alpha_before;
			u = points[circles[i].points[j]].value - circles[i].center;
			beta = myatan2(u);
			if (alpha < beta) alpha = alpha + 2 * M_PI;
			delta = alpha - beta;
			if (abs(delta) < std::numeric_limits<double>::min()) continue;
			if (delta < min_delta) {
				min_delta = delta;
				k = circles[i].points[j];
			}
		}
		return k;
	}

	std::vector<Outline> traverse_outline() {
		size_t k, i;
		find_starting_point(k, i);

		size_t count = 0;
		bool first = true;
		size_t k_start = k;
		bool type = 0;
		size_t k_next, i_next;
		bool type_next;
		glm::dvec2 v, v_next;
		while (first || k != k_start) {
			first = false;
			outline.push_back(Outline());

			if (points.empty()) {
				glm::dvec2 up = glm::dvec2(0, 1);
				outline[count].start = circles[i].center + circles[i].radius * up;
				outline[count].indices = glm::dvec2(i, RAND_MAX);
				outline[count].block = circles[i].block;
				outline[count].end = circles[i].center + circles[i].radius * up;
				break;
			}


			outline[count].start = points[k].value;
			// circle
			if (type == 0) {
				outline[count].indices = glm::dvec2(i, RAND_MAX);
				outline[count].block = circles[i].block;
				find_closest_on_circle(i, k, k_next, type_next, i_next, v_next);
			}
			// segment
			else {
				glm::dvec2 z = circles[segments[i].indices[0]].center - circles[segments[i].indices[1]].center;
				if (dot(z, v) > 0) {
					outline[count].indices = segments[i].indices;
					outline[count].t1 = segments[i].t1;
					outline[count].t2 = segments[i].t2;
				}
				else {
					outline[count].indices = glm::dvec2(segments[i].indices[1], segments[i].indices[0]);
					outline[count].t1 = segments[i].t2;
					outline[count].t2 = segments[i].t1;
				}
				outline[count].block = segments[i].block;
				find_closest_on_segment(i, k, v, k_next, type_next, i_next, v_next);
			}
			outline[count].end = points[k_next].value;
			count++;
			v = v_next;
			k = k_next;
			i = i_next;
			type = type_next;
			if (count > 70) {				
				//model->write_model();	
				//std::cout << "count > 100" << std::endl;
				//Sleep(10000);
				break;
			}
			//print_outline(count - 1);
		}		

		//draw_outline();

		return outline;
	}
};

OutlineFinder::OutlineFinder(Model * _model) : model(_model) { }

void OutlineFinder::print_outline(const std::vector<Outline> & outline) {
	std::cout << "OUTLINE" << std::endl;
	for (size_t i = 0; i < outline.size(); i++) {
		std::cout << "outline[" << i << "]:" << std::endl;
		if (outline[i].indices[1] < RAND_MAX) {
			std::cout << "	t1 = " << outline[i].t1[0] << " " << outline[i].t1[1] << std::endl;
			std::cout << "	t2 = " << outline[i].t2[0] << " " << outline[i].t2[1] << std::endl;
		}
		std::cout << "	indices = " << outline[i].indices[0] << " " << outline[i].indices[1] << std::endl;
		std::cout << "	start = " << outline[i].start[0] << " " << outline[i].start[1] << std::endl;
		std::cout << "	end = " << outline[i].end[0] << " " << outline[i].end[1] << std::endl;
	}
	std::cout << std::endl << std::endl;
}

void OutlineFinder::print_outline3D(const std::vector<Outline3D> & outline) {
	std::cout << "OUTLINE 3D" << std::endl;
	for (size_t i = 0; i < outline.size(); i++) {
		std::cout << "outline[" << i << "]:" << std::endl;
		std::cout << "	indices = " << outline[i].indices[0] << " " << outline[i].indices[1] << std::endl;
		std::cout << "	start = " << outline[i].start[0] << " " << outline[i].start[1] << " " << outline[i].start[2] << std::endl;
		std::cout << "	end = " << outline[i].end[0] << " " << outline[i].end[1] << " " << outline[i].end[2] << std::endl;
	}
	std::cout << std::endl << std::endl;
}

Outline OutlineFinder::crop_outline_segment(const glm::dvec3 & c1, const glm::dvec3 & c2, double r1, double r2, const glm::dvec2 & t, Outline outline) {
	glm::dvec2 lt1, lt2, rt1, rt2;
	get_tangents(c1, c2, r1, r2, lt1, lt2, rt1, rt2);
	std::vector<glm::dvec2> P, Q;
	P.push_back(outline.start);
	P.push_back(outline.end);
	Q.push_back(rt1);
	Q.push_back(lt1);
	double min_distance = std::numeric_limits<double>::max();
	Outline o;
	for (size_t p = 0; p < P.size(); p++) {
		for (size_t q = 0; q < Q.size(); q++) {
			if (length(P[p] - Q[q]) < min_distance) {
				min_distance = length(P[p] - Q[q]);
				o = outline;
				if (p == 0) o.start = t;
				if (p == 1) o.end = t;
			}
		}
	}
	outline = o;
	return outline;
}

void OutlineFinder::adjust_fingers_outline(std::vector<Outline> & outline) {
	std::vector<size_t> crop_outline_indices;
	std::vector<size_t> limit_outline_indices;
	std::vector<glm::ivec2> crop_indices_map;
	std::vector<glm::ivec2> limit_indices_map;
	for (size_t i = 0; i < outline.size(); i++) {
		for (size_t j = 0; j < model->crop_indices_thumb.size(); j++) {
			if ((outline[i].indices[0] == model->crop_indices_thumb[j][0] && outline[i].indices[1] == model->crop_indices_thumb[j][1]) ||
				(outline[i].indices[1] == model->crop_indices_thumb[j][0] && outline[i].indices[0] == model->crop_indices_thumb[j][1])) {
				crop_outline_indices.push_back(i);
				crop_indices_map.push_back(model->crop_indices_thumb[j]);
			}
		}
		for (size_t j = 0; j < model->limit_indices_thumb.size(); j++) {
			if ((outline[i].indices[0] == model->limit_indices_thumb[j][0] && outline[i].indices[1] == model->limit_indices_thumb[j][1]) ||
				(outline[i].indices[1] == model->limit_indices_thumb[j][0] && outline[i].indices[0] == model->limit_indices_thumb[j][1])) {
				limit_outline_indices.push_back(i);
				limit_indices_map.push_back(model->limit_indices_thumb[j]);
			}
		}
	}
	glm::dvec2 t;
	for (size_t i = 0; i < crop_outline_indices.size(); i++) {
		for (size_t j = 0; j < limit_indices_map.size(); j++) {
			if (!intersect_segment_segment(outline[crop_outline_indices[i]].start, outline[crop_outline_indices[i]].end,
				outline[limit_outline_indices[j]].start, outline[limit_outline_indices[j]].end, t)) continue;
			outline[crop_outline_indices[i]] = crop_outline_segment(model->centers[crop_indices_map[i][0]], model->centers[crop_indices_map[i][1]],
				model->radii[crop_indices_map[i][0]], model->radii[crop_indices_map[i][1]], t, outline[crop_outline_indices[i]]);
			outline[limit_outline_indices[j]] = crop_outline_segment(model->centers[limit_indices_map[j][0]], model->centers[limit_indices_map[j][1]],
				model->radii[limit_indices_map[j][0]], model->radii[limit_indices_map[j][1]], t, outline[limit_outline_indices[j]]);

			//print_outline(std::vector<Outline>(outline.begin() + crop_outline_indices[i], outline.begin() + crop_outline_indices[i] + 1));
			//print_outline(std::vector<Outline>(outline.begin() + limit_outline_indices[j], outline.begin() + limit_outline_indices[j] + 1));

		}
	}
	Outline o; glm::dvec2 lt1, lt2, rt1, rt2;
	std::vector<Outline> fingers_outline;
	for (size_t i = 0; i < model->crop_indices_fingers.size(); i++) {
		get_tangents(model->centers[model->crop_indices_fingers[i][0]], model->centers[model->crop_indices_fingers[i][1]],
			model->radii[model->crop_indices_fingers[i][0]], model->radii[model->crop_indices_fingers[i][1]], lt1, lt2, rt1, rt2);
		o.start = lt1; o.t1 = lt1; o.end = lt2; o.t2 = lt2;
		o.indices = model->crop_indices_fingers[i];
		o.block = model->adjuct_block_indices_fingers[i];
		fingers_outline.push_back(o);
		o.start = rt1; o.t1 = rt1; o.end = rt2; o.t2 = rt2;
		o.indices = model->crop_indices_fingers[i];
		o.block = model->adjuct_block_indices_fingers[i];
		fingers_outline.push_back(o);
	}
	//print_outline(fingers_outline);

	std::vector<Outline> palm_outline;
	for (size_t i = 0; i < model->limit_indices_fingers.size(); i++) {
		get_tangents(model->centers[model->limit_indices_fingers[i][0]], model->centers[model->limit_indices_fingers[i][1]],
			model->radii[model->limit_indices_fingers[i][0]], model->radii[model->limit_indices_fingers[i][1]], lt1, lt2, rt1, rt2);
		o.start = lt1; o.t1 = lt1; o.end = lt2; o.t2 = lt2;
		o.indices = model->limit_indices_fingers[i];
		palm_outline.push_back(o);
		o.start = rt1; o.t1 = rt1; o.end = rt2; o.t2 = rt2;
		o.indices = model->limit_indices_fingers[i];
		palm_outline.push_back(o);
	}
	//print_outline(palm_outline);

	for (size_t i = 0; i < fingers_outline.size(); i++) {
		for (size_t j = 0; j < palm_outline.size(); j++) {
			if (!intersect_segment_segment(fingers_outline[i].start, fingers_outline[i].end, palm_outline[j].start, palm_outline[j].end, t)) continue;
			o = crop_outline_segment(model->centers[fingers_outline[i].indices[0]], model->centers[fingers_outline[i].indices[1]],
				model->radii[fingers_outline[i].indices[0]], model->radii[fingers_outline[i].indices[1]], t, fingers_outline[i]);
			outline.push_back(o);
			//print_outline(std::vector<Outline>(outline.end() - 1, outline.end()));
		}
	}
	//print_outline(outline);
}

std::vector<Outline3D> OutlineFinder::find_3D_outline( const std::vector<Outline> & outline) {
	std::vector<Outline3D> outline3D;
	double z_start, z_end, alpha;
	Outline3D o;
	for (size_t i = 0; i < outline.size(); i++) {
		if (outline[i].indices[1] == RAND_MAX) {
			z_start = model->centers[outline[i].indices[0]][2];
			z_end = model->centers[outline[i].indices[0]][2];
			o.start = glm::dvec3(outline[i].start[0], outline[i].start[1], z_start);
			o.end = glm::dvec3(outline[i].end[0], outline[i].end[1], z_end);
		}
		else {
			glm::dvec3 c1 = model->centers[outline[i].indices[0]];
			glm::dvec3 c2 = model->centers[outline[i].indices[1]];
			alpha = length(outline[i].t1 - outline[i].start) / length(outline[i].t1 - outline[i].t2);
			z_start = c1[2] * (1 - alpha) + c2[2] * alpha;
			alpha = length(outline[i].t1 - outline[i].end) / length(outline[i].t1 - outline[i].t2);
			z_end = c1[2] * (1 - alpha) + c2[2] * alpha;
			o.start = glm::dvec3(outline[i].start[0], outline[i].start[1], z_start);
			o.end = glm::dvec3(outline[i].end[0], outline[i].end[1], z_end);

		}
		o.indices = outline[i].indices;
		o.block = outline[i].block;
		outline3D.push_back(o);
	}
	//print_outline3D(outline3D);
	return outline3D;
}

void OutlineFinder::draw_outline(const std::vector<Outline> & outline) {}

void OutlineFinder::find_outline(){
	
	OutlineTraverser palm_outline_traverser(model, model->palm_block_indices);
	palm_outline_traverser.find_outline_intersections();
	std::vector<Outline> palm_outline = palm_outline_traverser.traverse_outline();

	std::vector<Outline> final_outline;
	int finger_index; int palm_index;
	std::vector<std::pair<glm::dvec2, glm::dvec2>> intersections;
	for (size_t f = 0; f < model->fingers_block_indices.size(); f++) {	
		//std::cout << "f = " << f << std::endl;
		// compute finger outline		
		OutlineTraverser finger_outline_traverser(model, model->fingers_block_indices[f]);
		finger_outline_traverser.find_outline_intersections();
		std::vector<Outline> finger_outline = finger_outline_traverser.traverse_outline();		

		/// Find common outline between palm and finger
		finger_index = -1; palm_index = -1;
		Outline o;
		for (size_t i = 0; i < finger_outline.size(); i++) {
			if (finger_outline[i].indices[0] == model->fingers_base_centers[f] && finger_outline[i].indices[1] == RAND_MAX) {
				finger_index = i;
				break;
			}
		}
		for (size_t i = 0; i < palm_outline.size(); i++) {
			if (palm_outline[i].indices[0] == model->fingers_base_centers[f] && palm_outline[i].indices[1] == RAND_MAX) {
				palm_index = i;
				break;
			}
		}
		if (palm_index != -1 && finger_index != -1) {
			intersections = intersect_segment_segment_same_circle(glm::dvec2(model->centers[palm_outline[palm_index].indices[0]][0], model->centers[palm_outline[palm_index].indices[0]][1]),
				model->radii[palm_outline[palm_index].indices[0]], palm_outline[palm_index].start, palm_outline[palm_index].end,
				finger_outline[finger_index].start, finger_outline[finger_index].end);
			for (size_t k = 0; k < intersections.size(); k++) {
				o.indices = glm::ivec2(model->fingers_base_centers[f], RAND_MAX);
				o.start = intersections[k].first;
				o.end = intersections[k].second;
				o.block = finger_outline[finger_index].block;
				finger_outline.push_back(o);
			}
		}
		
		if (palm_index != -1) palm_outline.erase(palm_outline.begin() + palm_index);
		if (finger_index != -1) finger_outline.erase(finger_outline.begin() + finger_index);

		final_outline.insert(final_outline.end(), finger_outline.begin(), finger_outline.end());

	}
	final_outline.insert(final_outline.end(), palm_outline.begin(), palm_outline.end());

	OutlineTraverser wrist_outline_traverser(model, model->wrist_block_indices);
	std::vector<Outline> wrist_outline;
	if (model->fit_wrist_separately) {
		wrist_outline_traverser.find_outline_intersections();
		wrist_outline = wrist_outline_traverser.traverse_outline();
		final_outline.insert(final_outline.end(), wrist_outline.begin(), wrist_outline.end());
	}
	
	outline3D = find_3D_outline(final_outline);

}

void OutlineFinder::write_outline() {
	int num_items = 3;
	int d = 3;
	double * outline_pointer = new double[d * num_items * outline3D.size()];
	for (size_t i = 0; i < outline3D.size(); i++) {
		for (size_t j = 0; j < d; j++)
			outline_pointer[d * num_items * i + j] = outline3D[i].start[j];
		for (size_t j = 0; j < d; j++)
			outline_pointer[d * num_items * i + 3 + j] = outline3D[i].end[j];
		for (size_t j = 0; j < 2; j++)
			outline_pointer[d * num_items * i + 6 + j] = outline3D[i].indices[j];
		outline_pointer[d * num_items * i + 8] = outline3D[i].block;
	}
	std::string data_path = "...";
	std::ofstream output_file;
	output_file.open(data_path + "O.txt");
	for (size_t i = 0; i < d * num_items * outline3D.size(); i++) {
		output_file << outline_pointer[i] << " ";
	}
	output_file.close();
}

void OutlineFinder::compute_projections_outline(const std::vector<glm::vec3> & centers, const std::vector<float> & radii, const std::vector<glm::vec3> & points, const glm::vec3 & camera_ray) {

	size_t num_outline_fields = 3; int d = 3;
	double * outline = new double[d * num_outline_fields * outline3D.size()];
	for (size_t i = 0; i < outline3D.size(); i++) {
		for (size_t j = 0; j < d; j++)
			outline[d * num_outline_fields * i + j] = outline3D[i].start[j];
		for (size_t j = 0; j < d; j++)
			outline[d * num_outline_fields * i + 3 + j] = outline3D[i].end[j];
		for (size_t j = 0; j < 2; j++)
			outline[d * num_outline_fields * i + 6 + j] = outline3D[i].indices[j];
		outline[d * num_outline_fields * i + 8] = RAND_MAX;
	}
	int NUM_OUTLINES = outline3D.size();
	int NUM_OUTLINE_FIELDS = 3;
	int D = 3;
	size_t shift_indices = 6;
	size_t shift_start = 0;
	size_t shift_end = 3;

	std::vector<glm::dvec3> projections = std::vector<glm::dvec3>(points.size(), glm::dvec3());
	std::vector<glm::ivec3> indices = std::vector<glm::ivec3>(points.size(), glm::ivec3());
	double min_distance;
	glm::dvec3 t1, t2, c1, c2, q; double r1; size_t index1;
	for (size_t i = 0; i < points.size(); i++) {
		glm::dvec3 p = points[i];
		min_distance = std::numeric_limits<double>::max();
		for (size_t j = 0; j < NUM_OUTLINES; j++) {
			if (outline[D * NUM_OUTLINE_FIELDS * j + shift_indices + 1] == RAND_MAX) {
				index1 = outline[D * NUM_OUTLINE_FIELDS * j + shift_indices];
				t1 = glm::dvec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_start], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 2]);
				t2 = glm::dvec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_end], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 2]);
				//c1 = glm::dvec3(centers[D * index1], centers[D * index1 + 1], centers[D * index1 + 2]);
				//r1 = radii[index1];
				c1 = centers[index1];
				r1 = radii[index1];
				q = project_point_on_arc(p, c1, r1, camera_ray, t1, t2);
			}
			else {
				c1 = glm::dvec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_start], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_start + 2]);
				c2 = glm::dvec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_end], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_end + 2]);
				q = project_point_on_segment(p, c1, c2);
			}
			//std::cout << "(" << i << ", " << j << "): " << q[0] << " " << q[1] << " " << q[2] << std::endl;
			if (length(p - q) < min_distance) {
				min_distance = length(p - q);
				projections[i] = q;
				indices[i] = glm::ivec3(outline[D * NUM_OUTLINE_FIELDS * j + shift_indices], outline[D * NUM_OUTLINE_FIELDS * j + shift_indices + 1], outline[D * NUM_OUTLINE_FIELDS * j + shift_indices + 2]);
			}
		}
	}

	double * projections_pointer = new double[d * projections.size()];
	for (size_t i = 0; i < projections.size(); i++) {
		for (size_t j = 0; j < d; j++)
			projections_pointer[d * i + j] = projections[i][j];
	}
	//write_doubles("L", projections_pointer, d * projections.size());
}
