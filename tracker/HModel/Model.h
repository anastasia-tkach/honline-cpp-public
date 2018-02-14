#pragma once
#include <vector>
#include <iostream>
#include "cudax/cuda_glm.h"
#include "util/MathUtils.h"
#include "tracker/ForwardDeclarations.h"
#include "tracker/DataStructure/CustomJointInfo.h"
#include "OutlineFinder.h"
#include "ModelSerializer.h"
#include "ModelSemantics.h"
#include "tracker/Energy/Fitting/Settings.h"

#include "opencv2/core/core.hpp"       ///< cv::Mat
#include "opencv2/highgui/highgui.hpp" ///< cv::imShow
#include "opencv2/imgproc/imgproc.hpp"

enum BetaType {
	TOP_PHALANGE_LENGTH = 0,
	PHALANGE_LENGTH = 1,
	FINGER_BASE_X = 2,
	FINGER_BASE_Y = 3,
	FINGER_BASE_Z = 4,
	PALM_CENTER_X = 5,
	PALM_CENTER_Y = 6,
	RADIUS = 7,
	FINGER_TOP_Y = 8,
	FINGER_TOP_Z = 9
};

enum AxisType {
	ROTATION_AXIS = 0,
	TRANSLATION_AXIS = 1,
};

struct Phalange {
	string name;
	int parent_id;
	std::vector<size_t> children_ids;
	Mat4f init_local;
	Mat4f local;
	Mat4f global;
	float length;
	float radius1;
	float radius2;

	size_t center_id;
	std::vector<size_t> attachments;
	std::vector<Vec3d> offsets;

	size_t segment_id; //temporary
};

struct PoseUnit {
	string name;
	std::vector<size_t> kinematic_chain;
	void set(string name, const vector<size_t> & kinematic_chain) {
		this->name = name;
		this->kinematic_chain = kinematic_chain;
	}
};

struct ShapeUnit {
	string name;
	std::vector<size_t> shape_chain;
	void set(string name, const vector<size_t> & shape_chain) {
		this->name = name;
		this->shape_chain = shape_chain;
	}
};

struct PoseDof {
	AxisType type;
	Eigen::Vector3f axis;
	size_t phalange_id;
	float min;
	float max;
};

struct ShapeDof {
	string name;
	BetaType type;
	size_t phalange_id;
	size_t center_id;
	size_t top_center_id;
	size_t attachment_center_id;
	float min;
	float max;
	void set(string name, BetaType type, size_t phalange_id) {
		this->name = name;
		this->type = type;
		this->phalange_id = phalange_id;	
		this->center_id = -1;
		this->top_center_id = -1;
	}
	void set(string name, BetaType type, size_t phalange_id, size_t center_id) {
		this->name = name;
		this->type = type;
		this->phalange_id = phalange_id;
		this->center_id = center_id;
		this->top_center_id = -1;
	}
	void set(string name, BetaType type, size_t phalange_id, size_t center_id, size_t top_center_id) {
		this->name = name;
		this->type = type;
		this->phalange_id = phalange_id;
		this->center_id = center_id;
		this->top_center_id = top_center_id;
	}
	void set(string name, BetaType type, size_t phalange_id, size_t center_id, size_t top_center_id, size_t attachment_center_id) {
		this->name = name;
		this->type = type;
		this->phalange_id = phalange_id;
		this->center_id = center_id;
		this->top_center_id = top_center_id;
		this->attachment_center_id = attachment_center_id;
	}
};

struct Tangent {
	glm::vec3 v1; glm::vec3 v2; glm::vec3 v3; glm::vec3 n;
	glm::vec3 u1; glm::vec3 u2; glm::vec3 u3; glm::vec3 m;
	Tangent() {
		v1 = glm::vec3(0, 0, 0); v2 = glm::vec3(0, 0, 0); v3 = glm::vec3(0, 0, 0);
		u1 = glm::vec3(0, 0, 0); u2 = glm::vec3(0, 0, 0); u3 = glm::vec3(0, 0, 0);
		n = glm::vec3(0, 0, 0); m = glm::vec3(0, 0, 0);
	}
};

struct  SemanticLimit {
	std::vector<std::tuple<int, float, bool>> smaller;
	std::vector<std::tuple<int, float, bool>> bigger;
	SemanticLimit(std::vector<std::tuple<int, float, bool>> smaller, std::vector<std::tuple<int, float, bool>> bigger) {
		this->smaller = smaller;
		this->bigger = bigger;
		for (size_t i = 0; i < smaller.size(); i++) {
			if (std::get<0>(smaller[i]) == -2)
				cout << "misprint in semantic limit name" << endl;
		}
		for (size_t i = 0; i < bigger.size(); i++) {
			if (std::get<0>(bigger[i]) == -2)
				cout << "misprint in semantic limit name" << endl;
		}
	}
};

/*struct ShapeDofsBlocker {
	std::vector<std::pair<int, float>> blocking_conditions;
};*/

class Model {
public:
	int num_betas;
	int num_parameters;
	int num_shape_units;
	int num_pose_units;
	int num_centers = 39;
	int num_blocks = 31;
	int num_tangent_fields = 8;
	int num_outline_fields = 3;
	int max_num_outlines = 200;

	std::vector<glm::vec3> centers;
	std::vector<glm::vec3> centers_template;
	std::vector<float> radii;
	std::vector<float> radii_template;
	std::vector<glm::ivec3> blocks;
	std::vector<Tangent> tangent_points;

	float * host_pointer_centers;
	float * host_pointer_radii;
	int * host_pointer_blocks;
	float * host_pointer_tangent_points;
	float * host_pointer_outline;
	int * host_pointer_blockid_to_pose_unit_id_map;
	int * host_pointer_blockid_to_shape_unit_id_map;
	std::vector<KinematicChain> host_pointer_kinematic_chains;
	std::vector<ShapeChain> host_pointer_shape_chains;

	glm::vec3 camera_ray;

	std::vector<int> palm_block_indices;
	std::vector<int> wrist_block_indices;
	std::vector<std::vector<int>> fingers_block_indices;
	std::vector<size_t> fingers_base_centers;
	std::vector<glm::ivec2> crop_indices_thumb;
	std::vector<glm::ivec2> limit_indices_thumb;
	std::vector<glm::ivec2> crop_indices_fingers;
	std::vector<glm::ivec2> limit_indices_fingers;
	std::vector<int> adjuct_block_indices_fingers;

	OutlineFinder outline_finder;
	ModelSerializer serializer;
	ModelSemantics semantics;
	std::vector<ThetaInfo> theta_infos;
	std::vector<BetaInfo> beta_infos;
	int * rendered_pixels;
	float * rendered_points;
	float * rendered_normals;
	int * rendered_block_ids;
	int num_rendered_points;

	std::vector<float> theta;
	std::vector<float> beta;
	std::vector<float> beta_template;
	std::vector<float> beta_certainty;
	std::vector<float> beta_latent;
	std::vector<float> beta_saved;
	std::vector<Phalange> phalanges;
	std::vector<Phalange> phalanges_template;
	std::vector<ShapeUnit> shape_units;
	std::vector<PoseUnit> pose_units;
	std::vector<PoseDof> pose_dofs;
	std::vector<ShapeDof> shape_dofs;
	std::vector<SemanticLimit> semantic_limits;
	std::map<std::string, size_t> phalanges_name_to_id_map;
	std::map<std::string, size_t> centers_name_to_id_map;
	std::map<std::string, size_t> shape_dofs_name_to_id_map;
	std::map<size_t, size_t> jointid_to_centerid_map;
	std::map<size_t, size_t> jointid_to_phalangeid_map;

	std::vector<int> blockid_to_phalangeid_map;
	std::vector<int> blockid_to_pose_unit_id_map;
	std::vector<int> blockid_to_shape_unit_id_map;

	///// replace by something more elegant
	float palm_middle_offset_ratio;
	float palm_ring_offset_ratio;
	std::map<size_t, size_t> centerid_to_attachment_id_map;
	std::vector<float> membranes_fractional_length = { 0.45f, 0.47f, 0.53f, 0.61f };
	///// replace by something more elegant

	std::map<CalibrationType, int> calibration_type_to_num_betas_map;
	std::map<CalibrationType, std::vector<float>> calibration_type_to_beta_template_map;

	cv::Mat silhouette_texture;
	cv::Mat outline_texture;
	cv::Mat real_color;

	CalibrationType calibration_type;
	CalibrationStage calibration_stage;

	bool enable_shape_prior;
	bool enable_shape_dofs_blockers;
	bool compute_membranes_derivatives;
	bool fit_wrist_separately;
	bool show_initialization_calls;
	
	std::string data_path;
	std::string sequence_path;
	std::string sequence_name;
	std::string calibrated_model_path;
	
	Model();
	~Model();

	void init(std::string data_path, std::string sequence_path, std::string sequence_name, std::string calibrated_model_path, CalibrationType calibration_type, CalibrationStage calibration_stage,
		bool load_calibrated_model, bool enable_shape_prior, bool enable_shape_dofs_blockers, bool fit_wrist_separately, bool show_initialization_calls);

	void change_calibration_type(CalibrationType calibration_type);

	void initialize_offsets(bool rewrite_thumb = true);

	void reindex();

	void compute_outline();

	void compute_tangent_point(const glm::vec3 & camera_ray, const glm::vec3 & c1, const glm::vec3 & c2, const glm::vec3 & c3, float r1, float r2, float r3,
		glm::vec3 & v1, glm::vec3 & v2, glm::vec3 & v3, glm::vec3 & u1, glm::vec3 & u2, glm::vec3 & u3, glm::vec3 & n, glm::vec3 & m);

	void compute_tangent_points();

	void print_model();

	void render_outline();

	int compute_outline_energy_data_helper(int row, int col, Vector3 p, int block, int num_rendered_points, Vector2 n_image, bool display,
		cv::Mat & image, const cv::Mat & sensor_silhouette, Camera * camera, energy::fitting::Settings * const settings, bool is_segment_point);

	void compute_outline_energy_data(const cv::Mat & sensor_silhouette, Camera * camera, energy::fitting::Settings * const settings);

	void write_model(std::string data_path, int frame_number = -1);

	void load_model_from_file(bool load_calibrated_model, std::string path = "");

	void resize_model(float uniform_scaling_factor, float width_scaling_factor, float thickness_scaling_factor);

	void update_centers();

	Matrix_3xN jacobian_theta(const Vector3 & s, size_t id);

	Matrix_3xN jacobian_beta(const int shape_unit_id, const glm::vec3 & q, const glm::vec3 & s, glm::ivec3 & index);

	Matrix_3xN jacobian_theta_new(const int pose_unit_id, const glm::vec3 & q, const glm::vec3 & s, const glm::ivec3 & index);

	void update_parameters(const std::vector<float> & theta);

	void update_theta(const std::vector<float> & theta);

	void update_beta(const std::vector<float> & beta);

	void update_beta(const VectorN & solution);

	void update_phalanges();

	void transform_joints(const std::vector<float> & theta);

	void Model::update(Phalange & phalange);

	const std::vector<float>& Model::get_theta();

	const std::vector<float>& Model::get_beta();

	void print_theta();

	void print_beta();

	std::vector<float> Model::get_updated_parameters(const vector<float> & theta, const vector<float> &delta_theta);

	Vec3f get_palm_center();

	float get_palm_length();

	Mat3f build_rotation_matrix(Vec3f euler_angles);

	void manually_adjust_initial_transformations();

	void perturb_parameters(unsigned random_seed, float uniform_scaling_mean, float prior_standard_deviation, float parameters_standard_deviation);

	void set_initial_pose();
};