#include "tracker/Types.h"
#include "tracker/ForwardDeclarations.h"
#include <vector>
#include <fstream>

class GroundTruthLoader {

	DatasetType dataset_type;
	bool fit_wrist_separatedly;
	std::string sequence_path;
	std::vector<std::vector<float>> marker_positions;
	int num_markers = 36;

	Vector3 point_at_depth_pixel(int z, int x, int y, Camera* camera);

public:

	Model * model;
	Camera * camera;

	bool enable_ground_truth_reinit = false;
	std::vector<Vector3> active_model_points; 
	std::vector<Vector3> active_data_points;

	ofstream marker_based_metrics_file;

	GroundTruthLoader(DatasetType dataset_type, bool fit_wrist_separatedly, std::string sequence_path);

	void load_ground_truth_marker_positions();

	void write_model_marker_positions(const std::vector<Vector3> & model_marker_positions);

	void write_marker_based_metrics(const std::vector<Vector3> & model_marker_positions, std::string marker_based_metrics_filename, int frame_id);

	void get_sensor_silhouette(const DataFrame & current_frame, Camera * camera, cv::Mat & sensor_silhouette, cv::Mat & sensor_silhouette_wrist);

	std::vector<Vector3> get_ground_truth_marker_positions(int frame_id);

	void get_marker_positions_tompson_finger(std::vector<std::string> centers_names, std::vector<size_t> centers_block_indices, float base_offset, bool is_thumb,
		std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices);

	void get_marker_positions_tompson(std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices);

	void get_marker_positions_sridhar(std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices);

	void get_marker_positions(std::vector<Vector3> & marker_positions, std::vector<size_t> & marker_block_indices);

	void track(LinearSystem & system, DataFrame & current_frame);

};