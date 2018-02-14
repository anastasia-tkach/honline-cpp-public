#pragma once
#include "tracker/ForwardDeclarations.h"
#include "tracker/Types.h"

class Experimenter {

	enum ESTIMATION_TYPE {
		BATCH_OFFLINE = 2,		
		KALMAN_STANDARD = 1,
		KALMAN_EXTENDED = 3,
		INDEPENDENT = 0,
		BATCH_ONLINE = 4,		
	};
	ESTIMATION_TYPE last = BATCH_OFFLINE;

	enum ESTIMATION_PHASE {
		CALIBRATION = 0,
		EVALUATION = 1,
	};

	ESTIMATION_TYPE estimation_type;
	ESTIMATION_PHASE estimation_phase;

	Worker * worker;
	std::string sequence_path;
	std::ofstream * online_metrics_file;
	std::ofstream * marker_metrics_file;
	Mode mode;

	void iterate_batch_solver();

	void reinitialize_model();

	void run_sequence_for_evaluation();

public:

	size_t sequence_length;

	Experimenter(Worker * worker, std::string sequence_path, ofstream * online_metrics_file, ofstream * marker_metrics_file);

	void change_metrics_file_name(std::string & filename);

	void run_sequence_again_with_calibrated_model(Mode mode);

	void run_calibration_experiments(Mode mode);
};