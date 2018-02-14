# Online Generative Model Personalization for Hand Tracking

- **Video**: http://lgg.epfl.ch/publications/2017/HOnline/video.mp4
- **Paper PDF**: http://lgg.epfl.ch/publications/2017/HOnline/paper.pdf
- **Dataset**: http://lgg.epfl.ch/publications/2017/HOnline/guess-who.zip

## Disclaimer
To obtain the results shown in the video proper hardware is necessary:
- **Windows 10**
- Intel Core **i7 @4GhZ**
- CUDA Graphic card (**NVIDIA GTX1080** used in our demo)
- Inter RealSense depth camera 

Other notes:
- note the software must be compiled in **64bits**
- **Wristband** color calibration (make sure the wristband is detected robustly otherwise the tracking might not perform as effectively, you can check this by enabling "show wband" in the hmodel_atb application)

## BibTex
@article{tkach2017online,
  title={Online generative model personalization for hand tracking},
  author={Tkach, Anastasia and Tagliasacchi, Andrea and Remelli, Edoardo and Pauly, Mark and Fitzgibbon, Andrew},
  journal={ACM Transactions on Graphics (TOG)},
  volume={36},
  number={6},
  pages={243},
  year={2017},
  publisher={ACM}
}
	
## Running "Teaser" Sequence

in `honline/apps/honline_atb/main.cpp`

- change `90 | worker.settings->sequence_path ` to the path to teaser dataset in you machine
- change `91 | worker.settings->data_path` to the path to `honline/data/` folder on your machine
- set `98| bool benchmark`to `true`

## Running Live

in `honline/apps/honline_atb/main.cpp`

- set `98 | bool benchmark` to false
- wear a blue wristband and make sure that the wristband is always in the view of the sensor

## Tracking

### Tracking on a fast machine
* worker.settings->calibration_type = NONE
* worker.E_fitting.settings->fit2D_outline_enable = true
* worker.settings->termination_max_iters = 8

### Tracking on a slow machine
* worker.settings->calibration_type = NONE
* worker.E_fitting.settings->fit2D_outline_enable = false
* worker.settings->termination_max_iters = 6

## Calibration

### Independent
* worker.settings->calibration_type = FULL;

### Standard Kalman Filter
* worker.settings->calibration_type = FULL;
* worker.settings->run_kalman_filter = true;
* worker.settings->kalman_filter_type = STANDARD;
* worker.settings->display_estimated_certainty = true;

### Extended Kalman Filter
* worker.settings->calibration_type = FULL;
* worker.settings->run_kalman_filter = true;
* worker.settings->kalman_filter_type = EXTENDED;
* worker.settings->display_estimated_certainty = true;

## Batch

To add every 30th frame to batch solve
* worker.settings->frames_interval_between_measurements = 30;

To add the current frame to batch solve
* press "A"

To start batch solve
* press "B"

To run more iterations of batch solve
* press "B"

To stop batch solve and resume tracking (the system sometimes crashes at this point)
* press "Shift + B"


### Batch Online
* worker.settings->calibration_type = FULL;
* worker.settings->run_batch_solver = true;
* worker.settings->use_online_betas_for_batch_solver = true;

### Batch Offline
* worker.settings->calibration_type = NONE;
* worker.settings->run_batch_solver = true;
* worker.settings->use_online_betas_for_batch_solver = false;

### Manual Uniform Scaling
* Press "1" to make the model bigger
* Press "2" to make the model smaller
* Press "3" to make the model wider
* Press "4" to make the model narrower
* Press "5" to make the model thicker
* Press "6" to make the model thinner