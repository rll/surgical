#ifndef _global_thread_param_estimation_h
#define _global_thread_param_estimation_h

enum TrajectoryModes {NONE_TRAJ, RECORD, PLAYBACK};
#define traj_mode RECORD
//#define TRAJ_FILE_PATH "rrt_traj.txt"
#define TRAJ_FILE_PATH "../vision/suture_sets/suturenylon_traj4/suturenylon_traj4_traj.txt"


enum ImageRecordModes {NONE_IMAGE, RECORD_IMAGE, SAVE_POINTS};
#define image_record_mode RECORD_IMAGE
//#define IMAGE_BASE_NAME "./model_training_ims/rrt_traj_imgs"
#define IMAGE_BASE_NAME "../vision/suture_sets/suturenylon_traj4/suturenylon_traj4_traj_imgs"
#define NUM_PTS_TO_SAVE 17





#define IM_DIR_1 "./suture_sets/suturenylon_traj4/suturenylon_traj4_1-" 
#define IM_DIR_2 "./suture_sets/suturenylon_traj4/suturenylon_traj4_2-" 
#define IM_DIR_3 "./suture_sets/suturenylon_traj4/suturenylon_traj4_3-" 

#define IMAGE_SAVE_BASE "./suture_sets/suturenylon_traj4/reproj_allparams/suturenylon_traj4_origParams_"
#define num_ims_before_write 100000

#define ENERGY_PARAMS_LOCATION "../config/test.params"



#endif
