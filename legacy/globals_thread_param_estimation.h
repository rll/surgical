#ifndef _global_thread_param_estimation_h
#define _global_thread_param_estimation_h

enum TrajectoryModes {NONE_TRAJ, RECORD, PLAYBACK};
#define traj_mode NONE_TRAJ
#define TRAJ_FILE_PATH "rrt_traj.txt"


enum ImageRecordModes {NONE_IMAGE, RECORD_IMAGE, SAVE_POINTS};
#define image_record_mode NONE_IMAGE
#define IMAGE_BASE_NAME "./model_training_ims/rrt_traj_imgs"
#define NUM_PTS_TO_SAVE 17



#endif
