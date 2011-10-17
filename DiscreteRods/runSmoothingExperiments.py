import os, sys

#control_in_filename = ["t7", "t6", "t5", "t4", "t3", "t2", "t1"]
control_in_filename = ["s1"]
start_ind = 0
end_ind = -1
single_horizon = [0, 5]
noise_thresh = [0.01, 0.05, 0.1, 0.15, 0.2, 0.3]

for control_in in control_in_filename:
  for horizon in single_horizon:
    for noise in noise_thresh:
      run_command = "%s %s %s %s %d %d %d %f" % ("./runSmoothingExperiment", control_in + "_" + str(start_ind) + "_" + str(end_ind) + "_" + str(horizon) + "_" + str(noise), control_in, control_in + "_world", start_ind, end_ind, horizon, noise)
      print run_command
      os.system(run_command)
      
      
      ./runSmoothingExperiment s1_short_world s1_short s1_world 0 -1 25

