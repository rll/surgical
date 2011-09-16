import os, sys

control_in_filename = ["w3s", "w3"]
start_ind = 0
end_ind = -1
single_horizon = [0]
noise_thresh = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0]

for control_in in control_in_filename:
  for horizon in single_horizon:
    for noise in noise_thresh:
      run_command = "%s %s %s %s %d %d %d %f" % ("./runExperiment", control_in + "_" + str(start_ind) + "_" + str(end_ind) + "_" + str(horizon) + "_" + str(noise), control_in, control_in + "_world", start_ind, end_ind, horizon, noise)
      print run_command
      os.system(run_command)
