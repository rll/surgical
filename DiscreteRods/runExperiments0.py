import os, sys

control_in_filename = ["x10s", "x10"]
start_ind = 0
end_ind = -1
single_horizon = [0]
noise_thresh = [0.8, 0.9, 1.0, 1.1, 1.2, 1.3]

for control_in in control_in_filename:
  for horizon in single_horizon:
    for noise in noise_thresh:
      for i in range(0, 1):
        run_command = "%s %s %s %s %d %d %d %f" % ("./runExperiment", control_in + "_" + str(start_ind) + "_" + str(end_ind) + "_" + str(horizon) + "_" + str(noise) + "_exp" + str(i), control_in, control_in + "_world", start_ind, end_ind, horizon, noise)
        print run_command
        os.system(run_command)
