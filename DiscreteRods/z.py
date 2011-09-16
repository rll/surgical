import os, sys

control_in_filename = [sys.argv[1]]
set = int(sys.argv[2])

print control_in_filename, set

start_ind = 0
end_ind = -1
single_horizon = [5]

if set == 1:
  noise_thresh = [0.0, 0.2]
elif set == 2:
  noise_thresh = [0.4, 0.6]
else
  noise_thresh = [0.8, 1.0]

for control_in in control_in_filename:
  for horizon in single_horizon:
    for noise in noise_thresh:
      for i in range(0, 10):
        run_command = "%s %s %s %s %d %d %d %f" % ("./runExperiment", control_in + "_" + str(start_ind) + "_" + str(end_ind) + "_" + str(horizon) + "_" + str(noise) + "_exp" + str(i), control_in, control_in + "_world", start_ind, end_ind, horizon, noise)
        print run_command
        os.system(run_command)
