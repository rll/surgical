import os, sys

control_in_filename = [sys.argv[1]]
control_in_filename = ["w4"]
set = int(sys.argv[2])
ran = int(sys.argv[3])

print control_in_filename, set

start_ind = 0
end_ind = -1
single_horizon = [0]

if set == 0:
  noise_thresh = [0.0]
elif set == 1:
  noise_thresh = [0.2]
elif set == 2:
  noise_thresh = [0.4]
elif set == 3:
  noise_thresh = [0.6]
elif set == 4:
  noise_thresh = [0.8]
else:
  noise_thresh = [1.0]

noise_thresh = [1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4, 2.6, 2.8, 3.0]

for control_in in control_in_filename:
  for horizon in single_horizon:
    for noise in noise_thresh:
      if (ran == 0):
        for i in range(0, 10):
          run_command = "%s %s %s %s %d %d %d %f" % ("./runExperiment", control_in + "_" + str(start_ind) + "_" + str(end_ind) + "_" + str(horizon) + "_" + str(noise) + "_exp" + str(i), control_in, control_in + "_world", start_ind, end_ind, horizon, noise)
          print run_command
          os.system(run_command)
      else:
        for i in range(0, 10):
          run_command = "%s %s %s %s %d %d %d %f" % ("./runExperiment", control_in + "_" + str(start_ind) + "_" + str(end_ind) + "_" + str(horizon) + "_" + str(noise) + "_exp" + str(i), control_in, control_in + "_world", start_ind, end_ind, horizon, noise)
          print run_command
          os.system(run_command)
