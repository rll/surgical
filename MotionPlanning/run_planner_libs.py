import os, sys

PARAM_START_IND = int(sys.argv[1])
PARAM_END_IND = int(sys.argv[2])
PARAM_LINK_NUM = sys.argv[3]

COMMAND = "./run_planner_libs"

for index in range(PARAM_START_IND, PARAM_END_IND+1):
  run_command = "%s %d %d %s" % (COMMAND, index, index, PARAM_LINK_NUM)
  print run_command
  os.system(run_command)

