import os, sys

PARAM_START_IND = int(sys.argv[1])
PARAM_END_IND = int(sys.argv[2])
PARAM_LINK_NUM = sys.argv[3]

num_links = int(PARAM_LINK_NUM)

COMMAND_BIN = "./run_planner_libs"
COMMAND_BIN1 = "./run_planner_libs_dim1"
COMMAND_BIN2 = "./run_planner_libs_dim2"

for index in range(PARAM_START_IND, PARAM_END_IND+1):
  #run_command = "%s %d %d %s" % (COMMAND_BIN, index, index, PARAM_LINK_NUM)
  run_command1 = "%s %d %d %s" % (COMMAND_BIN1, index, index, PARAM_LINK_NUM)
  #print run_command
  #os.system(run_command)
  if num_links > 15:
    print run_command1
    os.system(run_command1)
  if num_links > 25:
    run_command2 = "%s %d %d %s" % (COMMAND_BIN2, index, index, PARAM_LINK_NUM)
    os.system(run_command2)
    print run_command2
  

