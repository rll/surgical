import os, sys, subprocess

PARAM_START_IND = int(sys.argv[1])
PARAM_END_IND = int(sys.argv[2])
PARAM_STEP = int(sys.argv[3])
PARAM_LINK_NUM = sys.argv[4]

raw_input("Have you done git pull + make yet? Enter to continue");
COMMAND = "python run_planner_libs.py"

for index in range(PARAM_START_IND, PARAM_END_IND+1, PARAM_STEP):
  command = "%s %d %d %s"%(COMMAND, index, index+PARAM_STEP-1, PARAM_LINK_NUM)
  print command 
  subprocess.Popen(command, shell=True)


