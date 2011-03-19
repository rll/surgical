"""
This script needs to be one level above the results folder
"""
from os import listdir
import sys

RESULTS_DIR = 'results/ec2_1_28_iso/'
FILTER_ARG = 1
OUTFILE_NAME_ARG = 2


if __name__ == "__main__":
  filter_type = sys.argv[FILTER_ARG]
  outfile_name = sys.argv[OUTFILE_NAME_ARG]
  files = listdir(RESULTS_DIR)
  files = filter(lambda x: filter_type in x, files)
  lines = [ ] 
  for file in files:
    f = open(RESULTS_DIR + file)
    for line in f:
      lines.append(line.rstrip().split(' '))
    f.close()
  print lines
  lines.sort(key=lambda i: (float(i[0]), float(i[1])))
  outLines = [ ] 
  for line in lines:
    outLines.append(' '.join(line)+ '\n')
  f_out = open(RESULTS_DIR + outfile_name, 'w')
  f_out.writelines(outLines)
  f_out.close()
