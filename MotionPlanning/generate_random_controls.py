""" 
Generates and writes random controls to file "reversibility/controls.dat"
usage: python generate_random_controls.py <num_controls> <max_norm translation> <max rotation>
max rotation is factor of pi
"""

import sys, os, math
from random import random as rand
import numpy
from numpy.linalg import norm

def sample_in_sphere(max_norm):
  x = max_norm * rand()
  y = max_norm * rand()
  z = max_norm * rand() 
  v = numpy.array([x,y,z])
  while(norm(v,2) > max_norm): 
    x = max_norm * rand()
    y = max_norm * rand()
    z = max_norm * rand() 
    v = numpy.array([x,y,z])
  return (x,y,z)

def sample_in_box(max_val):
  x = max_val * rand()
  y = max_val * rand()
  z = max_val * rand() 
  return (x,y,z)

def main():
  num_controls = int(sys.argv[1])
  max_norm_trans = float(sys.argv[2])
  max_rot = float(sys.argv[3]) * math.pi 

  "filename = 'reversibility/controls_%d_%f_%f.dat' % (num_controls, max_norm_trans, max_rot)"
  filename = 'reversibility/controls.dat'
  f = open(filename, 'w')
 
  for i in range(num_controls):
    x_start, y_start, z_start = sample_in_sphere(max_norm_trans)
    r1_start, r2_start, r3_start = sample_in_box(max_rot)
    x_end, y_end, z_end = sample_in_sphere(max_norm_trans)
    r1_end, r2_end, r3_end = sample_in_box(max_rot)
    control = '%f %f %f %f %f %f %f %f %f %f %f %f\n' % (x_start, y_start, z_start, r1_start, r2_start, r3_start, x_end, y_end, z_end, r1_end, r2_end, r3_end)
    print control
    f.write(control)

  f.close()



if __name__ == "__main__":
  main()
