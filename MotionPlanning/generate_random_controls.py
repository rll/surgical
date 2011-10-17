""" 
Generates and writes random controls to file "reversibility/controls.dat"
usage: python generate_random_controls.py <num_controls> <max_norm translation> <max rotation>
max rotation is factor of pi
"""

import sys, os, math
from random import random as rand
import numpy
from numpy.linalg import norm
from numpy import array, zeros 

def sample_on_sphere(num, max_norm): 
  v = zeros(num)
  for i in range(len(v)):
    v[i] = rand()
  v = v / norm(v,2)
  for i in range(len(v)):
    if rand() < 0.5: 
      v[i] *= max_norm
    else:
      v[i] *= -max_norm
  return v;

def sample_in_sphere(max_norm):
  x = (2*max_val) * rand() - max_val
  y = (2*max_val) * rand() - max_val
  z = (2*max_val) * rand() - max_val
  v = numpy.array([x,y,z])
  while(norm(v,2) > max_norm): 
    x = (2*max_val) * rand() - max_val
    y = (2*max_val) * rand() - max_val
    z = (2*max_val) * rand() - max_val
    v = numpy.array([x,y,z])
  return (x,y,z)

def sample_in_box(max_val):
  x = (2*max_val) * rand() - max_val
  y = (2*max_val) * rand() - max_val
  z = (2*max_val) * rand() - max_val
  return (x,y,z)

def main():
  num_controls = int(sys.argv[1])
  max_norm_trans = float(sys.argv[2])
  #max_rot = float(sys.argv[3]) * math.pi 
  max_rot = float(sys.argv[3])

  "filename = 'reversibility/controls_%d_%f_%f.dat' % (num_controls, max_norm_trans, max_rot)"
  filename = 'reversibility/controls.dat'
  f = open(filename, 'w')
 
  for i in range(num_controls):
    #x_start, y_start, z_start = sample_on_sphere(max_norm_trans)
    #r1_start, r2_start, r3_start = sample_on_sphere(max_rot)
    #x_end, y_end, z_end = sample_on_sphere(max_norm_trans)
    #r1_end, r2_end, r3_end = sample_on_sphere(max_rot)
    v = sample_on_sphere(12, max_norm_trans)
    #control = '%f %f %f %f %f %f %f %f %f %f %f %f\n' % (x_start, y_start, z_start, r1_start, r2_start, r3_start, x_end, y_end, z_end, r1_end, r2_end, r3_end)
    control = '%f %f %f %f %f %f %f %f %f %f %f %f\n' % (v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9], v[10], v[11])
    print control
    f.write(control)

  f.close()



if __name__ == "__main__":
  main()
