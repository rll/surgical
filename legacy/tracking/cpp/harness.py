#! /usr/bin/python
# -*- coding: utf-8 -*-

# Rectifies a whole bunch of images at once
import os

f = open('/home/jinna/tracking/matlab/bandCorners.txt')
nums = xrange(406,492+1)
lines = f.readlines()
filepre = '/home/jinna/tracking/data/2010Jul01/rect/BandOnly/test1-'
filesuf = '.ppm'
exe = "./rectifyIm"

for i in xrange(len(lines)):
    coords = lines[i].split()
    imNum = nums[i]

    cmdargs = lines[i].rstrip() + " " + filepre + " " + str(imNum) + " " + filesuf
    cmd = exe + " " + cmdargs
    print cmd
    os.system(cmd)
    