# DOOSim - Deformable One-Dimensional Object Simulation

## Overview

Recent advances in the modeling of deformable one-dimensional objects (DOOs) such as surgical suture, rope, and hair show significant promise for improving the simulation, perception, and manipulation of such objects.

An important application of these tasks lies in the area of medical robotics, where robotic surgical assistants have the potential to greatly reduce surgeon fatigue and human error by improving the accuracy, speed, and robustness of surgical tasks such as suturing.

Towards this goal, we need a DOO simulation environment in order to reliably, accurately, and quickly simulate suture.

## Project Description

DiscreteRods contains the classes for simulating DOOs. The model is based off Discrete Elastic Rods (Bergou et al., SIGGRAPH 2008).

TODO: Motion Planning, Vision, ec2

## Installation Details

DOOSim has been tested on Ubuntu 10.10+ and Mac OS X.

__Requirements__

* [Eigen](http://eigen.tuxfamily.org)
* [OpenCV 2.1](http://opencv.willowgarage.com)
* [PvAPI](http://www.alliedvisiontec.com/us/products/software/windows/avt-pvapi-sdk.html)
* [GLUT](http://www.opengl.org/resources/libraries/glut/)
* [lshkit](http://lshkit.sourceforge.net)
* ...


On Ubuntu, try running the following commands:
 
    sudo apt-get update  
    sudo apt-get install xserver-xorg xserver-xorg-core openbox g++ gcc libeigen2-dev
        libgle3 libgle3-dev fortran-compiler libglut3 libglut3-dev subversion git-core
        build-essential libavformat-dev ffmpeg libcv2.1 libcvaux2.1 libhighgui2.1
        python-opencv opencv-doc libcv-dev libcvaux-dev libhighgui-dev libboost-dev

On OS X, the easiest way to install libraries is through MacPorts

    sudo port install eigen boost opencv

  You will also need to install gle from source from http://linas.org/gle/. The code will not compile unless you modify the source to #include malloc/malloc.h instead of malloc.h.




