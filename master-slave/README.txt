UC Berkeley Telesurgical Workstation

Current Maintainers:
    Daniel Duckworth    -   duckworthd @ berkeley
    Humphrey Hu         -   humprey.hu @ gmail
    Xiao-Yu Fu          -   spiralingcosmos @ gmail
    Andrew Wan          -   andrewwan1 @ gmail

Overview:
    The top level directory contains folders for master and slave code, the CwMtx C++ matrix library, and misc. libs
    
Slaves:
    The slave codebase was built and tested on Ubuntu 8.10.

    Most interesting code can be found in the "test" folder. Specifically:
        test_setpoints2.cpp     -   The current most-capable teleop file. Contains the main loop for running the slaves from networked masters
        tune_pid.cpp            -   Slightly tweaked version of test_setpoints2 with variable pid controller gains
        create_slave_1.h        -   Instantiates slave 1 (sensors, actuators, offsets, kinematic matrices)
        record/replay.cpp       -   Record or replay a trajectory file (from networked masters)
    
    Toplevel:
        hardware interfaces     -   quatech, miranova, sensor, actuator
        controllers             -   pid, dummy controller, controller
        shared files            -   shared, util, transform
        
        Slave   -   workspace cropping, inverse kinematics, control offsets
        Master  -   socket communication, forward kinematics (saved on intialization)
    
    Our socket files are slightly modified from the current rll revision, and will be back-committed after some more testing.
    
Masters:
    Due to driver support, the masters are built for Windows. We have been using Visual Studio 2008 Professional.
    
    All of the interesting code resides in masters/surgical_masters/ (namely main.cpp)
    
    main.cpp basically reads the masters using SensAble's QuickHaptics API, and optionally sends these readings over a socket.
    The slaves can (optionally) send a force-feedback vector over the client socket.
    