Rethink Robotics Research SDK 0.6.1 Release Notes
=================================================

Date: 7/12/2013

Overview
========

The Research SDK provides a platform for roboticists to develop custom 
applications to run on the Baxter hardware platform.

See our wiki for more information:

https://github.com/RethinkRobotics/sdk-docs/wiki


Installation Instructions
=========================

https://github.com/RethinkRobotics/sdk-docs/wiki/Getting-Started


Additions
=========

* Added tuck/untuck arms routine.
* Added ability to enable/disable sonar sensors.
* Added ability to flip/mirror cameras.
* Added joint trajectory playback example.
* Added generic PID controller.

Removals
========
- None

Changes
=======

- New joint trajectory action server implementation in support of MoveIt! and Arm Navigation. Action client now configurable via dynamic_reconfigure. PID controller to track commanded joint trajectory honoring positions/times commanded.
- Updated Baxter interface limb.py API to only support 'full' joint naming (i.e. the joint names specified in the URDF). This is a reflection of the upcoming removal of redundant /robot/limb/<side>/joint_states in favor of /robot/joint_states. All examples now reflect this standard usage in elimination of 'short' naming (i.e. elimination of 's1' in favor of 'left_s1').
- Updated URDF. More accurate collision geometries, inertial descriptions, joint damping.
- URDF moved into baxter_description/urdf/.
- Joint trajectory test example updated with more exaggerated motion.

Fixes
=====

- Smoke test hostname is now abiguous to ROS distributions.
- Head camera now flipped/mirrored by default resulting in 'upright' image orientation.
- Head pan tolerance now matches deadband for the joint controller. Eliminates timeout for sub-deadband commands.
- Tuned state publish rates to 100Hz in all velocity examples.

Issues
======
                                          
- Calibration and Tare scripts will not run if there is a joint error related to the head.
Joint errors can be found in the robot monitor.  
https://github.com/RethinkRobotics/sdk-docs/wiki/Tools---robot_monitor
- The ROS_HOSTNAME is always set to "<robot_hostname>.local",
which needs to be resolvable by a client machine (avahi, /etc/hosts or DNS server configuration)
