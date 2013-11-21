Rethink Robotics SDK 0.7.0 Release Notes
========================================

Date: 11/21/2013

Overview
------------

The Baxter SDK provides a platform for roboticists to develop custom
applications to run with the Baxter Research Robot.


See our wiki for more information:
https://github.com/RethinkRobotics/sdk-docs/wiki


Update Instructions
-------------------

https://github.com/RethinkRobotics/sdk-docs/wiki/Software-Update

Changes Wiki Page
-------------------

https://github.com/RethinkRobotics/sdk-docs/wiki/Release-Changes

Major Updates
-------------

* Now publicly available!
* Joint torque control
* Gazebo Support
* Joint position accuracy improvements (factor of 10)
* ROS Groovy on robot
* ROS Topic/Message cleanup allowing for direct robot communication (no '/sdk' proxy, decreased latency)
* Catkinization of SDK
* SDK reorganization into standalone metapackages, multiple repositories
* Ability to set speeds of joint position command execution
* Ability to specify gripper/object mass for custom grippers

Additions
---------

* Joint torque control
* Gazebo Support
* Ability to set speeds of joint position command execution
* Ability to specify gripper/object mass for custom grippers
* Introduction of baxter_core_msgs and baxter_maintenance_msgs (formerly baxter_msgs)
* Convenient catkin environment initialization script baxter.sh (formerly init.sh)
* ROS/network configuration display in tty3 of the robot
* Gripper Cuff control example
* Ability to reset cameras (re-enumerate if not listed at startup)
* Gripper Action Server and example program
* Gripper control in joint trajectory playback example
* Gripper cuff control in joint recorder
* Ability to disable arm-to-arm collision avoidance
* Ability to disable cuff zero-g interaction
* Ability to disable gravity compensation
* Launch files for action server/joystick examples
* Boot animation
* rosbags of /diagnostics in FTP Logs
* All (rosrun) examples now include -h (help) argument/description

Removals
--------

* baxter_msgs (now baxter_core_msgs, baxter_maintenance_msgs)
* rosbuild support
* joystart scripts (now replaced with roslaunch files)
* init.sh (now replaced with baxter.sh for catkin environments)

Changes
-------

* ROS Groovy on robot
* Joint position accuracy improvements (factor of 10)
* ROS Topic/Message cleanup allowing for direct robot communication (no '/sdk' proxy, decreased latency)
* SDK reorganization into standalone metapackages
* Catkin/Bloom standard package reorganization
* Limb joint commands now single message - reflected in limb interface
* Major gripper rewrite to support more advanced usage and future custom grippers
* ROS logging cleanup for console, diagnostics use.
* More explicit error messages
* Improved verbosity/standardization of help output
* DigitalIO state_changed signal/slot and read-only state property (no longer callable)

Fixes
-----

* Tuck arm improvements to handle ctrl-c and partial tucks

Known Issues
------------

