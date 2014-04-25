Rethink Robotics SDK 1.0.0 Release Notes
========================================

Date: 5/1/2014

Overview
------------

The Baxter SDK provides a platform for roboticists to develop custom
applications to run with the Baxter Research Robot.


See our wiki for more information:
http://sdk.rethinkrobotics.com/wiki

Update Instructions
-------------------

http://sdk.rethinkrobotics.com/wiki/Software_Update

Changes Wiki Page
-------------------

http://sdk.rethinkrobotics.com/wiki/Release_Changes

Major Updates
-------------

* SSH access to the robot! Compile and run custom software on the robot
* New user editable wiki (http://sdk.rethinkrobotics.com/wiki)
* New 'raw' joint position control mode, command joint positions directly to JCBs
* Dynamic URDF loading of robot internal model to the parameter server
* Inverse kinematics service seeding ability
* Bootable demo mode
* Joint trajectory action server spline fitting, more accurate trajectory execution
* Full interface compatible Gazebo support
* Rosbridge
* Debian installation availability
* New joint position waypoints example program
* Advanced network configuration options
* Custom gripper state/property publication, ability to emulate Rethink grippers

Additions
---------

* SSH access to the robot
* New user editable wiki (http://sdk.rethinkrobotics.com/wiki)
* Dynamic URDF loading of robot internal model to the parameter server
* Rosbridge
* Demo mode
* 'Raw' joint position control mode, command directly to the JCBs
* Software version compatibility verification
* Debian installation
* Advanced network and ROS environment configuration options (static IP, ROS master naming type, etc.)
* Joint position waypoints example
* Ability to stop, start, reboot robot software
* URDF accelerometer, display links and joints
* URDF Gazebo plugins for cameras, display, sonar, IR range sensors, accelerometers
* Gripper assembly meshes
* Second generation pneumatic gripper meshes

Removals
--------

* baxter_maintenance_msgs LSCores and RMCores services

Changes
-------

* Inverse kinematics service seeding ability
* Joint trajectory action server spline fitting, more accurate trajectory execution
* Full interface compatible Gazebo support
* All examples and tools verify software version compatibility
* Command timeout no longer disables robot, reverts to position mode holding current
* Sourceable baxter.sh script
* 'sim' baxter.sh argument
* https rosinstall repository checkouts (no SSH keys required)
* Gripper class hardware_version to hardware_name

Fixes
-----

* Limb tcp_nodelay transport hint for joint states, endpoint state, and joint commands
* Ability to command subset of joints in torque and velocity control modes
* Head nod incorrect wait_for validation
* Update_robot timeout waiting for available updates

Known Issues
------------

