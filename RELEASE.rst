Rethink Robotics SDK 1.1.1 Release Notes
========================================

Date: 5/15/2015

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

Additions
---------

* Added torso navigators to control the UI in Demo Mode

Removals
--------

Changes
-------

* Migration from update_robot.py to the *rethink-updater* command on-robot via *SSH*
  http://sdk.rethinkrobotics.com/wiki/SSH_Update

Fixes
-----

* Reduced overall robot restart/shutdown time
* Added access to cmake, git, and wstool tools for the ruser account
* Patched on-robot ros_comm Transport Memory leak
* Upgraded on-robot OpenCV to 2.4.9 and recompiled with several plugins enabled (including gtk).
  This fixes an issue that prevented xdisplay_image.py from displaying any image on Baxter's screen when   
  run from the ruser account
* Fixed an issue that prevented on-robot ROSBridge from loading and communicating properly
* Fixed a bug that caused the JTAS to error with a path of one or two points is supplied as a trajectory
* Fixed an issue that caused the Joint Trajectory Action Server to throw an error when a path
  of one or two points were supplied
* Added use of mktemp in baxter.sh for Arch linux compatibility
* Added a calculation to increase the amount of time allowed to move arm to the initial
  pose of joint_trajectory_playback script in baxter_examples
* Fixed an issue in syncing gripper playback with joint_trajectory_playback arm execution
* Fixed a timing issue preventing joint_trajectory_playback from completing execution
* Fixed an issue where the on-robot /cameras/list service returned only one camera if another camera
  was broken (or unplugged)
* Fixed an issue that caused the on-robot IK service to be too conservative in estimating if  solutions
  were feasible which resulted in an increase in No Solutions

Known Issues
------------

