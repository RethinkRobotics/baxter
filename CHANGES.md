Rethink Robotics Research SDK 0.6.2 Release Notes
=================================================

Date: 10/15/2013

Overview
========

The Research SDK provides a platform for roboticists to develop custom 
applications to run with the Baxter hardware platform.

The rsdk-0.6.2 Update is a patch fix primarily for enabling the NTP time
service on the robot. The software is otherwise identical to the rsdk-0.6.1
version, aside from minor underlying changes to support robot hardware
and software downgrades.

See our wiki for more information:
https://github.com/RethinkRobotics/sdk-docs/wiki


Update Instructions
=========================

https://github.com/RethinkRobotics/sdk-docs/wiki/Software-Update

Additions
=========

* Enabled NTP Server on robot for setting and synchronizing robot time.
  For more on configuring time and NTP, see the Time and NTP wiki page:
  https://github.com/RethinkRobotics/sdk-docs/wiki/Time-and-NTP
* Low-level support for robot hardware manufacturing process.

Removals
========
- None

Changes
=======
- None

Fixes
=====

- Incorrect robot time will be fixed (and fixable) via the NTP service.
- Correct time will fix the TF errors regarding expired transforms.
- Low-level support for smoother EndEffector Firmware Updates. Reboots after
  updates are no longer required for new grippers.

Issues
======

- The ROS_HOSTNAME is always set to "<robot_hostname>.local", which needs to
  be resolvable by a client machine (via avahi, /etc/hosts or DNS server
  configuration)
