Catkinized Version of SDK Examples
============
RSDK Wiki:  http://github.com/RethinkRobotics/sdk-docs/wiki

## Start

1. Source `init.sh` to initialize the RSDK Shell environment.  
    ```bash
    $ ./init.sh <robot_hostname or ip> [ROS distro]
    ```
   - Optionally provide the ROS Master hostname or IP address, and ROS distro.  

2. (Install Dependencies)  
    ```bash
    $ sudo apt-get install ros-electric-control ros-electric-joystick-drivers ros-electric-geometry  
    $ sudo apt-get install python-argparse  
    ```  
3. (Build RSDK packages with `rosmake`)
  - Use `$ rosmake baxter --pre-clean` to make all baxter packages.  
  - Use `$ rosmake joint_position` to make the joint_position package with dependencies.  
3. Enable Baxter with the tools enable_robot.py script  
  - Use `$ rosrun tools enable_robot.py -e` to enable Baxter (see help with `-h`).  


## RSDK Directory Overview
```
.
|-- init.sh                     # environment initialization script
+-- baxter                      # the baxter rsdk meta-package
    +-- baxter_description          # .stl meshes for rendering
    +-- baxter_interface            # python interfaces to baxter
    +-- baxter_core_msgs            # ROS messages and services for robot control
    +-- baxter_maintenance_msgs     # ROS messages and services for robot maintenance
    +-- examples                    # runnable example programs:
    |   +-- gripper_control             # control grippers w/keyboard or joystick
    |   |   +-- src                         # (where python programs live)
    |   +-- head_control                # display to screen; head movement
    |   +-- input_output                # navigator buttons, digital and analog I/O
    |   +-- inverse_kinematics          # IK solver (for Cartesian endpoint poses)
    |   +-- joint_position              # joint position control
    |   +-- joint_trajectory            # joint trajectory interface example
    |   +-- joint_velocity              # joint velocity control
    +-- tools                       # robot operation tools 
    |   +-- src
    |   |   +-- enable_robot.py         # robot enabler / state control utility
    |   |   +-- tare.py                 # simple calibration tool (~weekly)
    |   |   +-- calibrate_arm.py        # more extensive calibration tool (~monthly)
    |   |   +-- update_robot.py         # robot software updater tool
    |   |   +-- smoke_test              # light operational hardware test
    +-- urdf                        # robot description (URDF) file
    +-- utilities                   # generic python helper classes for sdk code
```

**Notes:**  
- **baxter_interface:** For use in python programs. The python classes in here are generally not runnable by themselves.
- **examples:** These are all of the level "this is how you can make Baxter do stuff (using baxter_interface)".
- **tools:** These are executable tools intended for regular use by the user.  You will likely be using `enable_robot.py` throughout normal operation.

