sdk-examples
============

SDK Examples  
source init.sh to set up the ros environment  
 - optionally provide the ros master hostname or IP address  

use `rosmake joint_pose` to make the joint_pose package with dependencies  
use `rosmake baxter` to make all baxter packages

use `rosrun tools enable_robot.py -e` to enable Baxter (see help with `-h`)

```
.
|-- init.sh                     # enviornment initialization script
+-- baxter/                     # the baxter rsdk stack
    +-- baxter_description/         # stl meshes for rendering
    +-- baxter_interface/           # python interfaces to baxter
    |   +-- src/                        # convention in a ros pkg for python code
    |       +-- baxter_interface/           # python package, importable
    +-- baxter_msgs/                # any and all proprietary messages and services
    |   +-- msg/                        # messages
    |   +-- srv/                        # services
    +-- examples/                   # any and all examples:
    |   +-- gripper_control/            # control grippers w/keyboard or joystick
    |   |   +-- src/                        # where the python code lives
    |   +-- head_control/               # screen display; head movement
    |   |   +-- images/                     # sample images
    |   +-- input_output/               # digital and analog I/O
    |   +-- joint_position/             # set of examples for joint position control
    |   +-- joint_velocity/             # example of velocity joint control
    +-- tools/                      # robot operation tools 
    |   +-- src/                        # where the python code lives
    |   |   +-- enable_robot.py             # robot enabler / state control utility
    |   |   +-- tare.py                     # simple calibration tool (~weekly)
    |   |   +-- calibrate_arm.py            # more extensive calibration tool (~monthly)
    +-- urdf/                       # robot description (URDF) file
    +-- utilities/                  # helper code and useful classes for sdk code
```

**Notes:**  
- **baxter_interface:** Only for use in other code. The python classes in here are generic and not runnable.
- **examples:** These are all of the level "this is how you can make baxter do stuff (using baxter_interface)".
- **tools:** These are executable tools intended for regular use by the user.  You will likely be using `enable_robot.py` throughout normal operation.
