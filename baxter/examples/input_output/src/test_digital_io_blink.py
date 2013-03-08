#!/usr/bin/env python
import roslib;
roslib.load_manifest('baxter_interface');
import rospy;

import baxter_interface.digital_io as DIO

def test_interface(io_component = 'left_itb_light_outer'):
    """ Blinks a Digital Output on then off. """
    b = DIO.DigitalIO(io_component)
    print b.state()
    # False
    b.set_output(True)
    #  (default: left arm navigator's outer light turns on)
    rospy.sleep(1)
    print b.state()
    # True
    b.set_output(False)
    #  (light out)

if __name__ == '__main__':
    rospy.init_node('test_dio', anonymous=True)
    test_interface()

