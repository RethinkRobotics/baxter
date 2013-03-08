#!/usr/bin/env python
import roslib;
roslib.load_manifest('baxter_interface');
import rospy;

import baxter_interface.analog_io as AIO

def test_interface(io_component = 'torso_fan'):
    """ Ramps an Analog component from 0 to 100, then back down to 0. """
    b = AIO.AnalogIO(io_component)
    print b.state()
    # 0.0
    rate = rospy.Rate(2)
    for i in range(0,101,10):
        b.set_output(i)
        if i % 10 == 0: print i
        rate.sleep()
    print b.state()
    # 100.0
    for i in range(100,-1,-10):
        b.set_output(i)
        if i % 10 == 0: print i
        rate.sleep()
    b.set_output(0)
    #  (fans off)

if __name__ == '__main__':
    rospy.init_node('test_aio', anonymous=True)
    test_interface()

