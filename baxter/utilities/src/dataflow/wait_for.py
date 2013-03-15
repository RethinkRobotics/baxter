import errno

import roslib
roslib.load_manifest('utilities')
import rospy

def wait_for(func_it, timeout=1.0, raise_on_error=True, rate=100):
    """
    waits until some condition evaluates to True
    @param func_it - zero param function to be evaluated
    @param timeout - max amount of time to wait
    @param raise_on_error - raise or just return False
    @param rate - the rate at which to check
    """
    end_time = rospy.get_time() + timeout
    rate = rospy.Rate(rate)
    while not func_it():
        if rospy.is_shutdown():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS shutdown")
            return False
        elif rospy.get_time() >= end_time:
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, "timeout expired")
            return False
        rate.sleep()
    return True

if __name__ == '__main__':
    import sys
    rospy.init_node("wait_for_test")
    x = [0]
    def test():
        x[0] += 1
        sys.stdout.write("%d, " % (x[0],))
        return x[0] > 20
    try:
        print("waiting up to 1.0s for x > 20")
        wait_for(test)
        print("success")
        x[0] = 0
        print("waiting up to 0.1s for x > 20")
        wait_for(test, 0.1)
        print("success")
    except OSError as e:
        print e.strerror
    print("done.")
