# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import errno

import roslib
roslib.load_manifest('utilities')
import rospy

def wait_for(test, timeout=1.0, raise_on_error=True, rate=100, timeout_msg="timeout expired", body=None):
    """
    waits until some condition evaluates to True
    @param test - zero param function to be evaluated
    @param timeout - max amount of time to wait. negative or inf means indefinitely
    @param raise_on_error - raise or just return False
    @param rate - the rate at which to check
    @param timout_msg - message to supply to the timeout exception
    @param body - optional function to execute while waiting
    """
    end_time = rospy.get_time() + timeout
    rate = rospy.Rate(rate)
    notimeout = (timeout < 0.0) or timeout == float("inf")
    while not test():
        if rospy.is_shutdown():
            if raise_on_error:
                raise OSError(errno.ESHUTDOWN, "ROS shutdown")
            return False
        elif (not notimeout) and (rospy.get_time() >= end_time):
            if raise_on_error:
                raise OSError(errno.ETIMEDOUT, timeout_msg)
            return False
        if callable(body):
            body()
        rate.sleep()
    return True

def test():
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

if __name__ == '__main__':
    test()
