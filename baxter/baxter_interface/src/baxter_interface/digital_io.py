import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_msgs.msg import (
    DigitalIOState,
    DigitalOutputCommand,
)

USE_SDK_REPEATERS = True

class DigitalIO(object):
    """
    Interface class for a simple Digital Input and/or Output on the Baxter robot.

    Input
      - read input state
    Output
      - turn output On/Off 
      - read current output state
    # is an input, output, both?
    """
    def __init__(self, component_id):
        """
        @param component_id - globally-unique id of digital component to interface
        """
        # member variables
        self._id = component_id
        self._component_type = 'digital_io'

        self._ns_context = ('/sdk/robot' if USE_SDK_REPEATERS else '/robot') + '/' + self._component_type
        self._topic_base = self._ns_context + '/' + self._id

        # private member variables
        self._state = {}
        self._is_output = None

        # internal subscribers (for updating states)
        self._sub_state = rospy.Subscriber(
            self._topic_base + '/state',
            DigitalIOState,
            self._state_callback)

        # wait for subscriber initialization to start
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if len(self._state):
                break
            rate.sleep()

        # internal publishers (for carrying out functions)
        #  Check if IO is output capable first
        if self._is_output:
            self._pub_output_cmd = rospy.Publisher(
                self._ns_context + '/command',
                DigitalOutputCommand)

    # internal callbacks for internal subscribers for internal states
    def _state_callback(self, msg):
        """
        Updates the internally stored state of the Digital Input/Output.
        """
        if self._is_output == None:
            self._is_output = not msg.isInputOnly

        self._state['state'] = (msg.state == DigitalIOState.PRESSED)

    # public 'get (internal) state' methods
    def state(self):
        """
        Return the latest state of the Digital Input/Output.
        """
        return self._state['state']

    # public functions for carrying out (action) Functions for component
    def set_output(self, value):
        """
        Control the state of the Digital Output.
        
        @param value bool    - new state {True, False} of the Output.
        """
        if not self._is_output:
            rospy.logerr("Component is not an output [%s: %s]" % (
                self._component_type, self._id))
            return
        m_cmd = DigitalOutputCommand()
        m_cmd.name = self._id
        m_cmd.value = value
        self._pub_output_cmd.publish(m_cmd)

