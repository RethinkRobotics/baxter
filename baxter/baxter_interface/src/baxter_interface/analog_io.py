import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_msgs.msg import (
    AnalogIOState,
    AnalogOutputCommand,
)

USE_SDK_REPEATERS = True

class AnalogIO(object):
    """
    Interface class for a simple Analog Input and/or Output on the Baxter robot.

    Input
    - read input state
    Output
    - set new output state
    # is an input, output, both?
    """
    def __init__(self, component_id):
        """
        @param component_id - globally-unique id of analog component to interface
        """
        # member variables
        self._id = component_id
        self._component_type = 'analog_io'

        self._ns_context = '/robot' + '/' + self._component_type
        self._topic_base = self._ns_context + '/' + self._id
        
        # private member variables
        self._state = {}
        self._is_output = None

        # internal subscribers (for updating states)
        self._sub_state = rospy.Subscriber(
            ('/sdk' if USE_SDK_REPEATERS else '') + self._topic_base + '/state',
            AnalogIOState,
            self._state_callback)

        # wait for initialization to start
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if len(self._state):
                break
            rate.sleep()

        # internal publishers (for carrying out functions)
        #  Check if IO is output capable first
        if self._is_output:
            self._pub_output_cmd = rospy.Publisher(
                ('/sdk' if USE_SDK_REPEATERS else '') + self._ns_context + '/command',
                AnalogOutputCommand)

    # internal callbacks for internal subscribers for internal states
    def _state_callback(self, msg):
        """
        Updates the internally stored state of the Analog Input/Output.
        """
        if self._is_output == None:
            self._is_output = not msg.isInputOnly
        
        self._state['value'] = msg.value
        self._state['timestamp'] = msg.timestamp

    # public functions for carrying out (config) Functions (with topics/internal publisher)

    # public 'get (internal) state' methods
    def state(self):
        """
        Return the latest value of the Analog Input/Output.
        """
        return self._state['value']

    # public functions for carrying out (action) Functions for component
    def set_output(self, value):
        """
        Control the state of the Analog Output.
        
        @param value uint16    - new state of the Output.
        """
        if not self._is_output:
            rospy.logerr("Component is not an output [%s: %s]" % (
                self._component_type, self._id))
            return

        m_cmd = AnalogOutputCommand()
        m_cmd.name = self._id
        m_cmd.value = value
        self._pub_output_cmd.publish(m_cmd)

