import errno

import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_msgs.msg import (
    AnalogIOState,
    AnalogOutputCommand,
)

class AnalogIO(object):
    """
    Interface class for a simple Analog Input and/or Output on the Baxter robot.

    Input       - read input state
    Output      - set new output state
                - read current output state
    """
    def __init__(self, component_id):
        """
        @param component_id - unique id of analog component
        """
        self._id = component_id
        self._component_type = 'analog_io'
        self._can_output = False

        self._state = {}

        type_ns = '/sdk/robot/' + self._component_type
        topic_base = type_ns + '/' + self._id

        self._sub_state = rospy.Subscriber(
            topic_base + '/state',
            AnalogIOState,
            self._on_io_state)

        timeout = 200
        while not rospy.is_shutdown() and timeout > 0:
            if len(self._state.keys()):
                break
            rospy.sleep(0.01)
            timeout -= 1

        # check if output-capable before creating publisher
        if self._can_output:
            self._pub_output = rospy.Publisher(
                type_ns + '/command',
                AnalogOutputCommand)

    def _on_io_state(self, msg):
        """
        Updates the internally stored state of the Analog Input/Output.
        """
        self._can_output = not msg.isInputOnly
        self._state['value'] = msg.value

    def state(self):
        """
        Return the latest value of the Analog Input/Output.
        """
        return self._state['value']

    def set_output(self, value):
        """
        Control the state of the Analog Output.

        @param value uint16    - new state of the Output.
        """
        if not self._can_output:
            raise IOError(errno.EACCES, "Component is not an output [%s: %s]" %
                (self._component_type, self._id))
        m_cmd = AnalogOutputCommand()
        m_cmd.name = self._id
        m_cmd.value = value
        self._pub_output.publish(m_cmd)

