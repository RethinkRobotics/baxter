import roslib
roslib.load_manifest('baxter_interface')
import rospy

from baxter_msgs.msg import (
    DigitalIOState,
    DigitalOutputCommand,
)

class DigitalIO(object):
    """
    Interface class for a simple Digital Input and/or Output on the Baxter robot.

    Input       - read input state
    Output      - turn output On/Off
                - read current output state
    """
    def __init__(self, component_id):
        """
        @param component_id - unique id of the digital component
        """
        self._id = component_id
        self._component_type = 'digital_io'
        self._is_output = None
        self._state = {}

        type_ns = '/sdk/robot/' + self._component_type
        topic_base = type_ns + '/' + self._id

        self._sub_state = rospy.Subscriber(
            topic_base + '/state',
            DigitalIOState,
            self._on_io_state)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            if len(self._state):
                break
            rate.sleep()

        # check if output-capable before creating publisher
        if self._is_output:
            self._pub_output = rospy.Publisher(
                type_ns + '/command',
                DigitalOutputCommand)

    def _on_io_state(self, msg):
        """
        Updates the internally stored state of the Digital Input/Output.
        """
        if self._is_output == None:
            self._is_output = not msg.isInputOnly
        self._state['state'] = (msg.state == DigitalIOState.PRESSED)

    def state(self):
        """
        Return the latest state of the Digital Input/Output.
        """
        return self._state['state']

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
        self._pub_output.publish(m_cmd)

