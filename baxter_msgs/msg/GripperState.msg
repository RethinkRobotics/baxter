# The following Gripper State fields are tristate: 0 = false; 1 = true; 2 = unknown/unsupported
  uint8   STATE_FALSE = 0
  uint8   STATE_TRUE = 1
  uint8   STATE_UNKNOWN = 2
#
uint8   enabled             # true if enabled
uint8   calibrated          # true if calibration has completed
uint8   ready               # true if ready to grip
uint8   moving              # true if moving
uint8   gripping            # true if gripping
uint8   missed              # true if GRIP/GOTO/SET was commanded and the gripper reaches the end of travel
uint8   error               # true if the gripper is in an error state
#
uint8   command             # current gripper command, one of the following:
  # Note that while these have the same as values as the gripper opcodes they
  # are not used that way.
  uint8   CMD_IDLE            = 0 
  uint8   CMD_GET_ID          = 1
  uint8   CMD_RESET_ALL       = 2
  uint8   CMD_RESET_FSM       = 3
  uint8   CMD_CALIBRATE       = 4
  uint8   CMD_PREPARE_TO_GRIP = 5
  uint8   CMD_GRIP            = 6
  uint8   CMD_RELEASE         = 7
  uint8   CMD_GOTO            = 8
  uint8   CMD_STOP            = 9
  uint8   CMD_SET             = 10
  uint8   CMD_CLEAR_CALIBRATION = 11
#
float32 position            # position as a percentage of the max position;      0=closed - 100=open
#
  float32 POSITION_CLOSED = 0.0
  float32 POSITION_OPEN   = 100.0
#
float32 force               # force as a percentage of max force;                0=none   - 100=max
#
  float32 FORCE_MIN = 0.0
  float32 FORCE_MAX = 100.0
#
