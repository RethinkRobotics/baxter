string name       # Gripper type name
uint8  type       # Gripper type identifier
# Heartland gripper type enumeration:
    uint8  NO_GRIPPER = 0
    uint8  SUCTION_CUP_GRIPPER = 1
    uint8  PNEUMATIC_GRIPPER = 2
    uint8  ELECTRIC_GRIPPER = 128
#
uint8  hardware_id   # Gripper hardware id
# Gripper hardware id enumeration:
    uint8  INVALID_GRIPPER_HARDWARE_ID = 0
uint8  version_major # Gripper version, 0 if none
uint8  version_minor # Gripper version, 0 if none
uint8  revision_lsb  # Gripper revision number, 0 if none
