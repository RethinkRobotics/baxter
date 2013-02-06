#flag indicating enabled or not
bool isEnabled

#flag indicating completion status at shutdown
int32 complete
    int32 NOT_COMPLETE = 0
    int32 COMPLETE_W_FAILURE = 1
    int32 COMPLETE_W_SUCCESS = 2

#string indicating brain behavior requesting this RC to run
string controlUid

#in the case of a shutdown, flag indicating if it was a timeout
bool timedOut

#field indicating state machine of the internal controller
int32 state
    #Basic statues, useful for all RCs
    int32 STATE_STOPPED = 0
    int32 STATE_RUNNING = 1
    int32 STATE_ERROR = 2

#in case of a failure, error code field to describe error type
int32 error
    int32 NO_ERROR = 0
    int32 RC_FAIL = 1
    # here and below are all specific failure flags
    int32 ERROR_UNRESOLVABLE_JOINT_LIMIT = 8
    int32 ERROR_UNRESOLVABLE_WORKSPACE_LIMIT = 13
    int32 ERROR_UNRESOLVABLE_COLLISION_POSE = 32
    int32 ERROR_JOINT_S0_LIMIT = 256
    int32 ERROR_JOINT_S1_LIMIT = 512
    int32 ERROR_JOINT_E0_LIMIT = 1024
    int32 ERROR_JOINT_E1_LIMIT = 2048
    int32 ERROR_JOINT_W0_LIMIT = 4096
    int32 ERROR_JOINT_W1_LIMIT = 8192
    int32 ERROR_JOINT_W2_LIMIT = 16384

    int32 ERROR_STUCK_IN_COLLISION = 32768

# user-friendly error description
string errorMsg
