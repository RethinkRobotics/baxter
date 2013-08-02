## General electric gripper command message
float32 position # position, as a percentage of max position;      0=closed - 100=open
float32 force    # force limit, as a percentage of max force;      0=none   - 100=max
float32 velocity # velocity, as a percentage of max velocity;      0=none   - 100=max
float32 holding  # holding force, as a percentage of max velocity; 0=none   - 100=max
float32 deadZone # dead zone, as a percentage of max position;     0=none   - 100=max