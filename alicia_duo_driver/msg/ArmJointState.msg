# Standard Header
std_msgs/Header header

# Main six joint angles (radians)
float32 joint1
float32 joint2
float32 joint3
float32 joint4
float32 joint5
float32 joint6

# Button states (assuming these are simple integers)
int32 but1
int32 but2

# Gripper angle (radians)
float32 gripper

# Optional motion control time (seconds)
# float32 time # Commented out as per your definition, uncomment if needed