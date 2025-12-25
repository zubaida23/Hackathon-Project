# Command for robot control
std_msgs/Header header
string command_type          # Type of command (e.g., "STAND_UP", "SIT_DOWN", "MOVE_JOINT")
string target_joint          # Joint name for joint-specific commands
float64 target_position      # Target position for position commands
string[] parameters          # Additional command parameters