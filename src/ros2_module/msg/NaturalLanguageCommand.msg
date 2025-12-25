# Interpreted natural language command
std_msgs/Header header
string raw_command           # Original command text
string interpreted_command   # Processed command
string command_type          # Type of command (e.g., "navigation", "manipulation")
Entity[] entities            # Identified entities in the command
float64 confidence           # Confidence score