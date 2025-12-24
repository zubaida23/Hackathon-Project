# Result from speech recognition system
std_msgs/Header header
string transcript            # Recognized text from speech
float64 confidence           # Confidence score (0.0-1.0)
string language              # Detected language code
float64 audio_duration       # Duration of audio in seconds
builtin_interfaces/Time timestamp  # Time of recognition