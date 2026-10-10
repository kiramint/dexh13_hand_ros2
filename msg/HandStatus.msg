std_msgs/Header header
bool connected
# left, right, or none. auto is only a parameter, never a reported type.
string hand_type
bool motor_enabled
# none, torque, speed, or position.
string control_mode
bool fault
string sdk_version
string firmware_version
bool force_control_active
