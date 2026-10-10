std_msgs/Header header
# Eleven modules, six contact slots each, in SDK order. Zero-force slots are included.
string[] model_name
# Signed contact coordinates on the module, one byte from the device.
int32[] x
int32[] y
int32[] z
# Scalar contact force. Unit is 0.1 N.
uint32[] force
