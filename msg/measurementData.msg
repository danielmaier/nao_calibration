# Header
Header header

# id (unique) of the measurement
string id

# joint state
sensor_msgs/JointState jointState

# chain description
string chain_name
string chain_root
string chain_tip

# camera frame
string camera_frame

# type of the marker (e.g. checkerboard)
string marker_type

# marker measurement data (e.g. checkerboard coordinates)
float64[] marker_data

# camera image
sensor_msgs/Image image

# marker frame
string marker_frame