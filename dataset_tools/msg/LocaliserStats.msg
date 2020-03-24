std_msgs/Header header

dataset_tools/Innovation innovation
dataset_tools/Confidence confidence

# Row-major representation of the 3x3 covariance matrix
# The orientation parameters use a fixed-axis representation.
# In order, the parameters are:
# (x, y, yaw)
float64[9] covariance
