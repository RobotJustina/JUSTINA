std_msgs/Header header
string id                            # name of identifying the object (milk, orange juice, etc)
geometry_msgs/Point position         # position in XYZ
bool isFound                         # is current position (is found) or last position (can be occluded, lost, etc.)
float32 confidence                   # value in [0,1] indicating the probability of a correct identification
sensor_msgs/PointCloud2 point_cloud  # Point cloud (probably, colored)