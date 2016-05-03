string id                                    #plane id (desk, floor, etc)
sensor_msgs/PointCloud2 point_cloud          #Point cloud (probably, colored)
geometry_msgs/Vector3 centroid               
geometry_msgs/Vector3 normal                 #normal vector to the plane (should be normalized)
geometry_msgs/Vector3[] bounding_polygon     #N vectors 
