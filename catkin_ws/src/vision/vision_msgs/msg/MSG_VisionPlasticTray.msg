std_msgs/Header header

sensor_msgs/Image   imgOutput
#std_msgs/Bool wasFound
#cluttery section
geometry_msgs/Vector3 size_zone_cluttery                 # Size in meters: size in x, y and z
geometry_msgs/Vector3[] eigen_vectors_zone_cluttery      # eigen vectors of the object from 3D informaction
geometry_msgs/Vector3 eigen_values_zone_cluttery	     # eigen values asociated with eigen_vector
geometry_msgs/Point  nearest_point_zone_cluttery         # The nearespoint of the object
geometry_msgs/Point  center_point_zone_cluttery          # The nearespoint of the object


geometry_msgs/Vector3 size_zone_dish                 # Size in meters: size in x, y and z
geometry_msgs/Vector3[] eigen_vectors_zone_dish      # eigen vectors of the object from 3D informaction
geometry_msgs/Vector3 eigen_values_zone_dish	     # eigen values asociated with eigen_vector
geometry_msgs/Point  nearest_point_zone_dish         # The nearespoint of the object
geometry_msgs/Point  center_point_zone_dish          # The nearespoint of the object


geometry_msgs/Vector3 size_zone_glass                 # Size in meters: size in x, y and z
geometry_msgs/Vector3[] eigen_vectors_zone_glass      # eigen vectors of the object from 3D informaction
geometry_msgs/Vector3 eigen_values_zone_glass	     # eigen values asociated with eigen_vector
geometry_msgs/Point  nearest_point_zone_glass         # The nearespoint of the object
geometry_msgs/Point  center_point_zone_glass          # The nearespoint of the object
