std_msgs/Header header
sensor_msgs/Image   imgOutput
geometry_msgs/Vector3 size				# Size in meters: size in x, y (z is always 1)
geometry_msgs/Vector3[] eigen_vectors   # eigen vectors of the object from 3D informaction
geometry_msgs/Vector3 eigen_values	    # eigen values asociated with eigen_vector
geometry_msgs/Point  nearest_point      # The nearespoint of the object
geometry_msgs/Point  center_point       # The nearespoint of the object

