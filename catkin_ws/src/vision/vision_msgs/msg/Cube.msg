string color #color of the cube
geometry_msgs/Point cube_centroid		#Cube Centroid (x,y,z)
bool detected_cube    #flag to indicate if the cube was detected
geometry_msgs/Point minPoint	#Cube's min point
geometry_msgs/Point maxPoint	#Cube's max point
geometry_msgs/Point colorRGB	#Cube's color RGB 
float32 roll	#Roll for grasping
float32 pitch	#pitch for grasping
float32 yaw	#yaw for grasping
int8 type_object	#indicate the kind of object: 0=silverware, 1=tableware
float32 priority	#indicate a priority to grasp the object depending on the kind of object and the distance to the robot
