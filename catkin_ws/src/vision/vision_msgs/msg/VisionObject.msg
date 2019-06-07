std_msgs/Header header
string id                                    #name of identifying the object (milk, orange juice, etc) or color (green,blue, etc)
string category                              #object type (drink, snack, etc)
float32 confidence                           #value in [0,1] indicating the probability of a correct identification in the case of color objects indiates the grasping priority
sensor_msgs/Image image			     #image in rgb of object;
sensor_msgs/PointCloud2 point_cloud          #Point cloud (probably, colored)
geometry_msgs/Vector3 size                   #Size in meters: size in x, y and z
geometry_msgs/Pose pose                      #Centroid and orientation
geometry_msgs/Vector3[] bounding_box         #Two vectors indicating the bounding box
geometry_msgs/Vector3[] bounding_polygon     #N vectors 
int32 x					     #top left x
int32 y					     #top left y
int32 width				     #top widht
int32 height				     #top height
bool graspable                               #graspable by the arm or detect an object by color

geometry_msgs/Point minPoint	#Cube's min point
geometry_msgs/Point maxPoint	#Cube's max point
geometry_msgs/Point colorRGB	#Cube's color RGB 
float32 roll	#Roll for grasping cutleries
float32 pitch	#pitch for grasping cutleries
float32 yaw	#yaw for grasping cutleries
int8 type_object	#indicate the kind of object: CUTLERY = 0, BOWL= 1, DISH=2, GLASS=3