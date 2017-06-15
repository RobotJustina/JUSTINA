#include <ros/ros.h>
#include <justina_tools/JustinaVision.h>
#include <justina_tools/JustinaTools.h>

int main(int argc, char ** argv){
	
	ros::init(argc, argv, "find_all_objec_test");
	ros::NodeHandle nh;

	JustinaVision::setNodeHandle(&nh);

	ros::Rate rate(10);
	
	int state = 0;
	bool finished = false;
	while(ros::ok() && !finished){
		std::vector<vision_msgs::VisionObject> objects;
		switch(state){
			case 0:
				JustinaVision::detectAllObjects(objects);
				JustinaTools::pdfImageExport("findAllObjects", "/home/$USER/objs/");
				for (int i = 0; i < objects.size(); i++)
				{
					std::cout << "Category - Object_" << i << ":  " << objects[i].category << std::endl; 
				}
				state = 1;
				break;
			case 1:
				finished = true;
				break;
		}
		rate.sleep();
		ros::spinOnce();
	}

	return 1;

}
