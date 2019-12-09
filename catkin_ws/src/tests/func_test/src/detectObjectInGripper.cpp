#include "ros/ros.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){
	
	std::cout << "INITIALIZING A TEST JUST FOR TEST..." << std::endl;
	ros::init(argc, argv, "detect_object_in_gripper");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
	ros::Rate loop(10);
	
    float x, y, z;
    geometry_msgs::Point gripperPose;
    bool detected=false;

	int state = 0;
	bool finished = false;

	while(ros::ok() && cv::waitKey(1) != 'q'){
		

		switch(state){
			case 0:

                JustinaHRI::waitAfterSay("hello", 4000, 0);
                JustinaHRI::say("please put the red block in my gripper");
                ros::Duration(1.0).sleep();
                JustinaTasks::detectObjectInGripper("", true, 7000);

				state=1;

				break;
			case 1:
                JustinaHRI::say("now, i am going to verify if the block is in my hand");

                JustinaManip::hdGoTo(0, -0.9, 3000);
                boost::this_thread::sleep(boost::posix_time::milliseconds(400));
                if (JustinaVision::getGripperPos(gripperPose)) {
                    x = gripperPose.x;
                    y = gripperPose.y;
                    z = gripperPose.z;
                } else {
                        JustinaManip::getLeftHandPosition(x, y, z);
                        gripperPose.x=x;
                        gripperPose.y=y;
                        gripperPose.z=z;
                }

                detected = JustinaVision::detectColorObjectGripper("red", gripperPose);
                if(detected){
                    JustinaHRI::say("i can confirm the object is un my hand");
                    finished = true;
				    break;
                }
                else
                    detected = JustinaVision::detectColorObjectGripper("red", gripperPose);
                
                if(detected){
                    JustinaHRI::say("i can confirm the object is un my hand");
                    finished = true;
				    break;
                }
                else{
                    JustinaHRI::say("sorry the object is not in my hand");
                    finished = true;
				    break;
                }
		}
		loop.sleep();
		ros::spinOnce();
	}

	return 0;

}