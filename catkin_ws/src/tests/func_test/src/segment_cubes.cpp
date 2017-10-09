#include <ros/ros.h>
#include <justina_tools/JustinaVision.h>
#include <justina_tools/JustinaTools.h>
#include <justina_tools/JustinaTasks.h>

int main(int argc, char ** argv){
	
	ros::init(argc, argv, "segment_cubes_test");
	ros::NodeHandle nh;

	JustinaVision::setNodeHandle(&nh);

	ros::Rate rate(10);
	
	int state = 0;
	bool finished = false;
	while(ros::ok() && !finished){
		vision_msgs::CubesSegmented my_cubes;
		std::vector<vision_msgs::CubesSegmented> my_Stacks;

		switch(state){
			case 0:
				JustinaVision::getCubesSeg(my_cubes);
				JustinaTasks::sortCubes(my_cubes, my_Stacks);
				
				if(my_Stacks[0].recog_cubes.size()>0)
				{
					std::cout<<"Stack1"<<std::endl;
					for(int i=0; i<my_Stacks[0].recog_cubes.size(); i++)
						std::cout << my_Stacks[0].recog_cubes[i].color <<std::endl; 
				}
				else
					std::cout<<"The Stack1 is void"<<std::endl;

				if(my_Stacks[1].recog_cubes.size()>0)
				{
					std::cout<<"Stack2"<<std::endl;
					for(int i=0; i<my_Stacks[1].recog_cubes.size(); i++)
						std::cout << my_Stacks[1].recog_cubes[i].color << std::endl;
				}
				else
					std::cout<<"The Stack2 is void"<<std::endl;

				state=1;

				break;
			case 1:
				finished = true;
				break;
		}
		rate.sleep();
		ros::spinOnce();
	}

	return 0;

}