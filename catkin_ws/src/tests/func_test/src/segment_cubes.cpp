#include "ros/ros.h"
#include "justina_tools/JustinaVision.h"
#include "justina_tools/JustinaTasks.h"

int main(int argc, char ** argv){
	
	std::cout << "INITIALIZING A TEST JUST FOR TEST..." << std::endl;
	ros::init(argc, argv, "segment_cubes_test");
	ros::NodeHandle nh;

	JustinaTasks::setNodeHandle(&nh);
	ros::Rate loop(10);
	
	int state = 0;
	bool finished = false;
	vision_msgs::CubesSegmented my_cubes;
	std::vector<vision_msgs::CubesSegmented> my_Stacks;
	my_Stacks.resize(3);
	//my_cubes.recog_cubes.resize(4);

	vision_msgs::VisionObjectList my_cutlery;	  

	my_cutlery.ObjectList.resize(8); 

	my_cutlery.ObjectList[0].id="red";
	my_cutlery.ObjectList[1].id="green";
	my_cutlery.ObjectList[2].id="green_2";
	my_cutlery.ObjectList[3].id="lemon";
	my_cutlery.ObjectList[4].id="melon";
	my_cutlery.ObjectList[5].id="blue";
	my_cutlery.ObjectList[6].id="pink_1";
	my_cutlery.ObjectList[7].id="yellow";

	while(ros::ok() && cv::waitKey(1) != 'q'){
		

		switch(state){
			case 0:
				JustinaVision::getObjectSeg(my_cutlery); 
				for(int i =0; i < my_cutlery.ObjectList.size(); i ++){
					std::cout << "el objeto de color: " << my_cutlery.ObjectList[i].id << 
					"con centroide: " << my_cutlery.ObjectList[i].pose.position.x << ", " << 
					my_cutlery.ObjectList[i].pose.position.y << ", " <<
					my_cutlery.ObjectList[i].pose.position.z <<std::endl; 
				} 
				//JustinaVision::getCubesSeg(my_cubes);
				//JustinaTasks::sortCubes(my_cubes, my_Stacks);
				//JustinaTasks::getStacks(my_cubes, my_Stacks, nStacks);

				//std::cout << "el numero de pilas es: " << nStacks <<std::endl; 

				
				/*if(my_Stacks[0].recog_cubes.size()>0)
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

				if(my_Stacks[2].recog_cubes.size()>0)
				{
					std::cout<<"Stack3"<<std::endl;
					for(int i=0; i<my_Stacks[2].recog_cubes.size(); i++)
						std::cout << my_Stacks[2].recog_cubes[i].color << std::endl;
				}
				else
					std::cout<<"The Stack3 is void"<<std::endl;*/

				state=0;

				break;
			case 1:
				finished = true;
				break;
		}
		loop.sleep();
		ros::spinOnce();
	}

	return 0;

}
