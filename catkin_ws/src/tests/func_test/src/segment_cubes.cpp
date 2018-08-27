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
	my_cubes.recog_cubes.resize(6);
	int nStacks = 0;

	my_cubes.recog_cubes[0].color="red";
	my_cubes.recog_cubes[1].color="green";
	my_cubes.recog_cubes[2].color="blue";
	my_cubes.recog_cubes[3].color="purple";
	my_cubes.recog_cubes[4].color="orange";
	my_cubes.recog_cubes[5].color="yellow";

	while(ros::ok() && cv::waitKey(1) != 'q'){
		

		switch(state){
			case 0:
				JustinaVision::getCutlerySeg(my_cubes); 
				for(int i =0; i < my_cubes.recog_cubes.size(); i ++){
					std::cout << "el objeto de color: " << my_cubes.recog_cubes[i].color << 
					"con centroide: " << my_cubes.recog_cubes[i].cube_centroid.x << ", " << 
					my_cubes.recog_cubes[i].cube_centroid.y << ", " <<
					my_cubes.recog_cubes[i].cube_centroid.z <<std::endl; 
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
