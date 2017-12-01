#include <iostream>
#include "justina_tools/JustinaTasks.h"
#define SM_LARM_RISING  10
#define SM_RARM_RISING  20
#define SM_LARM_FALLING 30
#define SM_RARM_FALLING 40
#define SM_BOTH_RISING  50
#define SM_BOTH_FALLING 60

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "arms_test");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    int nextState = 0;
    int time_rnd = 0;

    
    while(ros::ok()){
      time_rnd = int( (rand() % 17) * 1000);
      
      switch(nextState){

      case 0:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_WAITING" << std::endl;
	for(int i = 0; i < 30; i++)
	  {
	    std::cout << "Time: " << int(30-i) << std::endl; 
	    boost::this_thread::sleep( boost::posix_time::milliseconds(1000) );
	  }
	nextState = SM_RARM_RISING;
	break;
	
      case SM_RARM_RISING:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_RARM_RISING" << std::endl;
	JustinaManip::startRaGoTo("navigation");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
	JustinaManip::startRaGoTo("take");
	boost::this_thread::sleep( boost::posix_time::milliseconds(time_rnd) );
	nextState = SM_LARM_RISING;
	break;

      case SM_LARM_RISING:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_LARM_RISING" << std::endl;
	JustinaManip::startLaGoTo("navigation");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
	JustinaManip::startLaGoTo("take");
	boost::this_thread::sleep( boost::posix_time::milliseconds(time_rnd) );
	nextState = SM_RARM_FALLING;
	break;

      case SM_RARM_FALLING:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_RARM_FALLING" << std::endl;
	JustinaManip::startRaGoTo("navigation");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
	JustinaManip::startRaGoTo("home");
	boost::this_thread::sleep( boost::posix_time::milliseconds(time_rnd) );
	nextState = SM_LARM_FALLING;
	break;

      case SM_LARM_FALLING:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_LARM_FALLING" << std::endl;
	JustinaManip::startLaGoTo("navigation");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
	JustinaManip::startLaGoTo("home");
	boost::this_thread::sleep( boost::posix_time::milliseconds(time_rnd) );
	nextState = SM_BOTH_RISING;
	break;

      case SM_BOTH_RISING:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_BOTH_RISING" << std::endl;
        JustinaManip::startLaGoTo("take");
	JustinaManip::startRaGoTo("take");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
	JustinaManip::startLaGoTo("navigation");
	JustinaManip::startRaGoTo("navigation");
	boost::this_thread::sleep( boost::posix_time::milliseconds(time_rnd) );
	nextState = SM_BOTH_FALLING;
	break;

      case SM_BOTH_FALLING:
	std::cout << "-----------------" << std::endl;
	std::cout << "SM_BOTH_FALLING" << std::endl;
	JustinaManip::startLaGoTo("take");
	JustinaManip::startRaGoTo("take");
	boost::this_thread::sleep( boost::posix_time::milliseconds(3000) );
	JustinaManip::startLaGoTo("home");
	JustinaManip::startRaGoTo("home");
	boost::this_thread::sleep( boost::posix_time::milliseconds(10000) );
	nextState = SM_RARM_RISING;
	break;

      }
      ros::spinOnce();
      loop.sleep();
    }
    
    return 0;
}
