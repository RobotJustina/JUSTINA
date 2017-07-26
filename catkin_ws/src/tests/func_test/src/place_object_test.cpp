#include <iostream>
#include "justina_tools/JustinaTasks.h"

int main(int argc, char** argv)
{
    std::cout << "INITIALIZING A TEST JUST FOR TEST BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "act_pln");
    ros::NodeHandle n;
    JustinaTasks::setNodeHandle(&n);
    ros::Rate loop(10);

    bool fail = false; 
    bool success = false;

    std::vector<float> point;

    while(ros::ok() && !fail && !success){
        //JustinaTasks::placeObject(true, 0.15);
        //boost::this_thread::sleep(boost::posix_time::milliseconds(10000));
        //JustinaTasks::placeObject(false, 0.10);
        
        
        //if (JustinaVision::findTable(point) )
        //{
        //    std::cout << "p_x:  " << point[0] << std::endl;
        //    std::cout << "p_y:  " << point[1] << std::endl;
        //    std::cout << "p_z:  " << point[2] << std::endl;
        //}
      
        //success = JustinaTasks::findAndAlignTable();
        
        success = JustinaTasks::placeObjectOnShelf(true, 0.0);
      
        ros::spinOnce();
        loop.sleep();
    }
          
    return 0;
}
