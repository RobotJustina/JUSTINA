#include "ros/ros.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int32.h"
#include "justina_tools/JustinaHardware.h"
//#include "justina_tools/JustinaIROS.h"

bool startTimeRecived = false;
bool stopPub = false;
bool executing = false;
int timeout = 0;
boost::posix_time::ptime curr;
boost::posix_time::ptime prev;

void callbackStartTime(const std_msgs::Int32::ConstPtr& msg){
    std::cout << "monitor.->Recived start time:" << msg->data << std::endl;
    timeout = msg->data;
    startTimeRecived = true;
    stopPub = false;
    curr = boost::posix_time::second_clock::local_time();
    prev = curr;
}

void callbackRestartTime(const std_msgs::Empty::ConstPtr& msg){
    std::cout << "monitor.->Recived restart time" << std::endl;
    stopPub = false;
}

void callbackStartExecuting(const std_msgs::Empty::ConstPtr& msg)
{
    executing = true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "monitor");
    ros::NodeHandle n;
    ros::Rate rate(10);

    ros::Publisher pubTasksStop = n.advertise<std_msgs::Empty>("/planning/tasks_stop", 1);
    ros::Subscriber subStartTime = n.subscribe("/planning/start_time", 1, callbackStartTime);
    ros::Subscriber subStartExecuting = n.subscribe("/planning/start_executing", 1, callbackStartExecuting);
    ros::Subscriber subRestartTime = n.subscribe("/planning/restart_time", 1, callbackRestartTime);

    JustinaHardware::setNodeHandle(&n);
    //JustinaIROS::setNodeHandle(&n);

    while(ros::ok()){

        //if(JustinaIROS::getLastBenchmarkState() == roah_rsbb_comm_ros::BenchmarkState::STOP && executing)
            //stopPub = true;

        if(startTimeRecived){
            curr = boost::posix_time::second_clock::local_time();
            // std::cout << "monitor.->" << (curr - prev).total_milliseconds() << std::endl;
            if((curr - prev).total_milliseconds() > timeout){
                startTimeRecived = false;
                stopPub = true;
            }
        }

        if(stopPub){
            std_msgs::Empty msg;
            pubTasksStop.publish(msg);
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;	
}
