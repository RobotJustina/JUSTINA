#include "justina_tools/JustinaIROS.h"

ros::Subscriber JustinaIROS::subBenchmark;
ros::Subscriber JustinaIROS::subBenchmarkState;
ros::Subscriber JustinaIROS::subBell;
ros::Publisher JustinaIROS::pubMessagesSaved;

int JustinaIROS::last_benchmark = -1;
int JustinaIROS::last_benchmark_state = -1;
int JustinaIROS::bell = -1;

JustinaIROS::~JustinaIROS(){}

void JustinaIROS::setNodeHandle(ros::NodeHandle *nh){
    
    JustinaIROS::subBenchmarkState = nh->subscribe("/roah_rsbb/benchmark/state", 1, &JustinaIROS::callbackBenchmarkState);
    JustinaIROS::subBenchmark = nh->subscribe("/roah_rsbb/benchmark", 1, &JustinaIROS::callbackBenchmark);
    JustinaIROS::subBell = nh->subscribe("/roah_rsbb/devices/bell", 1, &JustinaIROS::callbackBell);

    JustinaIROS::pubMessagesSaved = nh->advertise<std_msgs::UInt32>("/roah_rsbb/messages_saved", 1);
} 


void JustinaIROS::callbackBenchmark(roah_rsbb_comm_ros::Benchmark::ConstPtr const& msg){
    std::cout << "JustinaIROS.-> benchmark_callback";
    last_benchmark = msg->benchmark;
    std::cout << "benchmark num : " << msg->benchmark << std::endl;
}

void JustinaIROS::callbackBenchmarkState(roah_rsbb_comm_ros::BenchmarkState::ConstPtr const& msg){
    std::cout << "JustinaIROS.-> benchmark_state_callback";
    last_benchmark_state = msg->benchmark_state;
    std::cout << "benchmark state num: " << msg->benchmark_state << std::endl;
}

void JustinaIROS::callbackBell(std_msgs::Empty::ConstPtr const& msg){
    std::cout << "JustinaIROS.-> bell";
    bell = 1;
}

void JustinaIROS::MessagesSaved(int size){
    std_msgs::UInt32 messages_saved_msg;
    messages_saved_msg.data = size;
    JustinaIROS::pubMessagesSaved.publish(messages_saved_msg);
}

int JustinaIROS::getLastBenchmark(){
    int benchmark;
    benchmark = JustinaIROS::last_benchmark;
    JustinaIROS::last_benchmark = -1;
    return benchmark;
}

int JustinaIROS::getLastBenchmarkState(){
    int benchmark_state;
    benchmark_state = JustinaIROS::last_benchmark_state;
    JustinaIROS::last_benchmark_state = -1;
    return benchmark_state;
}

int JustinaIROS::getBellState(){
    int bell_state;
    bell_state = JustinaIROS::bell;
    JustinaIROS::bell = -1;
    return bell_state;
}
