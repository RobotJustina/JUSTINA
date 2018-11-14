#include "justina_tools/JustinaIROS.h"

ros::Subscriber JustinaIROS::subBenchmark;
ros::Subscriber JustinaIROS::subBenchmarkState;
ros::Subscriber JustinaIROS::subBell;
ros::Subscriber JustinaIROS::subTabletCall;

ros::Publisher JustinaIROS::pubMessagesSaved;

ros::Publisher JustinaIROS::pubCommand;
ros::Publisher JustinaIROS::pubVisitor;
ros::Publisher JustinaIROS::pubNotification;
ros::Publisher JustinaIROS::pubRobotPose;
ros::Publisher JustinaIROS::pubTrajectory;
ros::Publisher JustinaIROS::pubImage1;
ros::Publisher JustinaIROS::pubImage2;
ros::Publisher JustinaIROS::pubDepth;
ros::Publisher JustinaIROS::pubScan1;
ros::Publisher JustinaIROS::pubScan2;

int JustinaIROS::last_benchmark = -1;
int JustinaIROS::last_benchmark_state = -1;
int JustinaIROS::bell = -1;
int JustinaIROS::tabletCall = -1;

JustinaIROS::~JustinaIROS(){}

void JustinaIROS::setNodeHandle(ros::NodeHandle *nh){
    
    JustinaIROS::subBenchmarkState = nh->subscribe("/roah_rsbb/benchmark/state", 1, &JustinaIROS::callbackBenchmarkState);
    JustinaIROS::subBenchmark = nh->subscribe("/roah_rsbb/benchmark", 1, &JustinaIROS::callbackBenchmark);
    JustinaIROS::subBell = nh->subscribe("/roah_rsbb/devices/bell", 1, &JustinaIROS::callbackBell);
    JustinaIROS::subTabletCall = nh->subscribe("/roah_rsbb/tablet/call", 1, &JustinaIROS::callbackTabletCall);

    JustinaNavigation::setNodeHandle(nh);
    
    JustinaIROS::pubMessagesSaved = nh->advertise<std_msgs::UInt32>("/roah_rsbb/messages_saved", 1);

    JustinaIROS::pubCommand = nh->advertise<std_msgs::String>("/ERLSCR/command", 1);
    JustinaIROS::pubVisitor = nh->advertise<std_msgs::String>("/ERLSCR/visitor", 1);
    JustinaIROS::pubNotification = nh->advertise<std_msgs::String>("/ERLSCR/audio", 1);
    JustinaIROS::pubRobotPose = nh->advertise<geometry_msgs::PoseStamped>("/erlc/robot_pose", 1);
    JustinaIROS::pubTrajectory = nh->advertise<nav_msgs::Path>("/erlc/trajectory", 1);
    JustinaIROS::pubImage1 = nh->advertise<sensor_msgs::Image>("/erlc/rgb_1/image", 1);
    JustinaIROS::pubImage2 = nh->advertise<sensor_msgs::Image>("/erlc/rgb_2/image", 1);
    JustinaIROS::pubDepth = nh->advertise<sensor_msgs::PointCloud2>("/erlc/depth_0/pointcloud", 1);
    JustinaIROS::pubScan1 = nh->advertise<sensor_msgs::LaserScan>("/erlc/scan_1", 1);
    JustinaIROS::pubScan2 = nh->advertise<sensor_msgs::LaserScan>("/erlc/scan_2", 1);
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

void JustinaIROS::callbackTabletCall(std_msgs::Empty::ConstPtr const& msg){
    std::cout << "JustinaIROS.-> tablet call";
    tabletCall = 1;
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

int JustinaIROS::getTabletCallState(){
    int tablet_call_state;
    tablet_call_state = JustinaIROS::tabletCall;
    JustinaIROS::tabletCall;
    return tablet_call_state;
}

void JustinaIROS::end_prepare()
{
    if (ros::service::waitForService ("/roah_rsbb/end_prepare", 100)) {
        std_srvs::Empty s;
        if (! ros::service::call ("/roah_rsbb/end_prepare", s)) {
            ROS_ERROR ("Error calling service /roah_rsbb/end_prepare");
        }
    }
    else {
        ROS_ERROR ("Could not find service /roah_rsbb/end_prepare");
    }
}

void JustinaIROS::end_execute()
{
    if (ros::service::waitForService ("/roah_rsbb/end_execute", 100)) {
        std_srvs::Empty s;
        if (! ros::service::call ("/roah_rsbb/end_execute", s)) {
            ROS_ERROR ("Error calling service /roah_rsbb/end_execute");
        } else {

            ROS_INFO("called /roah_rsbb/end_execute");
        }


    }
    else {
        ROS_ERROR ("Could not find service /roah_rsbb/end_execute");
    }
}
       
void JustinaIROS::loggingCommand(std::string command)
{
    std_msgs::String msg;
    msg.data = command;
    JustinaIROS::pubCommand.publish(msg);
}

void JustinaIROS::loggingVisitor(std::string visitior)
{
    std_msgs::String msg;
    msg.data = visitior;
    JustinaIROS::pubVisitor.publish(msg);
}

void JustinaIROS::loggingNotificacion(std::string notification)
{
    std_msgs::String msg;
    msg.data = notification;
    JustinaIROS::pubNotification.publish(msg);
}

void JustinaIROS::loggingRobotPose()
{
    float currx, curry, currtheta;
    JustinaNavigation::getRobotPose(currx, curry, currtheta);
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "/map";
    msg.pose.position.x = currx;
    msg.pose.position.y = curry;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = currtheta;
    msg.pose.orientation.w = 1.0;
    JustinaIROS::pubRobotPose.publish(msg);
}

void JustinaIROS::loggingTrajectory(nav_msgs::Path path)
{

    JustinaIROS::pubTrajectory.publish(path);
}

void JustinaIROS::loggingImage1(cv::Mat image)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    JustinaIROS::pubImage1.publish(msg);
}

void JustinaIROS::loggingImage2(cv::Mat image)
{
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    JustinaIROS::pubImage2.publish(msg);
}

void JustinaIROS::loggingPCL(sensor_msgs::PointCloud2 pcl)
{
}

void JustinaIROS::loggingScan1(sensor_msgs::LaserScan laserScan)
{
    JustinaIROS::pubScan1.publish(laserScan);
}

void JustinaIROS::loggingScan2(sensor_msgs::LaserScan laserScan)
{
    JustinaIROS::pubScan2.publish(laserScan);
}
