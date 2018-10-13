/*#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "std_msgs/UInt32.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"

#include <roah_rsbb_comm_ros/Benchmark.h>
#include <roah_rsbb_comm_ros/BenchmarkState.h>

#include <roah_rsbb_comm_ros/ResultHOPF.h>
#include <roah_rsbb_comm_ros/Percentage.h>

#include <std_srvs/Empty.h>

#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/PoseStamped.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/PointCloud2.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <justina_tools/JustinaNavigation.h>

class JustinaIROS{
    private:
        ros::NodeHandle * nh;

        static ros::Subscriber subBenchmark;
        static ros::Subscriber subBenchmarkState;
        static ros::Subscriber subBell;
        static ros::Subscriber subTabletCall;

        static ros::Publisher pubMessagesSaved;

        static ros::Publisher pubCommand;
        static ros::Publisher pubVisitor;
        static ros::Publisher pubNotification;
        static ros::Publisher pubRobotPose;
        static ros::Publisher pubTrajectory;
        static ros::Publisher pubImage1;
        static ros::Publisher pubImage2;
        static ros::Publisher pubDepth;
        static ros::Publisher pubScan1;
        static ros::Publisher pubScan2;

        static int last_benchmark;
        static int last_benchmark_state;
        static int bell;
        static int tabletCall;

    public:
        ~JustinaIROS();
        static void setNodeHandle(ros::NodeHandle * nh);

        static void callbackBenchmark(roah_rsbb_comm_ros::Benchmark::ConstPtr const& msg);
        static void callbackBenchmarkState(roah_rsbb_comm_ros::BenchmarkState::ConstPtr const& msg);
        static void callbackBell(std_msgs::Empty::ConstPtr const& msg);
        static void callbackTabletCall(std_msgs::Empty::ConstPtr const& msg);

        static void MessagesSaved(int size);

        static int getLastBenchmark();
        static int getLastBenchmarkState();
        static int getBellState();
        static int getTabletCallState();
        static void end_prepare(); 
        static void end_execute();

        static void loggingCommand(std::string command);
        static void loggingVisitor(std::string visitior);
        static void loggingNotificacion(std::string notification);

        static void loggingRobotPose();
        static void loggingTrajectory(nav_msgs::Path path);
        static void loggingImage1(cv::Mat image);
        static void loggingImage2(cv::Mat image);
        static void loggingPCL(sensor_msgs::PointCloud2 pcl);
        static void loggingScan1(sensor_msgs::LaserScan laserScan);
        static void loggingScan2(sensor_msgs::LaserScan laserScan);
};*/
