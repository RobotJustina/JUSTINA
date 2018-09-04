#include "ros/ros.h"

#include "std_msgs/Empty.h"
#include "std_msgs/UInt32.h"

#include <roah_rsbb_comm_ros/Benchmark.h>
#include <roah_rsbb_comm_ros/BenchmarkState.h>

#include <roah_rsbb_comm_ros/ResultHOPF.h>
#include <roah_rsbb_comm_ros/Percentage.h>

class JustinaIROS{
    private:
        ros::NodeHandle * nh;

        static ros::Subscriber subBenchmark;
        static ros::Subscriber subBenchmarkState;
        static ros::Subscriber subBell;
        static ros::Subscriber subTabletCall;

        static ros::Publisher pubMessagesSaved;

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
};
