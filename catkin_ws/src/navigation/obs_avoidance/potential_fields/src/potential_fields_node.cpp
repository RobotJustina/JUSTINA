#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>

ros::ServiceClient cltRgbdRobotDownsampled;

bool initVectorForces = false;
std::vector<float> repulsiveForces;
geometry_msgs::PoseArray poseRepulsiveForces;
geometry_msgs::PoseArray poseRepulsiveForce;
sensor_msgs::LaserScan laserScan;

void callbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(!initVectorForces){
        poseRepulsiveForces.poses.resize(msg->ranges.size());
        poseRepulsiveForce.poses.resize(1);
        poseRepulsiveForces.header.frame_id = "base_link";
        poseRepulsiveForce.header.frame_id = "base_link";
        repulsiveForces.resize(msg->ranges.size());
        initVectorForces = true;
    }
    laserScan = *msg; 
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "potential_fields_node");
    ros::NodeHandle nh;
    ros::Rate rate(30);
    std::string topicScan = "/hardware/scan";
    float distanceMin = 0.35;
    float Kr = 0.0;

    ros::param::get("~topic_scan", topicScan);
    ros::param::get("~distance_min", distanceMin);
    ros::param::get("~k_r", Kr);


    ros::Subscriber subLaserScan = nh.subscribe(topicScan, 1, callbackLaserScan);
    ros::Publisher pubRepulsionForce = nh.advertise<std_msgs::Float32>("potential_fields/repulsive_force", 1);
    ros::Publisher pubPoseRepulsionForces = nh.advertise<geometry_msgs::PoseArray>("potential_fields/pose_repulsive_forces", 1);
    ros::Publisher pubPoseRepulsionForce = nh.advertise<geometry_msgs::PoseArray>("potential_fields/pose_repulsive_force", 1);


    while(ros::ok()){
        if(initVectorForces){
            for(int i = 0 ; i < laserScan.ranges.size() ; i++){
                float angle = laserScan.angle_min + ( i * laserScan.angle_increment);
                //std::cout << i << "," << angle << std::endl;
                //std::cout << "distance_min" << distanceMin << std::endl;
                repulsiveForces[i] = 0;
                poseRepulsiveForces.poses[i].orientation.z = 0.0;
                poseRepulsiveForces.poses[i].position.x = 0.0;
                poseRepulsiveForces.poses[i].position.y = 0.0;
                //if(laserScan.ranges[i] > 0.2 && laserScan.ranges[i] < distanceMin){
                if(laserScan.ranges[i] > 0.08 && laserScan.ranges[i] < distanceMin && angle >= -M_PI_2 - 0.52 && angle <= M_PI_2 + 0.52){
                    poseRepulsiveForces.poses[i].position.x = laserScan.ranges[i] * cos(laserScan.angle_min + ( i * laserScan.angle_increment));
                    poseRepulsiveForces.poses[i].position.y = laserScan.ranges[i] * sin(laserScan.angle_min + ( i * laserScan.angle_increment));
                    float sRep = 1.0 / laserScan.ranges[i] - 1.0 / distanceMin;
                    float fRepY = -sqrt(sRep) * sin(laserScan.angle_min + ( i * laserScan.angle_increment));
                    repulsiveForces[i] = Kr * fRepY;
                    //std::cout << repulsiveForces[i] << std::endl;
                    //std::cout << repulsiveForces[i] << std::endl;
                    if(fRepY > 0){
                        poseRepulsiveForces.poses[i].orientation.z = 1.0;
                        poseRepulsiveForces.poses[i].orientation.w = 1.0;
                    }else{
                        poseRepulsiveForces.poses[i].orientation.z = -1.0;
                        poseRepulsiveForces.poses[i].orientation.w = 1.0;
                    }
                }
            }
            float meanRepulsionForce = 0.0;
            for(int i = 0; i < repulsiveForces.size(); i++)
                meanRepulsionForce += repulsiveForces[i];

            //std::cout << "potential_fields_node.->meanRepulsionForce :" << meanRepulsionForce << std::endl;
            meanRepulsionForce /= repulsiveForces.size();
            //std::cout << "potential_fields_node.->meanRepulsionForce prom:" << meanRepulsionForce << std::endl;

            if(meanRepulsionForce > 0){
                poseRepulsiveForce.poses[0].orientation.z = 1.0;
                poseRepulsiveForce.poses[0].orientation.w = 1.0;
                poseRepulsiveForce.poses[0].position.x = 0.35;
                poseRepulsiveForce.poses[0].position.y = -0.35;
            }else if(meanRepulsionForce < 0){
                poseRepulsiveForce.poses[0].orientation.z = -1.0;
                poseRepulsiveForce.poses[0].orientation.w = 1.0;
                poseRepulsiveForce.poses[0].position.x = 0.35;
                poseRepulsiveForce.poses[0].position.y = 0.35;
            }
            else{
                poseRepulsiveForce.poses[0].orientation.z = 0.0;
                poseRepulsiveForce.poses[0].orientation.w = 0.0;
                poseRepulsiveForce.poses[0].position.x = 0.35;
                poseRepulsiveForce.poses[0].position.y = 0.0;
            }

            std_msgs::Float32 msgRepulsionForce;
            msgRepulsionForce.data = meanRepulsionForce;
            pubPoseRepulsionForces.publish(poseRepulsiveForces);
            pubPoseRepulsionForce.publish(poseRepulsiveForce);
            pubRepulsionForce.publish(msgRepulsionForce);
        }
        rate.sleep();
        ros::spinOnce();
    }
}
