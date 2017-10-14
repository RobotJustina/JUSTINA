#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PointStamped.h"
#include "visualization_msgs/Marker.h"

//Constants to find leg hypothesis
#define FILTER_THRESHOLD  .081
#define FLANK_THRESHOLD  .04
#define HORIZON_THRESHOLD  25
#define MAX_FLOAT  57295779500
#define PIERNA_DELGADA  0.006241//7.9CM,0.006241,6241
#define PIERNA_GRUESA  0.037//19.23CM,0.037,37000
#define DOS_PIERNAS_DELGADAS  0.056644//23.8CM,0.056644,56644
#define DOS_PIERNAS_GRUESAS  0.25//50CM,0.25,250000
#define DOS_PIERNAS_CERCAS  0.022201//14.9CM,0.022201,22201
#define DOS_PIERNAS_LEJOS  0.16//40CM,0.16,160000
//Constants to check if there are legs in front of the robot
#define IN_FRONT_MIN_X  0.25
#define IN_FRONT_MAX_X  1.5
#define IN_FRONT_MIN_Y -0.5
#define IN_FRONT_MAX_Y  0.5
//BUTTERWORTH FILTER A Ó B EN X O Y
//cutoff frequency X: 0.7
//                 Y: 0.2
//lowpass filter
//#define BFA0X 1.0
//#define BFA1X 1.161917483671732
//#define BFA2X 0.695942755789651
//#define BFA3X 0.137761301259893
//#define BFB0X 0.374452692590159
//#define BFB1X 1.123358077770478
//#define BFB2X 1.123358077770478
//#define BFB3X 0.374452692590159
#define BFA0X 1.0
#define BFA1X -1.760041880343169
#define BFA2X 1.182893262037831
#define BFA3X -0.278059917634546
#define BFB0X 0.018098933007514
#define BFB1X 0.054296799022543
#define BFB2X 0.054296799022543
#define BFB3X 0.018098933007514
#define BFA0Y 1.0
#define BFA1Y -1.760041880343169
#define BFA2Y 1.182893262037831
#define BFA3Y -0.278059917634546
#define BFB0Y 0.018098933007514
#define BFB1Y 0.054296799022543
#define BFB2Y 0.054296799022543
#define BFB3Y 0.018098933007514


ros::NodeHandle* n;
ros::Subscriber subLaserScan;
ros::Publisher pub_legs_hypothesis;
ros::Publisher pub_legs_pose;      
ros::Publisher pub_legs_found;     
bool show_hypothesis   = false;
bool legs_found        = false;
int  legs_in_front_cnt = 0;
int  legs_lost_counter = 0;
float last_legs_pose_x = 0;
float last_legs_pose_y = 0;
std::vector<float> legs_x_filter_input;
std::vector<float> legs_x_filter_output;
std::vector<float> legs_y_filter_input;
std::vector<float> legs_y_filter_output;
std::string frame_id;

std::vector<float> filter_laser_ranges(std::vector<float>& laser_ranges)
{
    std::vector<float> filtered_ranges;
    filtered_ranges.resize(laser_ranges.size());
    filtered_ranges[0] = 0;
    int i = 0;
    int max_idx = laser_ranges.size() - 1;
    bool is_cluster = false;

    while(++i < max_idx)
        if(laser_ranges[i] < 0.4)
            filtered_ranges[i] = 0;

        else if(fabs(laser_ranges[i-1] - laser_ranges[i]) < FILTER_THRESHOLD &&
                fabs(laser_ranges[i] - laser_ranges[i+1] < FILTER_THRESHOLD))
            filtered_ranges[i] = (laser_ranges[i-1] + laser_ranges[i] + laser_ranges[i+1])/3.0;

        else if(fabs(laser_ranges[i-1] - laser_ranges[i]) < FILTER_THRESHOLD)
            filtered_ranges[i] = (laser_ranges[i-1] + laser_ranges[i])/2.0;

        else if(fabs(laser_ranges[i] - laser_ranges[i+1]) < FILTER_THRESHOLD)
            filtered_ranges[i] = (laser_ranges[i] + laser_ranges[i+1])/2.0;
        else
            filtered_ranges[i] = 0;
    
    filtered_ranges[i] = 0;
    return filtered_ranges;
}

bool is_leg(float x1, float y1, float x2, float y2)
{
    bool result = false;
    float m1, m2, px, py, angle;
    if(x1 != x2) m1 = (y1 - y2)/(x1 - x2);
    else m1 = MAX_FLOAT;

    px = (x1 + x2) / 2;
    py = (y1 + y2) / 2;
    if((px*px + py*py) < HORIZON_THRESHOLD)
    {
        if(px != 0)
            m2 = py / px;
        else
            m2 = MAX_FLOAT;
        angle = fabs((m2 - m1) / (1 + (m2*m1)));
        if(angle > 1.999)
            result = true;
    }
    return result;
}

void find_leg_hypothesis(sensor_msgs::LaserScan& laser, std::vector<float>& legs_x, std::vector<float>& legs_y)
{
    std::vector<float> laser_x;
    std::vector<float> laser_y;
    laser_x.resize(laser.ranges.size());
    laser_y.resize(laser.ranges.size());
    float theta = laser.angle_min;
    for(size_t i=0; i < laser.ranges.size(); i++)
    {
        theta = laser.angle_min + i*laser.angle_increment;
        laser_x[i] = laser.ranges[i] * cos(theta);
        laser_y[i] = laser.ranges[i] * sin(theta);
    }

    std::vector<float> flank_x;
    std::vector<float> flank_y;
    std::vector<bool>  flank_id;
    int ant2 = 0;
    float px, py, sum_x, sum_y, cua_x, cua_y;

    legs_x.clear();
    legs_y.clear();
    for(int i=1; i < laser.ranges.size(); i++)
    {
        int ant = ant2;
	if(fabs(laser.ranges[i] - laser.ranges[i-1]) > FLANK_THRESHOLD) ant2 = i;
        if(fabs(laser.ranges[i] - laser.ranges[i-1]) > FLANK_THRESHOLD &&
	   (is_leg(laser_x[ant], laser_y[ant], laser_x[i-1], laser_y[i-1]) || 
	    is_leg(laser_x[ant+1], laser_y[ant+1], laser_x[i-2], laser_y[i-2])))
        {
            if((pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) > PIERNA_DELGADA &&
               (pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) < PIERNA_GRUESA)
	    {
		sum_x = 0;
		sum_y = 0;
		for(int j= ant; j < i; j++)
		{
		    sum_x += laser_x[j];
		    sum_y += laser_y[j];
		}
		flank_x.push_back(sum_x / (float)(i - ant));
		flank_y.push_back(sum_y / (float)(i - ant));
		flank_id.push_back(false);
	    }
            else if((pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) > DOS_PIERNAS_DELGADAS &&
                    (pow(laser_x[ant] - laser_x[i-1], 2) + pow(laser_y[ant] - laser_y[i-1], 2)) < DOS_PIERNAS_GRUESAS)
	    {
		sum_x = 0;
		sum_y = 0;
		for(int j= ant; j < i; j++)
		{
		    sum_x += laser_x[j];
		    sum_y += laser_y[j];
		}
		cua_x = sum_x / (float)(i - ant);
		cua_y = sum_y / (float)(i - ant);
		legs_x.push_back(cua_x);
		legs_y.push_back(cua_y);   
            }
        }
    }

    for(int i=0; i < (int)(flank_x.size())-2; i++)
        for(int j=1; j < 3; j++)
            if((pow(flank_x[i] - flank_x[i+j], 2) + pow(flank_y[i] - flank_y[i+j], 2)) > DOS_PIERNAS_CERCAS &&
               (pow(flank_x[i] - flank_x[i+j], 2) + pow(flank_y[i] - flank_y[i+j], 2)) < DOS_PIERNAS_LEJOS)
            {
                px = (flank_x[i] + flank_x[i + j])/2;
                py = (flank_y[i] + flank_y[i + j])/2;
                if((px*px + py*py) < HORIZON_THRESHOLD)
                {
                    cua_x = px;
                    cua_y = py;
                    legs_x.push_back(cua_x);
                    legs_y.push_back(cua_y);
                    flank_id[i] = true;
                    flank_id[i+j] = true;
                }
            }

    if(flank_y.size() > 1 &&
       (pow(flank_x[flank_x.size()-2] - flank_x[flank_x.size()-1], 2) +
	pow(flank_y[flank_y.size()-2] - flank_y[flank_y.size()-1], 2)) > DOS_PIERNAS_CERCAS &&
       (pow(flank_x[flank_x.size()-2] - flank_x[flank_x.size()-1], 2) +
	pow(flank_y[flank_y.size()-2] - flank_y[flank_y.size()-1], 2)) < DOS_PIERNAS_LEJOS)
    {
	px = (flank_x[flank_x.size()-2] + flank_x[flank_x.size()-1])/2.0;
	py = (flank_y[flank_y.size()-2] + flank_y[flank_y.size()-1])/2.0;
	if((px*px + py*py) < HORIZON_THRESHOLD)
	{
	    cua_x = px;
	    cua_y = py;
	    legs_x.push_back(cua_x);
	    legs_y.push_back(cua_y);
	    flank_id[flank_y.size() - 2] = true;
	    flank_id[flank_y.size() - 1] = true;
	}
    }

    for(int i=0; i < flank_y.size(); i++)
	if(!flank_id[i])
        {
            float cua_x, cua_y;
            cua_x = flank_x[i];
            cua_y = flank_y[i];
            legs_x.push_back(cua_x);
            legs_y.push_back(cua_y);
        }
    
    //std::cout << "LegFinder.->Found " << legs_x.size() << " leg hypothesis" << std::endl;
}

visualization_msgs::Marker get_hypothesis_marker(std::vector<float>& legs_x, std::vector<float>& legs_y)
{
    visualization_msgs::Marker marker_legs;
    marker_legs.header.stamp = ros::Time::now();
    marker_legs.header.frame_id = frame_id;
    marker_legs.ns = "leg_finder";
    marker_legs.id = 0;
    marker_legs.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_legs.action = visualization_msgs::Marker::ADD;
    marker_legs.scale.x = 0.07;
    marker_legs.scale.y = 0.07;
    marker_legs.scale.z = 0.07;
    marker_legs.color.a = 1.0;
    marker_legs.color.r = 0;
    marker_legs.color.g = 0.5;
    marker_legs.color.b = 0;
    marker_legs.points.resize(legs_y.size());
    marker_legs.lifetime = ros::Duration(1.0);
    for(int i=0; i < legs_y.size(); i++)
    {
        marker_legs.points[i].x = legs_x[i];
        marker_legs.points[i].y = legs_y[i];
        marker_legs.points[i].z = 0.3;
    }
    return marker_legs;
}

bool get_nearest_legs_in_front(std::vector<float>& legs_x, std::vector<float>& legs_y, float& nearest_x, float& nearest_y)
{
    nearest_x = MAX_FLOAT;
    nearest_y = MAX_FLOAT;
    float min_dist = MAX_FLOAT;
    for(int i=0; i < legs_x.size(); i++)
    {
	if(!(legs_x[i] > IN_FRONT_MIN_X && legs_x[i] < IN_FRONT_MAX_X && legs_y[i] > IN_FRONT_MIN_Y && legs_y[i] < IN_FRONT_MAX_Y))
	    continue;
	float dist = sqrt(legs_x[i]*legs_x[i] + legs_y[i]*legs_y[i]);
	if(dist < min_dist)
	{
	    min_dist = dist;
	    nearest_x = legs_x[i];
	    nearest_y = legs_y[i];
	}
    }
    return nearest_x > IN_FRONT_MIN_X && nearest_x < IN_FRONT_MAX_X && nearest_y > IN_FRONT_MIN_Y && nearest_y < IN_FRONT_MAX_Y;
}

bool get_nearest_legs_to_last_legs(std::vector<float>& legs_x, std::vector<float>& legs_y, float& nearest_x,
				   float& nearest_y, float last_x, float last_y)
{
    nearest_x = MAX_FLOAT;
    nearest_y = MAX_FLOAT;
    float min_dist = MAX_FLOAT;
    for(int i=0; i < legs_x.size(); i++)
    {
	float dist = sqrt((legs_x[i] - last_x)*(legs_x[i] - last_x) + (legs_y[i] - last_y)*(legs_y[i] - last_y));
	if(dist < min_dist)
	{
	    min_dist = dist;
	    nearest_x = legs_x[i];
	    nearest_y = legs_y[i];
	}
    }
    return min_dist < 0.33;
    /*
    if(min_dist > 0.5)
    {
	nearest_x = last_x;
	nearest_y = last_y;
	return false;
    }
    return true;*/
}

void callback_scan(const sensor_msgs::LaserScan::Ptr& msg)
{
    msg->ranges = filter_laser_ranges(msg->ranges);
    std::vector<float> legs_x, legs_y;
    find_leg_hypothesis(*msg, legs_x, legs_y);
    if(show_hypothesis)
	pub_legs_hypothesis.publish(get_hypothesis_marker(legs_x, legs_y));

    float nearest_x, nearest_y;
    if(!legs_found)
    {
	if(get_nearest_legs_in_front(legs_x, legs_y, nearest_x, nearest_y))
	    legs_in_front_cnt++;
	if(legs_in_front_cnt > 20)
	{
	    legs_found = true;
	    legs_lost_counter = 0;
	    last_legs_pose_x = nearest_x;
	    last_legs_pose_y = nearest_y;
	    for(int i=0; i < 4; i++)
	    {
		legs_x_filter_input[i]  = nearest_x;
		legs_x_filter_output[i] = nearest_x;
		legs_y_filter_input[i]  = nearest_y;
		legs_y_filter_output[i] = nearest_y;
	    }
	}
    }
    else
    {
	geometry_msgs::PointStamped filtered_legs;
	filtered_legs.header.frame_id = frame_id;
	filtered_legs.point.z = 0.3;
	
	
	//float diff = sqrt((nearest_x - last_legs_pose_x)*(nearest_x - last_legs_pose_x) +
	//		  (nearest_y - last_legs_pose_y)*(nearest_y - last_legs_pose_y));
	bool publish_legs = false;
	if(get_nearest_legs_to_last_legs(legs_x, legs_y, nearest_x, nearest_y, last_legs_pose_x, last_legs_pose_y))
	{
	    last_legs_pose_x = nearest_x;
	    last_legs_pose_y = nearest_y;
	    legs_x_filter_input.insert(legs_x_filter_input.begin(), nearest_x);
	    legs_y_filter_input.insert(legs_y_filter_input.begin(), nearest_y);
	    legs_lost_counter = 0;
	    publish_legs = true;
	}
	else
	{
	    legs_x_filter_input.insert(legs_x_filter_input.begin(), last_legs_pose_x);
	    legs_y_filter_input.insert(legs_y_filter_input.begin(), last_legs_pose_y);
	    if(++legs_lost_counter > 20)
	    {
		legs_found = false;
		legs_in_front_cnt = 0;
	    }
	}
	legs_x_filter_input.pop_back();
	legs_y_filter_input.pop_back();
	legs_x_filter_output.pop_back();
	legs_y_filter_output.pop_back();
	legs_x_filter_output.insert(legs_x_filter_output.begin(), 0);
	legs_y_filter_output.insert(legs_y_filter_output.begin(), 0);
	
	legs_x_filter_output[0]  = BFB0X*legs_x_filter_input[0] + BFB1X*legs_x_filter_input[1] +
	    BFB2X*legs_x_filter_input[2] + BFB3X*legs_x_filter_input[3];
	legs_x_filter_output[0] -= BFA1X*legs_x_filter_output[1] + BFA2X*legs_x_filter_output[2] + BFA3X*legs_x_filter_output[3];

	legs_y_filter_output[0]  = BFB0Y*legs_y_filter_input[0] + BFB1Y*legs_y_filter_input[1] +
	    BFB2Y*legs_y_filter_input[2] + BFB3Y*legs_y_filter_input[3];
	legs_y_filter_output[0] -= BFA1Y*legs_y_filter_output[1] + BFA2Y*legs_y_filter_output[2] + BFA3Y*legs_y_filter_output[3];

	filtered_legs.point.x = legs_x_filter_output[0];
	filtered_legs.point.y = legs_y_filter_output[0];

	if(publish_legs)
	  pub_legs_pose.publish(filtered_legs);
    }
    std_msgs::Bool msg_found;
    msg_found.data = legs_found;
    pub_legs_found.publish(msg_found);
}

void callback_enable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
	subLaserScan = n->subscribe("/hardware/scan", 1, callback_scan);
    else
    {
	subLaserScan.shutdown();
	legs_found = false;
	legs_in_front_cnt = 0;
    }
}

int main(int argc, char** argv)
{
    show_hypothesis = false;
    for(int i=0; i < argc; i++)
    {
        std::string strParam(argv[i]);
        if(strParam.compare("--hyp") == 0)
            show_hypothesis = true;
    }
    
    std::cout << "INITIALIZING LEG FINDER BY MARCOSOFT..." << std::endl;
    ros::init(argc, argv, "leg_finder");
    n = new ros::NodeHandle();
    ros::Subscriber subEnable = n->subscribe("/hri/leg_finder/enable", 1, callback_enable);
    pub_legs_hypothesis = n->advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);
    pub_legs_pose       = n->advertise<geometry_msgs::PointStamped>("/hri/leg_finder/leg_poses", 1);
    pub_legs_found      = n->advertise<std_msgs::Bool>("/hri/leg_finder/legs_found", 1);            
    //n->getParam("~frame_id", frame_id);
    if(ros::param::has("~frame_id"))
        ros::param::get("~frame_id", frame_id);
    if(frame_id.compare("") == 0)
        frame_id = "base_link";

    ros::Rate loop(20);

    for(int i=0; i < 4; i++)
    {
	legs_x_filter_input.push_back(0);
	legs_x_filter_output.push_back(0);
	legs_y_filter_input.push_back(0);
	legs_y_filter_output.push_back(0);
    }

    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
    }
    delete n;
    return 0;
}
