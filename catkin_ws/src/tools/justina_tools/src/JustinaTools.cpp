#include "justina_tools/JustinaTools.h"

bool JustinaTools::is_node_set = false;
tf::TransformListener* JustinaTools::tf_listener;
int JustinaTools::counter = 0;

bool JustinaTools::setNodeHandle(ros::NodeHandle* nh)
{
    std::cout << "JustinaTools.->Setting ros node..." << std::endl;
	tf_listener = new tf::TransformListener();
    tf_listener->waitForTransform("map", "laser_link", ros::Time(0), ros::Duration(10.0));
    tf_listener->waitForTransform("base_link", "left_arm_link1", ros::Time(0), ros::Duration(10.0));
    tf_listener->waitForTransform("base_link", "right_arm_link1", ros::Time(0), ros::Duration(10.0));
}

void JustinaTools::laserScanToStdVectors(sensor_msgs::LaserScan& readings, std::vector<float>& robotX, std::vector<float>& robotY, std::vector<float>& mapX, std::vector<float>& mapY)
{
	
}

void JustinaTools::laserScanToPclWrtRobot(const sensor_msgs::LaserScan::ConstPtr& readings, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	tf::StampedTransform transformTf;
    tf_listener->lookupTransform("base_link","laser_link", ros::Time(0), transformTf);
    //std::cout<<"ya encontre la tr "<< (++counter) << std::endl;
    Eigen::Affine3d transformEigen;
    tf::transformTFToEigen(transformTf, transformEigen);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLaser(new pcl::PointCloud<pcl::PointXYZ>);
	cloudLaser->width = readings->ranges.size();
	cloudLaser->height = 1;
	cloudLaser->points.resize(cloudLaser->width * cloudLaser->height);
	for (size_t i = 0; i < cloudLaser->points.size(); ++i)
	{
		cloudLaser->points[i].x = readings->ranges[i] * cos(readings->angle_min + i * readings->angle_increment);
		cloudLaser->points[i].y = readings->ranges[i] * sin(readings->angle_min + i * readings->angle_increment);
	}
    pcl::transformPointCloud(*cloudLaser, *cloud, transformEigen);
        
}

void JustinaTools::laserScanToPclCylindrical(const sensor_msgs::LaserScan::ConstPtr& readings, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
	//tf::StampedTransform transformTf;
    //tf_listener->lookupTransform("map","laser_link", ros::Time(0), transformTf);
	//Eigen::Affine3d transformEigen;
    //tf::transformTFToEigen(transformTf, transformEigen);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloudLaser(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->width = readings->ranges.size();
	cloud->height = 1;
	cloud->points.resize(cloud->width * cloud->height);
	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		cloud->points[i].x = readings->ranges[i];
		cloud->points[i].y =readings->angle_min + i * readings->angle_increment;
	}
	//pcl::transformPointCloud(*cloudLaser, *cloud, transformEigen);
}

void JustinaTools::PointCloud2Msg_ToCvMat(sensor_msgs::PointCloud2& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
	//std::cout << "ObjectDetectorNode.-> Transforming from PointCloud2 ros message to cv::Mat type" << std::endl;
	//std::cout << "ObjectDetectorNode.-> Width= " << pc_msg.width << "  height= " << pc_msg.height << std::endl;
	pcl::PointCloud<pcl::PointXYZRGBA> pc_pcl;
	pcl::fromROSMsg(pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

	if(!pc_pcl.isOrganized())
	{
		std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
		return;
	}
	//std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width << "  pcl_h= " << pc_pcl.height << std::endl;
	bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
	pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
	pcl::PointXYZRGBA p_ = pc_pcl.at(320, 240);
	//std::cout<<"ObjectDetectorNode: Middle point: "<<p_.x << " " <<p_.y <<" "<<p_.z <<" "<< p_.r<<" "<<p_.g<<" "<< p_.b << std::endl;
	for (int h=0; h<bgr_dest.rows; h++)
		for (int w=0; w<bgr_dest.cols; w++)
		{
			pcl::PointXYZRGBA p = pc_pcl.at(w, h);

			bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
			bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
			bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
			pc_dest.at<cv::Vec3f>(h,w)[0] = isnan(p.x) ? 0.0 : p.x;
			pc_dest.at<cv::Vec3f>(h,w)[1] = isnan(p.y) ? 0.0 : p.y;
			pc_dest.at<cv::Vec3f>(h,w)[2] = isnan(p.z) ? 0.0 : p.z;
		}

}


void JustinaTools::PointCloud2Msg_ToCvMat(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, cv::Mat& bgr_dest, cv::Mat& pc_dest)
{
	pcl::PointCloud<pcl::PointXYZRGB> pc_pcl;
	pcl::fromROSMsg(*pc_msg, pc_pcl);  //Transform from PointCloud2 msg to pointCloud (from pcl) type

	if(!pc_pcl.isOrganized())
	{
		std::cout << "ObjectDetectorNode.->Point cloud is not organized!! O M G!!!!" << std::endl;
		return;
	}
	//std::cout << "ObjectDetectorNode.-> Pcl_w= " << pc_pcl.width << "  pcl_h= " << pc_pcl.height << std::endl;
	bgr_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_8UC3);
	pc_dest = cv::Mat::zeros(pc_pcl.height, pc_pcl.width, CV_32FC3);
	
	//pcl::PointXYZRGB p_ = pc_pcl.at(320, 240);
	//std::cout<<"ObjectDetectorNode: Middle point: "<<p_.x << " " <<p_.y <<" "<<p_.z <<" "<< p_.r<<" "<<p_.g<<" "<< p_.b << std::endl;
	for (int h=0; h<bgr_dest.rows; h++)
		for (int w=0; w<bgr_dest.cols; w++)
		{
			pcl::PointXYZRGB p = pc_pcl.at(w,h);
			bgr_dest.data[h*bgr_dest.step + w*3] = (unsigned char)p.b;
			bgr_dest.data[h*bgr_dest.step + w*3 + 1] = (unsigned char)p.g;
			bgr_dest.data[h*bgr_dest.step + w*3 + 2] = (unsigned char)p.r;
			pc_dest.at<cv::Vec3f>(h,w)[0] = isnan(p.x) ? 0.0 : p.x;
			pc_dest.at<cv::Vec3f>(h,w)[1] = isnan(p.y) ? 0.0 : p.y;
			pc_dest.at<cv::Vec3f>(h,w)[2] = isnan(p.z) ? 0.0 : p.z;
		}

}

bool JustinaTools::transformPoint(std::string src_frame, float inX, float inY, float inZ, std::string dest_frame, float& outX, float& outY, float& outZ)
{
    tf::StampedTransform transformTf;
    tf_listener->lookupTransform(dest_frame,src_frame, ros::Time(0), transformTf);
    tf::Vector3 v(inX, inY, inZ);
    v = transformTf * v;
    outX = v.x();
    outY = v.y();
    outZ = v.z();
    return true;
}

bool JustinaTools::transformPose(std::string src_frame, float inX, float inY, float inZ, float inRoll, float inPitch, float inYaw,
                                 std::string dest_frame,float& outX,float& outY,float& outZ, float& outRoll, float& outPitch, float& outYaw)
{
    std::cout << "Trans: " << inX << " " << inY << " " << inZ << " " << inRoll << " " << inPitch << " " << inYaw << std::endl;
    tf::StampedTransform ht;
    tf_listener->lookupTransform(dest_frame,src_frame, ros::Time(0), ht);
    tf::Quaternion q;
    q.setRPY(inRoll, inPitch, inYaw);
    tf::Vector3 p(inX, inY, inZ);
    p = ht * p;
    q = ht * q;
    double dRoll, dPitch, dYaw;
    tf::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);
    outX = p.x();
    outY = p.y();
    outZ = p.z();
    outRoll = (float)dRoll;
    outPitch = (float)dPitch;
    outYaw = (float)dYaw;
    
    std::cout << "Trans: " << outX << " " << outY << " " << outZ << " " << outRoll << " " << outPitch << " " << outYaw << std::endl;
    return true;
}

bool JustinaTools::transformPose(std::string src_frame, std::vector<float>& xyz_rpy_in, std::string dest_frame, std::vector<float>& xyz_rpy_out)
{
    if(xyz_rpy_in.size() < 6)
    {
        std::cout << "JustinaTools.->Cannot transform pose. vector<float> must have 6 values: xyz and roll pitch yaw" << std::endl;
        return false;
    }
    if(xyz_rpy_out.size() < 6)
    {
        xyz_rpy_out.clear();
        for(int i=0; i< 6; i++)
            xyz_rpy_out.push_back(0);
    }
    return transformPose(src_frame, xyz_rpy_in[0], xyz_rpy_in[1], xyz_rpy_in[2], xyz_rpy_in[3], xyz_rpy_in[4], xyz_rpy_in[5],
                         dest_frame, xyz_rpy_out[0], xyz_rpy_out[1], xyz_rpy_out[2], xyz_rpy_out[3], xyz_rpy_out[4], xyz_rpy_out[5]);

}

void JustinaTools::pdfImageExport(std::string testName, std::string output){
	std::string path="/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/pdfScript.sh";
        std::stringstream temp;
	temp << path;
	temp << " ";
	temp << testName;
	temp << " ";
	temp << output;
        std::string final = temp.str();
        //std::cout << "ss created in " << final << std::endl;
        system(final.c_str());
}

void JustinaTools::pdfStart(std::string theFile){
	std::string path="/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/createPdfScript.sh";
        std::stringstream temp;
	temp << path;
	temp << " ";
	temp << theFile;
        std::string final = temp.str();
        //std::cout << "ss created in " << final << std::endl;
        system(final.c_str());
}

void JustinaTools::pdfAppend(std::string fileAp,std::string lineAp){
        std::string path="/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/appendPdfScript.sh";
        std::stringstream temp;
	temp << path;
	temp << " ";
	temp << fileAp;
	temp << " ";
	temp << lineAp;
        std::string final = temp.str();
        //std::cout << "ss created in " << final << std::endl;
        system(final.c_str());
}

void JustinaTools::pdfStop(std::string theFile){
        std::string path="/home/$USER/JUSTINA/catkin_ws/src/vision/vision_export/stopPdfScript.sh";
        std::stringstream temp;
	temp << path;
	temp << " ";
	temp << theFile;
        std::string final = temp.str();
        //std::cout << "ss created in " << final << std::endl;
        system(final.c_str());
}
