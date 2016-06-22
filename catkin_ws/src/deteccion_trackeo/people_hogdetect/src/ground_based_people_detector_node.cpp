#include <ros/ros.h>
#include <ros/console.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl_conversions/pcl_conversions.h>

//Publish Messages
#include "roi_msgs/RoiRect.h"
#include "roi_msgs/Rois.h"
#include "std_msgs/String.h"

using namespace roi_msgs;
using namespace sensor_msgs;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool new_cloud_available_flag = false;
PointCloudT::Ptr cloud(new PointCloudT);

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

enum { COLS = 640, ROWS = 480 };

void cloud_cb(const PointCloudT::ConstPtr& callback_cloud)
{
  *cloud = *callback_cloud;
  new_cloud_available_flag = true;
}

struct callback_args{
  // structure used to pass arguments to the callback function
  PointCloudT::Ptr clicked_points_3d;
  pcl::visualization::PCLVisualizer* viewerPtr;
};

void
pp_callback (const pcl::visualization::PointPickingEvent& event, void* args)
{
  struct callback_args* data = (struct callback_args *)args;
  if (event.getPointIndex () == -1)
    return;
  PointT current_point;
  event.getPoint(current_point.x, current_point.y, current_point.z);
  data->clicked_points_3d->points.push_back(current_point);
  // Draw clicked points in red:
  pcl::visualization::PointCloudColorHandlerCustom<PointT> red (data->clicked_points_3d, 255, 0, 0);
  data->viewerPtr->removePointCloud("clicked_points");
  data->viewerPtr->addPointCloud(data->clicked_points_3d, red, "clicked_points");
  data->viewerPtr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "clicked_points");
  std::cout << current_point.x << " " << current_point.y << " " << current_point.z << std::endl;
}

int main (int argc, char** argv)
{
  ros::init(argc, argv, "ground_based_people_detector");
  ros::NodeHandle nh("~");

  // Read some parameters from launch file:
  std::string svm_filename;
  nh.param("classifier_file", svm_filename, std::string("./"));
  double min_confidence;
  nh.param("HogSvmThreshold", min_confidence, -1.5);
  double min_height;
  nh.param("minimum_person_height", min_height, 1.3);
  double max_height;
  nh.param("maximum_person_height", max_height, 2.3);
  int sampling_factor;
  nh.param("sampling_factor", sampling_factor, 1);
  std::string pointcloud_topic;
  nh.param("pointcloud_topic", pointcloud_topic, std::string("/camera/depth_registered/points"));
  double rate_value;
  nh.param("rate", rate_value, 30.0);

  // Fixed parameters:
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics

  // Subscribers:
  ros::Subscriber sub = nh.subscribe(pointcloud_topic, 1, cloud_cb);

  // Publishers:
  ros::Publisher pub_rois_;
  pub_rois_= nh.advertise<Rois>("GroundBasedPeopleDetectorOutputRois",3);

  Rois output_rois_;

  ros::Rate rate(rate_value);
  while(ros::ok() && !new_cloud_available_flag)
  {
    ros::spinOnce();
    rate.sleep();
  }

  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  cb_args.clicked_points_3d = clicked_points_3d;
  cb_args.viewerPtr = &viewer;
  viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);
  std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;

  viewer.spin();
  std::cout << "done." << std::endl;

  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);
  std::vector<int> clicked_points_indices;
  for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
    clicked_points_indices.push_back(i);
  pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
  model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
  ROS_ERROR("Ground plane coefficients: %f, %f, %f, %f.", ground_coeffs(0), ground_coeffs(1), ground_coeffs(2), ground_coeffs(3));

  
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;
  std::cout << svm_filename << std::endl;
  person_classifier.loadSVMFromFile(svm_filename);   
  
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;   
  people_detector.setVoxelSize(voxel_size);                        
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            
  people_detector.setClassifier(person_classifier);                
  people_detector.setHeightLimits(min_height, max_height);         
  people_detector.setSamplingFactor(sampling_factor);              

  // Loop:
  while(ros::ok())
  {
    if (new_cloud_available_flag)
    {
      new_cloud_available_flag = false;

      
      std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

      
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs);                    
      people_detector.compute(clusters);                           

      ground_coeffs = people_detector.getGround();                 

      output_rois_.rois.clear();
      output_rois_.header = cloud_header;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
        if(it->getPersonConfidence() > min_confidence)             
        {
          
          Eigen::Vector3f centroid = rgb_intrinsics_matrix * (it->getTCenter());
          centroid /= centroid(2);
          
          Eigen::Vector3f top = rgb_intrinsics_matrix * (it->getTTop());
          top /= top(2);
          
          RoiRect R;
          R.height = centroid(1) - top(1);
          R.width  = R.height * 2 / 3.0;
          R.x      = std::max(0, int(centroid(0) - R.width / 2.0));
          R.y      = std::max(0, int(top(1)));
          R.height = std::min(int(ROWS - R.y), int(R.height));
          R.width = std::min(int(COLS - R.x), int(R.width));
          R.label  = 1;
          R.confidence  = it->getPersonConfidence();
          output_rois_.rois.push_back(R);
        }
      }
      pub_rois_.publish(output_rois_);  
    }
    
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
