#include <ros/ros.h>
#include "ros/console.h"
#include <ros/package.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/point_types_conversion.h>

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

Eigen::VectorXf get_object_points (PointCloudT::Ptr cloud_blob, pcl::visualization::PCLVisualizer& viewerPtr)
{
  // PHASE 1: DATA CLEANUP
  ////////////////////////////////////////////////////////

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Filter out NaNs from data (this is necessary now) ...
  pcl::PassThrough<pcl::PointXYZRGB> nan_remover;
  nan_remover.setInputCloud(cloud_blob);
  nan_remover.setFilterFieldName("z");
  nan_remover.setFilterLimits(0.0, 10.0);
  nan_remover.filter(*cloud_filtered);

  // Filter out statistical outliers
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statistical_filter;
  statistical_filter.setInputCloud(cloud_filtered);
  statistical_filter.setMeanK(50);
  statistical_filter.setStddevMulThresh(1.0);
  statistical_filter.filter(*cloud_p);

  // Downsample to 1cm Voxel Grid
  pcl::VoxelGrid<PointT> downsampler;
  downsampler.setInputCloud(cloud_p);
  downsampler.setLeafSize(0.01f, 0.01f, 0.01f);
  downsampler.filter(*cloud_f);
  //ROS_INFO("PointCloud after filtering: %d data points.", cloud.width * cloud.height);

  // PHASE 2: FIND THE GROUND PLANE
  /////////////////////////////////////////////////////////

  // Step 3: Find the ground plane using RANSAC
  pcl::ModelCoefficients ground_coefficients;
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices ());
  pcl::SACSegmentation<pcl::PointXYZRGB> ground_finder;
  ground_finder.setOptimizeCoefficients(true);
  ground_finder.setModelType(pcl::SACMODEL_PLANE);
  ground_finder.setMethodType(pcl::SAC_RANSAC);
  ground_finder.setDistanceThreshold(0.015);
  ground_finder.setInputCloud(cloud_f);
  ground_finder.segment(*ground_indices, ground_coefficients);

  PointCloudT::Ptr ground_points;
  pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
  extractor.setInputCloud(cloud);
  extractor.setIndices(ground_indices);
  extractor.filter(*ground_points);

  Eigen::VectorXf coeffs;
  coeffs.resize(4);
  for (size_t i = 0; i < coeffs.size (); ++i)
    coeffs(i)=ground_coefficients.values[i];

  viewerPtr.removeAllPointClouds();
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(ground_points);
  viewerPtr.addPointCloud<PointT> (ground_points, rgb, "Model cloud");
  //viewerPtr.spinOnce(2000);
  return coeffs;
}

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
  //nh.getParam("svm_file", svm_filename);
  // Fixed parameters:
  float voxel_size = 0.06;
  Eigen::Matrix3f rgb_intrinsics_matrix;
  rgb_intrinsics_matrix << 525, 0.0, 319.5, 0.0, 525, 239.5, 0.0, 0.0, 1.0; // Kinect RGB camera intrinsics
  bool automatic = false;  
  bool clic= false;

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

  // Display pointcloud:
  pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
  viewer.addPointCloud<PointT> (cloud, rgb, "input_cloud");
  viewer.setCameraPosition(0,0,-2,0,-1,0,0);
  ROS_INFO_STREAM("File SVM read " << svm_filename );


//*******************************************************//
  // Ground plane estimation:
  struct callback_args cb_args;
  PointCloudT::Ptr clicked_points_3d (new PointCloudT);
  Eigen::VectorXf ground_coeffs;
  ground_coeffs.resize(4);

  if (!automatic){
    cb_args.clicked_points_3d = clicked_points_3d;
    cb_args.viewerPtr = &viewer;
    viewer.registerPointPickingCallback (pp_callback, (void*)&cb_args);

    std::cout << "Shift+click on three floor points, then press 'Q'..." << std::endl;
    clic=true;
    viewer.spin();
    // Spin until 'Q' is pressed:
  }
  else{
     ground_coeffs=get_object_points(cloud, viewer);
     viewer.spinOnce(3000);
  }


  std::cout << "done." << std::endl;

    if (clic){
        std::vector<int> clicked_points_indices;
        for (unsigned int i = 0; i < clicked_points_3d->points.size(); i++)
            clicked_points_indices.push_back(i);

        pcl::SampleConsensusModelPlane<PointT> model_plane(clicked_points_3d);
        model_plane.computeModelCoefficients(clicked_points_indices,ground_coeffs);
   }
//*******************************************************//

  ROS_ERROR("Ground plane coefficients: %f, %f, %f, %f.", ground_coeffs(0), ground_coeffs(1), ground_coeffs(2), ground_coeffs(3));


  // Create classifier for people detection:
  pcl::people::PersonClassifier<pcl::RGB> person_classifier;

  person_classifier.loadSVMFromFile(svm_filename);   // load trained SVM
  
  // People detection app initialization:
  pcl::people::GroundBasedPeopleDetectionApp<PointT> people_detector;    // people detection object
  people_detector.setVoxelSize(voxel_size);                        // set the voxel size
  people_detector.setIntrinsics(rgb_intrinsics_matrix);            // set RGB camera intrinsic parameters
  people_detector.setClassifier(person_classifier);                // set person classifier
  people_detector.setHeightLimits(min_height, max_height);         // set person classifier
  people_detector.setSamplingFactor(sampling_factor);              // set sampling factor

  // Main loop:
  while(ros::ok())
  {
    if (new_cloud_available_flag)
    {
      new_cloud_available_flag = false;

      // Convert PCL cloud header to ROS header:
      std_msgs::Header cloud_header = pcl_conversions::fromPCL(cloud->header);

      // Perform people detection on the new cloud:
      std::vector<pcl::people::PersonCluster<PointT> > finalFilteredClusters;
      std::vector<pcl::people::PersonCluster<PointT> > clustersWithinConfidenceBounds;
      std::vector<pcl::people::PersonCluster<PointT> > clusters;   // vector containing persons clusters
      people_detector.setInputCloud(cloud);
      people_detector.setGround(ground_coeffs);                    // set floor coefficients
      people_detector.compute(clusters);                           // perform people detection

      ground_coeffs = people_detector.getGround();                 // get updated floor coefficients

      // Write ROIs message and publish it:
      output_rois_.rois.clear();
      output_rois_.header = cloud_header;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clusters.begin(); it != clusters.end(); ++it)
      {
    	  if(it->getPersonConfidence() > min_confidence)             // keep only people with confidence above a threshold
    	  {
            clustersWithinConfidenceBounds.push_back(*it);
    	  }
      }
      const double MIN_DISTANCE = 0.30;
      for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it = clustersWithinConfidenceBounds.begin(); it != clustersWithinConfidenceBounds.end(); ++it) {
        Eigen::Vector3f bottom = it->getTBottom();
        bottom.y() = 0; // ignore height
        bool overlapsWithOtherCluster = false;

        for(std::vector<pcl::people::PersonCluster<PointT> >::iterator it2 = finalFilteredClusters.begin(); it2 != finalFilteredClusters.end(); ++it2) {
            Eigen::Vector3f otherBottom = it2->getTBottom();
            otherBottom.y() = 0; // ignore height

            if((bottom - otherBottom).norm() <= MIN_DISTANCE) {
                overlapsWithOtherCluster = true;
                break;
            }
        }

        if(!overlapsWithOtherCluster) finalFilteredClusters.push_back(*it);
      }

       for (std::vector<pcl::people::PersonCluster<PointT> >::iterator it = finalFilteredClusters.begin(); it != finalFilteredClusters.end(); ++it)
        {

          // theoretical person centroid:
          Eigen::Vector3f centroid = rgb_intrinsics_matrix * (it->getTCenter());
          centroid /= centroid(2);
          
	  // theoretical person top point:
          Eigen::Vector3f top = rgb_intrinsics_matrix * (it->getTTop());
          top /= top(2);

          // Define RoiRect and make sure it is not out of the image:
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
      //ROS_INFO("Number of persons: %d ",output_rois_.rois.size());  
      pub_rois_.publish(output_rois_);  // publish message
    }

    // Execute callbacks:
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}

