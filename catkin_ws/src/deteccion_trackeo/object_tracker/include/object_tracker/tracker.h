

#ifndef PARTICLE_TRACK_H_
#define PARTICLE_TRACK_H_

#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include <highgui.h>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include "ros/console.h"
// Package files
#include <object_tracker/particle_filter.h>
#include <object_tracker/features.h>
#include <object_tracker/bosqt.h>


using namespace cv;
using namespace std;

  /**
   * Track object in disparity space with particle filter.
   */

  class ParticleTrack : public ParticleFilter
  {

    forest forest1;
    features features1;

    // Stereo setup stuff
    cv::Mat xz_map_;
    double xz_map_res_, xz_map_offset_;
    double angle_;
    cv::Mat R;
    double Q_32, Q_03, Q_13, Q_23, Q_33;

    // Object properties
    double h;
    double ar_;

    struct posic_dt
    {
    	double dt;
    	cv::Point2f position;
    };
    std::vector<posic_dt> position_hist;
  public:
    double size_;

    cv::Point objectLocation;
    CvRect objectROI;
    CvRect searchROI;
    cv::Mat hist_;
    uint64_t uid;
    Point3f locationxyz;
    cv::Point2f centerROI;
   
    bool debug_mode_;  // if true, console output and visualization is active


    ParticleTrack();

    /**
     * Initialize a tracker
     * input input The input image
     * input inROI the tracking box
     */
     void initialize(cv::Mat& input, cv::Mat& disp, CvRect inROI, std::string filterDir,
        cv::Mat& Q, double angle);
    //void initialize(cv::Mat& input, cv::Mat& disp, CvRect inROI, cv::Mat& Eigvec, cv::Mat& Eigval,
    //    cv::Mat& Q, double angle); 

    void revive(cv::Mat& input, cv::Mat& xz, CvRect inROI);

    /**
     * Update the tracker state
     * input The input image (next image in sequence)
     */
    int update_state(cv::Mat& input, cv::Mat& xz, double dt);

    /**
     * Draw the tracker box
     * input image The image to draw on
     * input clr The color of the box
     */
    void draw_box(cv::Mat& image, CvScalar clr,cv::Mat& disp);

    /**
     * Score an object against the tracker
     * input image The image to draw on
     * input disp The disparity map
     * input roi The roi of the candidate object
     * return a score for the object
     */
    double score(cv::Mat& image, cv::Mat& disp, cv::Rect roi);

    /**
     * compute velocity of tracked object from position history
     * return object x and z velocity
     */
    cv::Point2f compute_velocity();

    /**
     * compute speed of tracked object from position history
     * return magnitude of velocity
     */
    double compute_speed();

    /**
     * return a bounding box in the image from 3D coordinates
     * input xz X and Z coordinates in 3D
     * return Rect containing bounding box coordinates in the image
     */
    cv::Rect xz_to_box(cv::Point2f xz);

    /**
     * return the estimated y-location of the object
     * value ret estimated y-location of object
     */
    double get_height(){
      return h;
    }

    /**
     * return the person box size (z * h)
     * val return tracker box size
     */
    double get_box_size(){
      return size_;
    }

    /**
     * return the person box aspect ratio
     * value return tracker box size aspect ratio
     */
    double get_ar(){
      return ar_;
    }

    
  private:
    void raw_detect(cv::Mat& input);
    void grow(cv::Mat& input);
    void prune();
    cv::Point2f find_location(cv::Mat& disp, cv::Rect roi);
    void make_xz_map(cv::Mat& disp, cv::Rect* roi = NULL);
    void make_xz_map_weighted(cv::Mat& image, cv::Mat& disp, cv::Rect* roi);
    void make_object_hist(cv::Mat& image, cv::Mat& disp, cv::Point maxLoc);
  };



#endif // PARTICLE_TRACK_H_

