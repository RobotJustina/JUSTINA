
#ifndef FEATURES_H_
#define FEATURES_H_

#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include <highgui.h>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <stdio.h>
#include <string.h>

  /**
   * Calculate and store features for online learning.
   */
  class features{

  public:
    cv::Mat featurePool1;
    cv::Mat horzMap;
    cv::Mat vertMap;
    cv::Mat diagMap;

    features();

    ~features();

    /**
     * Compute the feature pools
     * input gray The input image
     * input roi The input roi
     */
    void computePool1(cv::Mat gray, CvRect roi);

    /**
     * Compute haar sign features
     * input gray The input image
     * input roi The input roi
     */
    void haarSignFeatures(cv::Mat gray, CvRect roi);

    /**
     * Compute the color features
     * input clr The input image
     * input roi The input roi
     */
    void regionColorFeatures(cv::Mat clr, CvRect roi);

    /**
     * Compute maps for pool features
     */
    void computeMaps(cv::Mat gray);

    /**
     * Compute the feature pools
     * input i index of map to find features for
     * input j index of map to find features for
     */
    void writePool1FromMaps(int i, int j);
  private:
  };  // class features

#endif // FEATURES_H_
