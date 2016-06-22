
#ifndef FOREST_H_
#define FOREST_H_

#include <fstream>
#include <sys/stat.h>
#include <iostream>
#include <highgui.h>
#include <cv.h>
#include <cvaux.h>
#include <ml.h>
#include <stdio.h>
#include <string.h>

using namespace cv;
using namespace std;

#define VAGUE_LEARN false
#define TREE_DEPTH 18
#define N_TREES 8

#include <object_tracker/swri_tree.h>

  class forest{

    struct fTree{
      int features[TREE_DEPTH];
      swriTree* dTree;
      int treeVote;
    };

    fTree trees[N_TREES];
    cv::Mat h;
  public:

    forest();

    ~forest();


    int classify(cv::Mat& sequence);

    void addToTrees(cv::Mat& sequence, bool protect = false);

    void removeFromTrees(cv::Mat& sequence, bool protect = false);

    void subFeature(cv::Mat& sequence, int i, cv::Mat& h);

    void randomFeatures(int poolSize);

  }; 

#endif // FOREST_H_
