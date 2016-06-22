
#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

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

using namespace cv;
using namespace std;

  struct Particle{
    Mat x;
    double weight;
    double prevWeight;
  };


  class ParticleFilter{
    cv::RNG rng;
    double neff;
  public:
    double age;
    int N; 
    Mat A; 
    Mat B; 
    bool initialized;
    bool is_dead_;
    Particle current_map_;
    vector<Particle> particle_model; 
    bool console_output_;


    ParticleFilter();


    void initialize(Particle initial_state);


    void load_model(const char* perceptionPath);


    void updateState(cv::Mat& input, double dt);


    cv::Mat calculate_covariance();

  private:
    void resample();
    void drift_diffuse_sample(double dt); 
    void observation_density_reweight(cv::Mat& input); 
    virtual double likelihood(cv::Mat& input, Particle states); 

  };

#endif // PARTICLE_FILTER_H_
