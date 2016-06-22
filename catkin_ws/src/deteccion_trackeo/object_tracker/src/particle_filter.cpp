

#include <object_tracker/particle_filter.h>

    ParticleFilter::ParticleFilter():
        rng(0xFFFFFFFF)
    {
      initialized = false;
      particle_model.clear();
      N = 500;
      neff = N;
      is_dead_ = false;
      console_output_ = false;
    }

    void ParticleFilter::initialize(Particle initial_state)
    {
      is_dead_ = false;

      
      current_map_ = initial_state;

      
      particle_model.clear();
      for (int i = 0; i < N; i++)
      {
        Particle temp;
        temp.x = initial_state.x.clone();
        temp.weight = rng.uniform(0.0,1.0);;
        particle_model.push_back(temp);
      }
      
      double sum = 0.0;
      for(int i = 0; i < N; i++) sum += particle_model[i].weight;
      for(int i = 0; i < N; i++) particle_model[i].weight /= sum;
      initialized = true;
    }

    void ParticleFilter::load_model(const char* perceptionPath)
    {
      cv::FileStorage fs(std::string(std::string(perceptionPath)
      + std::string("/filter.xml")).c_str(),
          cv::FileStorage::READ);
      fs["eigenvectors"] >> A;
      fs["eigenvalues"] >> B;
      fs.release();
    }

    void ParticleFilter::updateState(cv::Mat& input, double dt)
    {
      // resample();
      drift_diffuse_sample(dt);
      observation_density_reweight(input);
      if(!is_dead_)
      {
        double max = 0.0;
        for(int i = 0; i < N; i++)
        {
          if(particle_model[i].weight > max)
          {
            max = particle_model[i].weight;
            current_map_.weight = particle_model[i].weight;
            current_map_.x = particle_model[i].x.clone();
          }
        }
      }
    }

    cv::Mat ParticleFilter::calculate_covariance()
    {
      // Put all points into a single matrix
      cv::Mat allPts(N, current_map_.x.rows, CV_64F); // N-samples, state rows columns
      for(int i = 0; i < N; i++)
        for(int j = 0; j < allPts.cols; j++)
          allPts.at<double>(i,j) = particle_model[i].x.at<float>(j,0);

      // calculate covariance
      cv::Mat covar;
      cv::Mat avg;
      cv::calcCovarMatrix(allPts,covar,avg,cv::COVAR_NORMAL + cv::COVAR_ROWS);
      return covar;
    }

    void ParticleFilter::resample()
    {
      for (int j = 0; j < N; j++)
      {
        
        double rn = rng.uniform(0.0,1.0);
        
        int i;
        for(i = 0; i < N; i++)
        {
          rn = rn - particle_model[i].weight;
          if(rn <= 0.0) break;
        }

        
        Particle temp;
        temp.x = particle_model[i].x.clone();
        temp.weight = particle_model[i].weight;
        particle_model.push_back(temp);
      }
      
      for(int j = 0; j < N; j++) particle_model.erase(particle_model.begin());
    }

    void ParticleFilter::drift_diffuse_sample(double dt)
    {
      if((!A.data)||(!B.data))
        {
        ROS_ERROR("Drift and Diffusion models do not"
            "exist.");
        return;
        }


      for(int j = 0; j < N; j++)
      {
        
        for(int i = 0; i < B.rows; i++)
          particle_model[j].x += (rng.gaussian(B.at<float>(i,0)*dt)*A.row(i)).t();
      }
    }

    void ParticleFilter::observation_density_reweight(cv::Mat& input)
    {
      
      for(int i = 0; i < N; i++)
      {
        particle_model[i].prevWeight = particle_model[i].weight;
        particle_model[i].weight *= likelihood(input, particle_model[i]);
      }

      
      double sum = 0.0;
      for(int i = 0; i < N; i++) sum += particle_model[i].weight;
      double normFactor = 1.0/sum;
      if(sum == 0.0) normFactor = 0.0;
      for(int i = 0; i < N; i++) particle_model[i].weight *= normFactor;

      
      sum = 0.0;
      for(int i = 0; i < N; i++) sum += pow(particle_model[i].weight, 2);
      neff = 1.0/sum;
      if(sum == 0) neff = 0.0;
      if (console_output_)
        ROS_INFO("Neff = %f", neff);

      if(neff < N * 0.04)
      {
        is_dead_ = true;
      }
      else if(neff < N * 0.75)
      {
        resample();
        observation_density_reweight(input);
      }

    }

    
    double ParticleFilter::likelihood(cv::Mat& input, Particle states)
    {
      

      int i = states.x.at<float>(1,0);
      int j = states.x.at<float>(0,0);
      if((i < 0)
          ||(j < 0)
          ||(i >= input.rows)
          ||(j >= input.cols))
        return 0.0;
      return input.at<float>(i,j);
    }



