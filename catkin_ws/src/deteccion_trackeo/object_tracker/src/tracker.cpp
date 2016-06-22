#include <object_tracker/tracker.h>

ParticleTrack::ParticleTrack()
{
  Q_32 = 1.0 / (0.075);
  Q_03 = -319.5;
  Q_13 = -239.5;
  Q_23 = 525.0;
  Q_33 = 0;
}

void ParticleTrack::initialize(cv::Mat& input, cv::Mat& disp, CvRect inROI,
    std::string filterDir, cv::Mat& Q, double angle)
{
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=input.cols || inROI.y+inROI.height>=input.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=disp.cols || inROI.y+inROI.height>=disp.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.height<=0 || inROI.width<=0){
    ROS_ERROR("inROI size too small %d %d",inROI.height,inROI.width);
  }
  age = 0.0;
  position_hist.clear();

  
  Q_32 = Q.at<double>(3,2);
  Q_03 = Q.at<double>(0,3);
  Q_13 = Q.at<double>(1,3);
  Q_23 = Q.at<double>(2,3);
  Q_33 = Q.at<double>(3,3);

  
  angle_ = angle;
  R = cv::Mat::zeros(3, 3, CV_64F);
  R.at<double>(0, 0) = 1;
  R.at<double>(1, 1) = std::cos(angle_ * CV_PI / 180.0);
  R.at<double>(1, 2) = -std::sin(angle_ * CV_PI / 180.0);
  R.at<double>(2, 1) = std::sin(angle_ * CV_PI / 180.0);
  R.at<double>(2, 2) = std::cos(angle_ * CV_PI / 180.0);

  
  cv::Point2f xz = find_location(disp, inROI);
  cv::Point xz_loc;
  cv::minMaxLoc(xz_map_, NULL, NULL, NULL, &xz_loc);
  make_object_hist(input, disp, xz_loc);

  
  size_ = inROI.height * xz.y; 
  ar_ = (double) inROI.width / (double) inROI.height;

  
  load_model(filterDir.c_str());
  Particle initialState;
  initialState.weight = 1.0;
  initialState.prevWeight = 1.0;
  initialState.x = cv::Mat::zeros(2, 1, CV_32F);
  initialState.x.at<float>(0, 0) = xz.x;
  initialState.x.at<float>(1, 0) = xz.y;
  ParticleFilter::initialize(initialState);

  
  forest1.randomFeatures(27 + 48);
  features1.computePool1(input, inROI);
  forest1.addToTrees(features1.featurePool1);
}

void ParticleTrack::revive(cv::Mat& input, cv::Mat& disp, CvRect inROI)
{
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=input.cols || inROI.y+inROI.height>=input.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.x<0 || inROI.y<0 ||inROI.x+inROI.width >=disp.cols || inROI.y+inROI.height>=disp.rows){
    ROS_ERROR("inROI out of bounds %d %d %d %d",inROI.x,inROI.y,inROI.height,inROI.width);
  }
  if(inROI.height<=0 || inROI.width<=0){
    ROS_ERROR("inROI size too small %d %d",inROI.height,inROI.width);
  }

  if (debug_mode_)
    ROS_INFO("Reviving tracker %d", (int) uid);
  age = 0.0;
  position_hist.clear();
  
  cv::Point2f xz = find_location(disp, inROI);
  cv::Point xz_loc;
  cv::minMaxLoc(xz_map_, NULL, NULL, NULL, &xz_loc);
  make_object_hist(input, disp, xz_loc);

  
  Particle initialState;
  initialState.weight = 1.0;
  initialState.prevWeight = 1.0;
  initialState.x = cv::Mat::zeros(2, 1, CV_32F);
  initialState.x.at<float>(0, 0) = xz.x;
  initialState.x.at<float>(1, 0) = xz.y;
  ParticleFilter::initialize(initialState);
}

int ParticleTrack::update_state(cv::Mat& input, cv::Mat& disp, double dt)
{
  age += dt;
  if (is_dead_)
    return is_dead_;
  
  make_xz_map_weighted(input, disp, NULL);

  
  double maxVal;
  cv::minMaxLoc(xz_map_, NULL, &maxVal);
  maxVal = 2500.0;
  xz_map_ = xz_map_ * (1.0 / maxVal);



  
  updateState(input, dt);
  if (is_dead_)
  {
    return is_dead_;
  }

  
  grow(input);
  
  posic_dt temp;
  temp.dt = dt;
  temp.position.x = current_map_.x.at<float>(0, 0);
  temp.position.y = current_map_.x.at<float>(1, 0);
  position_hist.push_back(temp);

  return is_dead_;
}

cv::Point2f ParticleTrack::compute_velocity()
{
  double dt_sum = 0.0;
  cv::Point2f v;
  v.x = 0;
  v.y = 0;
  if (position_hist.size() < 2)
    return v;
  int idx = position_hist.size() - 1;
  while (dt_sum < .5) 
  {
    dt_sum += position_hist[idx].dt;
    idx--;
    if (idx == 0)
      break;
  }
  v.x = position_hist.back().position.x - position_hist[idx].position.x;
  v.y = position_hist.back().position.y - position_hist[idx].position.y;
  v.x *= 1.0 / dt_sum;
  v.y *= 1.0 / dt_sum;
  return v;
}

double ParticleTrack::compute_speed()
{
  cv::Point2f vel = compute_velocity();
  return std::sqrt(std::pow(vel.x, 2) + std::pow(vel.y, 2));
}

void ParticleTrack::grow(cv::Mat& input)
{
  cv::Point2f loc;
  loc.x = current_map_.x.at<float>(0, 0);
  loc.y = current_map_.x.at<float>(1, 0);
  objectROI = xz_to_box(loc);
  if ((objectROI.x > 0) && (objectROI.y > 0)
      && (objectROI.x + objectROI.width < input.cols)
      && (objectROI.y + objectROI.height < input.rows)
      && objectROI.width>0 && objectROI.height>0 )
  {
    features1.computePool1(input, objectROI);
    forest1.addToTrees(features1.featurePool1);
  }
}

void ParticleTrack::draw_box(cv::Mat& image, CvScalar clr, cv::Mat& disp)
{
  if (!is_dead_)
  {
    cv::Point2f loc;
    loc.x = current_map_.x.at<float>(0, 0);
    loc.y = current_map_.x.at<float>(1, 0);
    objectROI = xz_to_box(loc);
    cv::rectangle(image, objectROI, clr, 2);
    if ((objectROI.x > 0) && (objectROI.y > 0)
        && (objectROI.x + objectROI.width < image.cols)
        && (objectROI.y + objectROI.height < image.rows)
  && objectROI.width>0 && objectROI.height>0 )
    {
      char buffer[50];
      sprintf(buffer, "%d", (int)uid);
      cv::putText(image, std::string(buffer), cv::Point(objectROI.x, objectROI.y), FONT_HERSHEY_PLAIN, 2.0, cv::Scalar(255));
      //sprintf(buffer, "%0.2fm/s", compute_speed());
      //cv::putText(image, std::string(buffer), cv::Point(objectROI.x, objectROI.y + objectROI.height), FONT_HERSHEY_PLAIN, 1., cv::Scalar(0, 255, 0));
    }
    
    centerROI.x = objectROI.x + objectROI.width/2;
    centerROI.y = objectROI.y + objectROI.height/2;
    locationxyz.x = centerROI.x + Q_03;
    locationxyz.y = centerROI.y + Q_13;
    locationxyz.z = Q_23;
    double W = Q_32 * disp.at<float>(centerROI.x, centerROI.y);
    locationxyz *= (1.0 / W);

  }
}

cv::Rect ParticleTrack::xz_to_box(cv::Point2f xz)
{
  cv::Rect box;
  box.height = size_ / xz.y;
  box.width = box.height * ar_;
  box.x = (xz.x+xz_map_res_/2.0) / xz.y * Q_23 - Q_03 - box.width / 2;

  
  double y = h; 
  y = y * std::cos(angle_ * CV_PI / 180.0) + xz.y * std::sin(angle_ * CV_PI / 180.0);

  
  double W = Q_23 / xz.y; 
  box.y = y * W - Q_13; 
  if(box.height <1 || box.width<1) ROS_WARN("Box = %d %d %d %d",box.x,box.y,box.height,box.width);

  return box;
}

cv::Point2f ParticleTrack::find_location(cv::Mat& disparity, cv::Rect roi)
{
  
  make_xz_map(disparity, &roi);

  
  cv::Point maxLoc;
  cv::minMaxLoc(xz_map_, NULL, NULL, NULL, &maxLoc);
  cv::Point2f loc;
  loc.x = maxLoc.x * xz_map_res_ + xz_map_offset_;
  loc.y = maxLoc.y * xz_map_res_;
  if(loc.y==0) loc.y = 1.0;
  return loc;
}

double ParticleTrack::score(cv::Mat& image, cv::Mat& disp, cv::Rect roi)
{
  double score = 0.0;
  
  cv::Point2f detLoc = find_location(disp, roi);
  cv::Point2f trkLoc;
  trkLoc.x = current_map_.x.at<float>(0, 0);
  trkLoc.y = current_map_.x.at<float>(1, 0);
  double dist = pow((double) detLoc.x - trkLoc.x, 2) + pow((double) detLoc.y - trkLoc.y, 2);
  dist = std::sqrt(dist);
  score += dist * -5.0 + 10.0; 

  
  roi = xz_to_box(detLoc);
  if ((roi.x > 0) && (roi.y > 0) && (roi.x + roi.width < image.cols) && (roi.y + roi.height < image.rows))
  {
    features1.computePool1(image, roi);
    double votes = forest1.classify(features1.featurePool1);
    score += votes / N_TREES;
    
  }

  
  return score;
}

void ParticleTrack::make_xz_map(cv::Mat& disp, cv::Rect* roi)
{
  
  xz_map_res_ = 0.0625; 
  xz_map_offset_ = -2.5; 
  xz_map_ = cv::Mat::zeros(5 / xz_map_res_, 5 / xz_map_res_, CV_32F); //5m x 5m

  int minI, maxI, minJ, maxJ;
  if (roi == NULL)
  {
    minI = 0;
    minJ = 0;
    maxI = disp.rows;
    maxJ = disp.cols;
  }
  else
  {
    minI = roi->y;
    minJ = roi->x;
    maxI = roi->y + roi->height;
    maxJ = roi->x + roi->width;
  }

  
  for (int i = minI; i < maxI; i++)
    for (int j = minJ; j < maxJ; j++)
      if (disp.at<float>(i, j) > 0.5)
      {
        Point3d xyz(j + Q_03, i + Q_13, Q_23);
        double W = Q_32 * disp.at<float>(i, j);
        xyz *= (1.0 / W);

        //Rotate
        Point3d rxyz;
        rxyz.x = xyz.x;
        rxyz.y = R.at<double>(1, 1) * xyz.y + R.at<double>(1, 2) * xyz.z;
        rxyz.z = R.at<double>(2, 1) * xyz.y + R.at<double>(2, 2) * xyz.z;
        xyz = rxyz;

        int x_idx = xyz.x / xz_map_res_ - xz_map_offset_ / xz_map_res_;
        int z_idx = xyz.z / xz_map_res_;
        
        if ((z_idx > 0) && (z_idx < xz_map_.rows) && (x_idx > 0) && (x_idx < xz_map_.cols))
        {
          if ((xyz.y > -.05) && (xyz.y < 1.0))
            xz_map_.at<float>(z_idx, x_idx) += xyz.z;
        } 
      } 
  cv::GaussianBlur(xz_map_, xz_map_, cv::Size(3, 3), 1.0);
}

void ParticleTrack::make_xz_map_weighted(cv::Mat& image, cv::Mat& disp, cv::Rect* roi)
{
  
  xz_map_res_ = 0.0625; 
  xz_map_offset_ = -2.5; 
  xz_map_ = cv::Mat::zeros(5 / xz_map_res_, 5 / xz_map_res_, CV_32F); //5m x 5m

  int minI, maxI, minJ, maxJ;
  if (roi == NULL)
  {
    minI = 0;
    minJ = 0;
    maxI = disp.rows;
    maxJ = disp.cols;
  }
  else
  {
    minI = roi->y;
    minJ = roi->x;
    maxI = roi->y + roi->height;
    maxJ = roi->x + roi->width;
  }


  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat planes[2];
  planes[0] = channels[0];
  planes[1] = channels[2];

  
  int bins[] = { 50, 50 };
  float h_ranges[] = { 0, 255 };
  float s_ranges[] = { 0, 255 };
  const float* ranges[] = { h_ranges, s_ranges };

  cv::Mat backproj;
  cv::calcBackProject(planes, 2, 0, hist_, backproj, &(ranges[0]), 1, true);
  backproj.convertTo(backproj, CV_32F);

  
  if(0)
  {
    imshow("BackProj", backproj / 10.0);
    cv::waitKey(10);
  }

  
  for (int i = minI; i < maxI; i++)
    for (int j = minJ; j < maxJ; j++) 
    {
      Point3d xyz(j + Q_03, i + Q_13, Q_23);
      double W = Q_32 * disp.at<float>(i, j);
      xyz *= (1.0 / W);

      
      Point3d rxyz;
      rxyz.x = xyz.x;
      rxyz.y = R.at<double>(1, 1) * xyz.y + R.at<double>(1, 2) * xyz.z;
      rxyz.z = R.at<double>(2, 1) * xyz.y + R.at<double>(2, 2) * xyz.z;
      xyz = rxyz;

      int x_idx = xyz.x / xz_map_res_ - xz_map_offset_ / xz_map_res_;
      int z_idx = xyz.z / xz_map_res_;
      
      if ((z_idx > 0) && (z_idx < xz_map_.rows) && (x_idx > 0) && (x_idx < xz_map_.cols))
      {
        if ((xyz.y > -.05) && (xyz.y < 1.0))
          xz_map_.at<float>(z_idx, x_idx) += xyz.z * backproj.at<float>(i, j);


      } 

    } 
  cv::GaussianBlur(xz_map_, xz_map_, cv::Size(5, 5), 2.0);
}

void ParticleTrack::make_object_hist(cv::Mat& image, cv::Mat& disp, cv::Point maxLoc)
{ 

  h = 0.356; 
  cv::Mat mask = cv::Mat::zeros(image.rows, image.cols, CV_8U);
  
  for (int i = 0; i < disp.rows; i++)
    for (int j = 0; j < disp.cols; j++)
      if (disp.at<float>(i, j) > 0.5)
      {
        Point3d xyz(j + Q_03, i + Q_13, Q_23);
        double W = Q_32 * disp.at<float>(i, j);
        xyz *= (1.0 / W);

        
        Point3d rxyz;
        rxyz.x = xyz.x;
        rxyz.y = R.at<double>(1, 1) * xyz.y + R.at<double>(1, 2) * xyz.z;
        rxyz.z = R.at<double>(2, 1) * xyz.y + R.at<double>(2, 2) * xyz.z;
        xyz = rxyz;

        int x_idx = xyz.x / xz_map_res_ - xz_map_offset_ / xz_map_res_;
        int z_idx = xyz.z / xz_map_res_;

        
        if ((z_idx == maxLoc.y) && (x_idx == maxLoc.x))
        {
          if ((xyz.y > -.05) && (xyz.y < 1.0))
            mask.at<uint8_t>(i, j) = 1;
        } // if

        
        if ((std::abs(z_idx - maxLoc.y) < 3) && (std::abs(x_idx - maxLoc.x) < 3))
        {
          if ((xyz.y > -.20) && (xyz.y < 1.0))
            if(xyz.y < h)
              h = xyz.y;
        } // if

      } // for i, j

  
  cv::Mat hsv;
  cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);
  cv::Mat planes[2];
  planes[0] = channels[0];
  planes[1] = channels[2];

  
  int bins[] = { 50, 50 };
  float h_ranges[] = { 0, 255 };
  float s_ranges[] = { 0, 255 };
  const float* ranges[] = { h_ranges, s_ranges };

  cv::calcHist(planes, 2, 0, mask, hist_, 2, &(bins[0]), &(ranges[0]), true, false);

  double sumT = cv::sum(hist_).val[0];
  hist_ = hist_ * 500.0 / sumT;
}
