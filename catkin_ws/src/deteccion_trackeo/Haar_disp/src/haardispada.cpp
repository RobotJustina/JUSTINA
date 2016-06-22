#include "Haar_disp/haardispada.hpp"
#include <stdlib.h>
#include <algorithm>

namespace HaarDispAda {


  HaarDispAdaClassifier::HaarDispAdaClassifier()
  {
    classifier_filename_ = "UnNamedHaarDispAda.xml";
    HaarDispAdaPrior_    = exp(2); 
    loaded = false;
    init();
  }

  HaarDispAdaClassifier::HaarDispAdaClassifier(string filename)
  {  
    classifier_filename_ = filename;
    loaded = false;
    init();
  }

  void HaarDispAdaClassifier::detect(vector<Rect> &R_in, 
             vector<int>  &L_in,
             Mat &D_in, 
             vector<Rect> &R_out, 
             vector<int> &L_out,
             bool label_all)
  {
    int count =0;
    Mat HF(1,num_filters_,CV_32F);
    Mat MH(1,num_filters_,CV_8UC1);

    R_out.clear();
    L_out.clear();
    if(!loaded) return;
    for(unsigned int i=0;i<R_in.size();i++){
      float result = 0;
      if(R_in[i].width > 2 && R_in[i].height > 2){
  setDImageROI_fast(R_in[i],D_in); 
  int rtn = haar_features_fast(HF); 
  if(rtn== 1){
    result = HDAC_.predict(HF); 
  }
        else{
    ROS_ERROR("WHY O WHY");
          result = 0;
  }
      }
      if(result>0 || label_all == true){
  R_out.push_back(R_in[i]);
  if(label_all) L_out.push_back(result); 
  if(!label_all)L_out.push_back(L_in[i]); 
  if(result>0) count++;
      }
    }
  }

  int HaarDispAdaClassifier::addToTraining(vector<Rect> &R_in, vector<int> &L_in, Mat &D_in)
  {
    Mat HF(1,num_filters_,CV_32F);
    Mat MH(1,num_filters_,CV_8UC1);
    for(unsigned int i = 0; i<R_in.size(); i++){
      if(R_in[i].width < 2 || R_in[i].height <2){
  
      }
      else if(numSamples_<maxSamples_){
  setDImageROI_fast(R_in[i],D_in);
  int rtn = haar_features_fast(HF);
  if((rtn) && (find_central_disparity(R_in[i].x, R_in[i].y, R_in[i].height, R_in[i].width, D_in) > 0.0)){
    for(int j=0;j<num_filters_;j++){
      trainingSamples_.at<float>(numSamples_,j) = HF.at<float>(0,j);
    }
    if(L_in[i] <= 0 ){
      trainingLabels_.at<int>(numSamples_,0) = 0;
    }
    else{
      trainingLabels_.at<int>(numSamples_,0) = L_in[i];
    }
    numSamples_++;
  }
      }
    }
    return(numSamples_);
  }

  void HaarDispAdaClassifier::setDImageRoi(Rect &R_in, Mat &I_in)
  {
    int xmin = R_in.x;
    int xmax = xmin+R_in.width;
    int ymin = R_in.y;
    int ymax = ymin+R_in.height;
    Image4Haar = I_in(Range(ymin,ymax),Range(xmin,xmax));
  }

  void HaarDispAdaClassifier::train(string filename)
  {
    CvBoostParams bparams = CvBoostParams();
    float priorFloat[] = { 1.0, HaarDispAdaPrior_ };  
    bparams.priors = &priorFloat[0];
    bparams.use_surrogates = false;
    bparams.weak_count = 100;

    
    Mat VarIdx;
    Mat Features(numSamples_,num_filters_,CV_32F);
    for(int i=0;i<numSamples_;i++){
      for(int j=0;j<num_filters_;j++){
  Features.at<float>(i,j)       = trainingSamples_.at<float>(i,j);
      }
    }
    Mat Responses(numSamples_,1,CV_32S);
    for(int i=0; i<numSamples_;i++){
      if(trainingLabels_.at<int>(i,0) == -1) trainingLabels_.at<int>(i,0) = 0;
      Responses.at<int>(i,0) = trainingLabels_.at<int>(i,0);
    }
    Mat vIdx=Mat::ones(Features.cols,1,CV_8UC1); 
    Mat sIdx=Mat::ones(Responses.rows,1,CV_8UC1); 
    Mat vtyp=Mat(Features.cols,1,CV_8UC1,CV_VAR_ORDERED); 
    Mat MDM; 
    HDAC_.train(Features, CV_ROW_SAMPLE, Responses,vIdx,sIdx,vtyp,MDM,bparams,false);
    ROS_ERROR("saving trained classifier to %s",filename.c_str());
    loaded = true;
    HDAC_.save(filename.c_str());
    
    int num_TP     = 0;
    int num_FP     = 0;
    int num_people = 0;
    int num_neg    = 0;
    int num_TN     = 0;
    int num_FN     = 0;
    for(int i=0;i<Features.rows;i++){
      float result = HDAC_.predict(Features.row(i));
      if(Responses.at<int>(i,0) == 1) num_people++;
      if(Responses.at<int>(i,0) != 1) num_neg++;
      if(result==1 && Responses.at<int>(i,0) == 1) num_TP++; 
      else if(result==1 && Responses.at<int>(i,0) == 0) num_FP++; 
      else if(result==0 && Responses.at<int>(i,0) == 0) num_TN++; 
      else if(result==0 && Responses.at<int>(i,0) == 1) num_FN++; 
    }
    float percent = (float)num_TP/(float)num_people*100.0;
    ROS_ERROR("Recall = %6.2f%c",percent,'%');
    percent = (float)num_FP/(float)num_neg*100.0;
    ROS_ERROR("False Positives = %6.2f%c",percent,'%');
  }

  void HaarDispAdaClassifier::setMaxSamples(int n)
  {
    maxSamples_ = n;
    numSamples_ = 0;
    trainingSamples_.create(maxSamples_,num_filters_,CV_32FC1);
    trainingLabels_.create(maxSamples_,1,CV_32SC1);

  }  

  void HaarDispAdaClassifier::init()
  {
    num_filters_ = 174;
    setMaxSamples(350); 

    AvePosTrainingImg_.create(64,64,CV_64F);
    
    
    map.create(16,16,CV_16SC1);

    
    haar16x16.create(16, 16, CV_32F);
    haar8x8.create(8, 8, CV_32F);
    haar4x4.create(4, 4, CV_32F);

  }

  void HaarDispAdaClassifier::load(string filename)
  {
    HDAC_.load(filename.c_str());
    loaded = true;
  }

  int HaarDispAdaClassifier::test()
  {
    return(1);
  }

  
  void HaarDispAdaClassifier::alpha_map(int idx)
  {
    int i, j, rowWidth, lPower, lPower2, type;

    type = idx % 3;  

    if (idx >= 174)
    {  
      rowWidth = 15;
      idx = idx - 174;
      lPower = 2;
      lPower2 = 1;
    }
    else if (idx >= 27)
    {  
      rowWidth = 7;
      idx = idx - 27;
      lPower = 4;
      lPower2 = 2;
    }
    else
    {  
      rowWidth = 3;
      lPower = 8;
      lPower2 = 4;
    }  

    int i1 = (idx / 3) % rowWidth;
    int j1 = (idx / 3) / rowWidth;

    
    map = cv::Mat::zeros(map.rows,map.cols,CV_16SC1);

    for (i = i1 * lPower2; i < i1 * lPower2 + lPower; i++)
    {  // width/x
      for (j = j1 * lPower2; j < j1 * lPower2 + lPower; j++)
      {  // height/y
        if (type == 0)
        {  // horz type
          if (i >= i1 * lPower2 + lPower2)
          {
      map.at<short int>(i,j) = 1;
          }
          else
          {
      map.at<short int>(i,j) = -1;
          }
        }
        else if (type == 1)
        {  // vert type
          if (j >= j1 * lPower2 + lPower2)
          {
      map.at<short int>(i,j) = 1;
          }
          else
          {
      map.at<short int>(i,j) = -1;
          }
        }
        else
        {  
          if (((i >= i1 * lPower2 + lPower2) && (j >= j1 * lPower2 + lPower2))
              || ((i < i1 * lPower2 + lPower2) && (j < j1 * lPower2 + lPower2)))
          {
      map.at<short int>(i,j) = 1;
          }
          else
          {
      map.at<short int>(i,j) = -1;
          }
        }  // if
      }  // for j
    }  // for i
  }  // alpha map


  void HaarDispAdaClassifier::mask_scale(Mat & input, Mat & output)
  {  
    assert(input.type() == CV_32F);
    float hratio = (float) input.rows / (float) output.rows;
    float wratio = (float) input.cols / (float) output.cols;
    for (int i = 0; i < output.rows; i++)
      {
  float *outputRow = output.ptr<float>(i);
  for (int j = 0; j < output.cols; j++)
    {
      outputRow[j] = 0;
      int n = 0;
      for (int i1 = i * hratio; (i1 < (i + 1) * hratio) && (i1 < input.rows); i1++)
        {
    float *inputRow = input.ptr<float>(i1);
    for (int j1 = j * wratio; (j1 < (j + 1) * wratio) && (j1 < input.cols); j1++)
      {
        if (inputRow[j1] > 0)  
          {
      outputRow[j] += inputRow[j1];
      n++;  
          }  // if
      }  // for i1, j1
        }
      if (n > 0)
        outputRow[j] /= n;  
    }  // for i,j
      }  // i
  }  
  

  void HaarDispAdaClassifier::setDImageROI_fast(Rect & R_in, Mat & I_in)
  {
    Mat ROI = I_in(R_in);
    mask_scale(ROI, haar16x16);
  }

  int HaarDispAdaClassifier::haar_features_fast(Mat &HF)
  {
    int jl, idx, i1, j1;

    cv::resize(haar16x16, haar8x8, cv::Size(8,8), 0, 0, CV_INTER_AREA);
    cv::resize(haar8x8, haar4x4, cv::Size(4,4), 0, 0, CV_INTER_AREA);

    float* hf_ptr = HF.ptr<float>(0);

    if(haar16x16.rows != 16 || haar16x16.cols !=16)
      {
  ROS_ERROR("Wrong sz Input4Haar haar_response %dX%d ",haar16x16.rows,haar16x16.cols);
  return(0);
      }


    
    for (jl = 0; jl < 27; jl += 3)
    {  
      idx = jl;
      i1 = (idx / 3) % 3;  
      j1 = (idx / 3) / 3;  
      hf_ptr[jl] = 16 * (-haar4x4.at<float>(j1,i1) + haar4x4.at<float>(j1,i1+1)
                      -haar4x4.at<float>(j1 +1 ,i1) + haar4x4.at<float>(j1 + 1,i1 + 1));

      hf_ptr[jl+1] = 16 * (-haar4x4.at<float>(j1,i1) - haar4x4.at<float>(j1,i1+1)
                         +haar4x4.at<float>(j1 +1 ,i1) + haar4x4.at<float>(j1 + 1,i1 + 1));

      hf_ptr[jl+2] = 16 * (haar4x4.at<float>(j1,i1) - haar4x4.at<float>(j1,i1+1)
                        -haar4x4.at<float>(j1 +1 ,i1) + haar4x4.at<float>(j1 + 1,i1 + 1));
    }  // for

    
    for (jl = 27; jl < 174; jl += 3)
    {  
      idx = jl - 27;
      i1 = (idx / 3) % 7;  
      j1 = (idx / 3) / 7;  
      hf_ptr[jl] = 4 * (-haar8x8.at<float>(j1,i1) + haar8x8.at<float>(j1,i1+1)
                      -haar8x8.at<float>(j1+1 ,i1) + haar8x8.at<float>(j1 + 1,i1 + 1));

      hf_ptr[jl+1] = 4 * (-haar8x8.at<float>(j1,i1) - haar8x8.at<float>(j1,i1+1)
                         +haar8x8.at<float>(j1 +1 ,i1) + haar8x8.at<float>(j1 + 1,i1 + 1));

      hf_ptr[jl+2] = 4 * (haar8x8.at<float>(j1,i1) - haar8x8.at<float>(j1,i1+1)
                        -haar8x8.at<float>(j1 +1 ,i1) + haar8x8.at<float>(j1 + 1,i1 + 1));
    }  // for
    return 1;

  }  


  int HaarDispAdaClassifier::haar_features(Mat &HF, Mat &MH)
  {
    for (int k = 0; k < num_filters_; k++){ 
      
      alpha_map(k); 
      Mat ScaledMap(Image4Haar.rows,Image4Haar.cols,CV_16SC1);
      resize(map, ScaledMap, Size(Image4Haar.cols, Image4Haar.rows), 0, 0, CV_INTER_NN ); 

      
      float plus_sum = 0;
      float minus_sum = 0;
      int num_plus=0;
      int num_minus=0;
      for(int ii=0;ii<Image4Haar.rows;ii++){
  float     *img_ptr = (float *)     &Image4Haar.at<float>(ii,0);
  short int *map_ptr = (short int *) &ScaledMap.at<short int>(ii,0);
  for(int jj=0;jj<Image4Haar.cols;jj++){
    if(*img_ptr !=0){
      if(*map_ptr==1){ 
        num_plus++;
        plus_sum += *img_ptr;
      }
      else if(*map_ptr==-1){ 
        num_minus++;
        minus_sum += *img_ptr;
      }
    }
    img_ptr++; 
    map_ptr++; 
  }
      }

      if(plus_sum ==0 || minus_sum == 0){
  MH.at<uchar>(0,k) =(uchar) 1; 
  HF.at<float>(0,k) = 0; 
      }
      else{
  MH.at<uchar>(0,k) = (uchar) 0; 
  float plus_ave  = plus_sum/num_plus;
  float minus_ave = minus_sum/num_minus;
  float ave_dif   = fabs(plus_ave - minus_ave);
  
  float scaling=2.0;
  if(k<27){
    scaling = 32.0;
  }
  else if(k<174){
    scaling = 8.0;
  }
  HF.at<float>(0,k) = scaling*ave_dif;
      }
    } 
    return(1);
  }// haar_features
  void HaarDispAdaClassifier::print_map(int k)
  {
    printf("Map %d :\n",k);
    alpha_map(k);
    for(int i=0;i<map.rows;i++){
      for(int j=0;j<map.cols;j++){
  printf("%3d ",map.at<short int>(i,j));
      }
      printf("\n");
    }
  }
  void HaarDispAdaClassifier::print_scaled_map(cv::Mat &A)
  {
    printf("SMap = [\n");
    for(int i=0;i<A.rows;i++){
      for(int j=0;j<A.cols;j++){
  printf("%2d",A.at<short int>(i,j));
      }
      printf("\n");
    }
    printf("]\n");
  }
  void HaarDispAdaClassifier::print_Image4Haar()
  {
    printf("Image4Haar = [\n");
    for(int i=0;i<Image4Haar.rows;i++){
      for(int j=0;j<Image4Haar.cols;j++){
  printf("%6.2f ",Image4Haar.at<float>(i,j));
      }
      printf("\n");
    }
    printf("]\n");
  }
  
    float HaarDispAdaClassifier::find_central_disparity(int x, int y, int height, int width, Mat &D_in)
  {
    
    if(x+width>D_in.cols || y+height>D_in.rows) return(0.0);

    float ad=0.0;
    int j=width/2.0;
    for(int i=0;i<height/3;i++){
      if(D_in.at<float>(y+i,x+j)>ad){
  ad = D_in.at<float>(y+i,x+j);
      }
    }

    return(ad);
  }


} // namespace HaarDispAda
