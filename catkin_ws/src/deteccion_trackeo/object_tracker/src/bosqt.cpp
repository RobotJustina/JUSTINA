
#include <object_tracker/bosqt.h>
#include <ros/ros.h>


    forest::forest()
  {
    h = cv::Mat::zeros(1, TREE_DEPTH, CV_16S);
    for (int i = 0; i < N_TREES; i++)
    {
      trees[i].dTree = new swriTree;
    }
  }

  forest::~forest()
  {
    for (int i = 0; i < N_TREES; i++)
    {
      delete trees[i].dTree;
    }
  }

  int forest::classify(cv::Mat& sequence)
  {
    int votes = 0;
    
    for (int i = 0; i < N_TREES; i++)
    {
      subFeature(sequence, i, h);
      trees[i].treeVote = trees[i].dTree->predict(h);
      
      if (trees[i].treeVote == 1)
      {
        votes++;
      }
    }

    return votes;
  }

  void forest::addToTrees(cv::Mat& sequence, bool protect)
  {
    for (int i = 0; i < N_TREES; i++)
    {  
      subFeature(sequence, i, h);
      trees[i].dTree->addToTree(h, protect);  
    }
  }

  void forest::removeFromTrees(cv::Mat& sequence, bool protect)
  {
    for (int i = 0; i < N_TREES; i++)
    {  
      subFeature(sequence, i, h);
      trees[i].dTree->removeFromTree(h, protect);  
    }
  }

  void forest::subFeature(cv::Mat& sequence, int i, cv::Mat& h)
  {
    for (int j = 0; j < TREE_DEPTH; j++)
    {
      h.at<int16_t>(0,j) = sequence.at<int16_t>(0,trees[i].features[j]);
    }
  }

  void forest::randomFeatures(int poolSize)
  {
    for (int i = 0; i < N_TREES; i++)
      for (int j = 0; j < TREE_DEPTH; j++)
      {
        trees[i].features[j] = rand() % poolSize;
      }
  }

  
