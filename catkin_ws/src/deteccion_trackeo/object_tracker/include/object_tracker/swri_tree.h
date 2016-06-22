
#ifndef SIMPL_TREE_H_
#define SIMPL_TREE_H_

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

  /**
   * Simple decision tree class
   * The decision tree makes binary decisions that lead to an end node.
   * The end node (leaf) will have a class value of non-object or object
   */

  class swriTree{

    /**
     * Node class for decision tree
     */
    class node{
    public:
      node* parent;
      node* left;
      node* right;
      int nClass;
      bool isProtected;

      node(){
        parent = NULL;
        left = NULL;
        right = NULL;
        nClass = 2;
        isProtected = false;
      }
      ~node(){
        delete left;
        delete right;
      }
    };
    node* root;
    int depth;
  public:
 
    swriTree();
    ~swriTree();

    /**
     * Predict class of an input sequence
     * input sequence The input feature vector
     * Return the tree's prediction
     */
    int predict(cv::Mat& sequence);

    /**
     * Add a sample to a tree
     * input sequence the sample to add
     * input protect whether or not to protect the leaf from pruning
     */
    void addToTree(cv::Mat& sequence,bool protect = false);

    /**
     * Remove a sample to a tree
     * input sequence the sample to remove
     * input protect whether or not to protect the leaf from growing
     */
    void removeFromTree(cv::Mat& sequence, bool protect = false);

    bool isDead(node* loc);
  }; //class simplTree

#endif // SIMPL_TREE_H_
