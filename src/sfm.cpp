#include "sfm.h"

using namespace cv;


void StructureFromMotion::newFrame(Mat frame_in)
{
  // this method is to be overwritten by the derived class
  // it is supposed to perform logic based on the frame id to determine if
  // the current frame is a keyframe
  std::cout << "base class method needs to be overwritten!" << std::endl;
}

void StructureFromMotion::featureTracker(Mat frame_in)
{

  if (features_cur.size()==0)
  {
    // no features have been initialized
    // populate the feature vector

    // convert to intensity image
    Mat frame_gray;
    cvtColor(frame_in, frame_gray, CV_BGR2GRAY);

    int max_points = 1000;
    double quality = 0.01;
    double min_dist = 10;
    Mat mask;
    int blockSize = 3;
    bool useHarris = false;
    double k = 0.04;
    std::cout << "here" << std::endl;
    goodFeaturesToTrack(frame_gray, features_cur, max_points, quality, min_dist, mask, blockSize, useHarris, k);

    // REMEMBER TO PUSH BACK ON features_all !!!!!!!!!!!!!!!!!!!!!!!!
  }
  else
  {
    // features have been initialized, perform tracking, remove outliers (from all feature sets of all idx)
    // look into the "keep" command with masking
    // REMEMBER TO PUSH BACK ON features_all !!!!!!!!!!!!!!!!!!!!!!!!

  }




}
