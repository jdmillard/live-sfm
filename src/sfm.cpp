#include "sfm.h"

using namespace cv;


void StructureFromMotion::loadCalibration()
{
  // load saved calibration
  FileStorage fsr("../calibration/calibration.xml", FileStorage::READ);
  fsr["intrinsic"] >> intrinsic;
  fsr["distortion"] >> distortion;
  fsr.release();
}

void StructureFromMotion::newFrame(Mat frame_in)
{
  // this method is to be overwritten by the derived class
  // it is supposed to perform logic based on the frame id to determine if
  // the current frame is a keyframe
  std::cout << "base class method needs to be overwritten!" << std::endl;
}

void StructureFromMotion::featureTracker(Mat frame_in)
{
  // convert to intensity image
  Mat frame_gray;
  cvtColor(frame_in, frame_gray, CV_BGR2GRAY);


  if (features_cur.size()==0)
  {
    // no features have been initialized
    // populate the feature vector

    // load calibration
    loadCalibration();

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
    goodFeaturesToTrack(frame_gray, features_cur, max_points, quality, min_dist, mask, blockSize, useHarris, k);

    drawFeatures(frame_in, features_cur);

    // remember the last iteration's frame
    frame_gray_old = frame_gray.clone();

    // REMEMBER TO PUSH BACK ON features_all !!!!!!!!!!!!!!!!!!!!!!!!
  }
  else
  {
    // features have been initialized, perform tracking, remove outliers (from all feature sets of all idx)
    // look into the "keep" command with masking

    // template matching
    int d_wind = 81; // window dimension
    int d_temp = 31; // template dimension

    // rename features_cur to old
    std::vector<Point2f> features_new;
    std::vector<bool>    mask;
    for (int i=0; i<features_cur.size(); i++)
    {

      // create a window from current image for the current feature
      int x1 = features_cur[i].x - (d_wind-1)/2;
      x1 = std::max(x1, 0);
      x1 = std::min(x1, frame_in.cols-d_wind);
      int y1 = features_cur[i].y - (d_wind-1)/2;
      y1 = std::max(y1, 0);
      y1 = std::min(y1, frame_in.rows-d_wind);
      Mat win = frame_gray(Rect(x1, y1, d_wind, d_wind));

      // create a template from previous image for the current feature
      int x2 = features_cur[i].x - (d_temp-1)/2;
      x2 = std::max(x2, 0);
      x2 = std::min(x2, frame_in.cols-d_temp);
      int y2 = features_cur[i].y - (d_temp-1)/2;
      y2 = std::max(y2, 0);
      y2 = std::min(y2, frame_in.rows-d_temp);
      Mat tem = frame_gray_old(Rect(x2, y2, d_temp, d_temp));

      // THIS ALLOWS OFF-CENTER FEATURES!!! which makes the max_point logic
      // wrong lines 89 and 90

      // match the current template of image 1 to window of image 2
      Mat output;
      matchTemplate(win, tem, output, TM_CCORR_NORMED);

      // normalize the intensity output
      normalize(output, output, 0, 1, NORM_MINMAX,  -1, Mat());

      // locate the position of highest correlation
      Point max_point;
      minMaxLoc(output, 0, 0, 0, &max_point, Mat());

      // represent the max point in original image coordinates
      max_point.x = max_point.x + d_temp/2 + x1;
      max_point.y = max_point.y + d_temp/2 + y1;

      features_new.push_back(max_point);

    } // end of looping through features


    drawFeatures(frame_in, features_new);
    features_cur.clear();
    features_cur = features_new;

    frame_gray_old = frame_gray.clone();

    // REMEMBER TO PUSH BACK ON features_all !!!!!!!!!!!!!!!!!!!!!!!!

  } // end of if check


} // end of method





void StructureFromMotion::drawFeatures(Mat img, std::vector<Point2f>& features)
{
  for (int i=0; i < features.size(); i++)
  {
  circle(img, features[i], 2, Scalar(255, 0, 0), 2);
  }
}
