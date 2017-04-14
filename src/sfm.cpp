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


  if (features_old.size()==0)
  {
    // no features have been initialized
    // populate the feature vector

    // load calibration
    loadCalibration();

    // find goodFeaturesToTrack
    int max_points = 1000;
    double quality = 0.01;
    double min_dist = 10;
    Mat mask;
    int blockSize = 3;
    bool useHarris = false;
    double k = 0.04;
    goodFeaturesToTrack(frame_gray, features_new, max_points, quality, min_dist, mask, blockSize, useHarris, k);

    // draw the features
    drawFeatures(frame_in, features_new);

    // this is the first iteration
    idx = 0;

    // start saving features
    features_all.push_back(features_new);

    // update the feature vectors for next iteration
    features_old.clear();
    features_old = features_new;
    features_new.clear();

    // remember the last iteration's frame
    frame_gray_old = frame_gray.clone();
  }
  else
  {
    // features have been initialized, perform tracking

    // template matching
    int d_wind = 81; // window dimension
    int d_temp = 31; // template dimension

    std::vector<Point2f> features_new;
    for (int i=0; i<features_old.size(); i++)
    {

      // create a window from current image for the current feature
      int x1 = features_old[i].x - (d_wind-1)/2;
      x1 = std::max(x1, 0);
      x1 = std::min(x1, frame_in.cols-d_wind);
      int y1 = features_old[i].y - (d_wind-1)/2;
      y1 = std::max(y1, 0);
      y1 = std::min(y1, frame_in.rows-d_wind);
      Mat win = frame_gray(Rect(x1, y1, d_wind, d_wind));

      // create a template from previous image for the current feature
      int x2 = features_old[i].x - (d_temp-1)/2;
      int y2 = features_old[i].y - (d_temp-1)/2;
      if (x2 < 0 || x2 > frame_in.cols-d_temp || y2 < 0 || y2 > frame_in.rows-d_temp)
      {
        // template doesn't fit in frame, it's too close to the edge
        // mark it as a bad feature and enter a false location
        features_mask.push_back(0);
        features_new.push_back(features_old[i]);
      }
      else
      {
        // feature location is good, not too close to the edge
        // mark it as a good feature and perform template matching
        features_mask.push_back(1);

        // generate good template
        Mat tem = frame_gray_old(Rect(x2, y2, d_temp, d_temp));

        // match the current template and window
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
      }



    } // end of looping through features


    // draw the features
    drawFeatures(frame_in, features_new);

    // saving the features
    idx++;
    features_all.push_back(features_new);

    // here use mask to clean up feature vectors (first and most recent)
    // make mask cleanup function
    // use findFundamentalMatrix to chop off outliers between:
    // two most recent, first and last
    // whenever done with cleanup, run     features_mask.clear();

    // update the feature vectors for next iteration
    features_old.clear();
    features_old = features_new;
    features_new.clear();

    // remember the last iteration's frame
    frame_gray_old = frame_gray.clone();


  } // end of if check


} // end of method





void StructureFromMotion::drawFeatures(Mat img, std::vector<Point2f>& features)
{
  for (int i=0; i < features.size(); i++)
  {
  circle(img, features[i], 2, Scalar(255, 0, 0), 2);
  }
}
