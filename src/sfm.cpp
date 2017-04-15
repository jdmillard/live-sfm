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

    // this is the first iteration
    idx = 0;

    // start saving features
    features_all.push_back(features_new);

    // save the undistorted features
    undistortPoints(features_new, features_new_u, intrinsic, distortion, noArray(), intrinsic);
    features_all_u.push_back(features_new_u);

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
        edge = true;
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

    // remove the features with poor positioning if any were found
    if (edge)
    {
      cleanFeatures();
      edge = false;
    }
    else
    {
      features_mask.clear();
    }

    // use findFundamentalMat to locate the outlier feature points
    int ep_dist = 0.03 * frame_in.rows; // acceptable distance from epipolar line
    double confidence = 0.95; // confidence of correct F matrix (0-1)
    Mat F = findFundamentalMat(features_all[0], features_new, FM_RANSAC, ep_dist, confidence, features_mask);
    cleanFeatures();

    // save the features
    idx++;
    features_all.push_back(features_new);

    // save the undistorted features
    undistortPoints(features_new, features_new_u, intrinsic, distortion, noArray(), intrinsic);
    features_all_u.push_back(features_new_u);

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





void StructureFromMotion::cleanFeatures()
{
  // the purpose of this method is to clean up the original and recent
  // feature vectors based on the class member "features_mask"
  std::vector<cv::Point2f> features_a, features_b, features_c;

  for (int i=0; i<features_mask.size(); i++)
  {
    if (features_mask[i] == 1)
    {
      // keep the current feature
      features_a.push_back(features_all[0][i]);
      features_b.push_back(features_new[i]);
      features_c.push_back(features_all_u[0][i]);
    }
  }

  // update feature vectors
  features_all[0]   = features_a;
  features_new      = features_b;
  features_all_u[0] = features_c;

  // clean the mask
  features_mask.clear();
}





void StructureFromMotion::getRotationTranslation()
{
  // if this is the first iteration, there will be no rotation/translation

  if (idx==0)
  {
    // set no rotation and no translation
    R = Mat::eye(3,3, CV_64F);
    T = Mat(3,1, CV_64F);
    T.at<double>(0,0) = 0;
    T.at<double>(1,0) = 0;
    T.at<double>(2,0) = 0;
  }
  else
  {
    // current and original undistorted features to get fundamental matrix:
    Mat F = findFundamentalMat(features_all_u[0], features_all_u[idx], FM_8POINT);
    //F_all.push_back(F_new);

    // get the essential matrix
    Mat E = intrinsic.t()*F*intrinsic;

    // normalize using svd
    Mat w, u, vt, w2;
    SVD::compute(E, w, u, vt);
    w2 = Mat::eye(3,3, CV_64F);     // 3x3 identity
    w2.at<double>(2,2) = 0;         // new normalized singular values
    E = u * w2 * vt;

    // get rotation and translation using recoverPose
    double fx = intrinsic.at<double>(0,0);
    double fy = intrinsic.at<double>(1,1);
    double cx = intrinsic.at<double>(0,2);
    double cy = intrinsic.at<double>(1,2);

    // decomposing the essential matrix gives us 4 combinations of possible
    // R and T; recoverPose does the cheirality check to get the correct one
    recoverPose(E, features_all_u[0], features_all_u[idx], R, T, fx, Point2f(cx, cy));

  }


  //std::cout << pow(pow(T.at<double>(0,0),2) + pow(T.at<double>(1,0),2) + pow(T.at<double>(2,0),2), 0.5) << std::endl;

  //std::cout << "---" << std::endl;
  //std::cout << T << std::endl;

  // get E from F
  // svd normalize
  // get r and t using recoverPose

  // translation is normalized, if we know the radius of the ball, we can
  // back out a scale factor
}





void StructureFromMotion::scaleTranslation()
{
  // use the highest ranked circle that exists in original and current frame
  // to resolve scale ambiguity

  // need a good circle hierarchy and association for this
  //std::cout << "here" << std::endl;
}
