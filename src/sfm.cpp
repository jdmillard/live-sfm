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
  // convert to intensity image
  Mat frame_gray;
  cvtColor(frame_in, frame_gray, CV_BGR2GRAY);


  if (features_cur.size()==0)
  {
    // no features have been initialized
    // populate the feature vector

    int max_points = 1000;
    double quality = 0.01;
    double min_dist = 10;
    Mat mask;
    int blockSize = 3;
    bool useHarris = false;
    double k = 0.04;
    std::cout << "here" << std::endl;
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
    int dim_win = 81;
    int dim_tem = 31;
    // delcare a features_new
    // rename features_cur to old
    std::vector<Point2f> features_new;
    for (int k=0; k<features_cur.size(); k++)
    {
      std::cout << "---" << std::endl;
      std::cout << features_cur[k] << std::endl;
      // create a window around features_cur[k], which is Point2f
      int x1 = features_cur[k].x - dim_win/2;
      if (x1 < 0)
      {
        x1 = 0;
      }
      if ((x1+dim_win)>frame_in.cols)
      {
        x1 = frame_in.cols - dim_win;
      }
      int y1 = features_cur[k].y - dim_win/2;
      if (y1 < 0)
      {
        y1 = 0;
      }
      if ((y1+dim_win)>frame_in.rows)
      {
        y1 = frame_in.rows - dim_win;
      }
      // window established in next image
      Mat win = frame_gray(Rect(x1, y1, dim_win, dim_win));

      // create a template around features_cur[k], which is Point2f
      int x2 = features_cur[k].x - dim_tem/2;
      if (x2 < 0)
      {
        x2 = 0;
        features_cur[k].x = x2 + dim_tem/2;
      }
      if ((x2+dim_tem)>frame_in.cols)
      {
        x2 = frame_in.cols - dim_tem;
        features_cur[k].x = x2 + dim_tem/2;
      }
      int y2 = features_cur[k].y - dim_tem/2;
      if (y2 < 0)
      {
        y2 = 0;
        features_cur[k].y = y2 + dim_tem/2;
      }
      if ((y2+dim_tem)>frame_in.rows)
      {
        y2 = frame_in.rows - dim_tem;
        features_cur[k].y = y2 + dim_tem/2;
      }

      // window established in next image
      Mat tem = frame_gray_old(Rect(x2, y2, dim_tem, dim_tem));

      // match the current template of image 1 to window of image 2
      Mat output;
      matchTemplate(win, tem, output, TM_CCORR_NORMED);
      std::cout << output.size() << std::endl;

      // normalize the intensity output
      normalize(output, output, 0, 1, NORM_MINMAX,  -1, Mat());

      // locate the position of highest correlation
      Point max_point;
      minMaxLoc(output, 0, 0, 0, &max_point, Mat());
      std::cout << max_point << std::endl;

      // represent the max point in original image coordinates
      max_point.x = max_point.x + dim_tem/2 + x1;
      max_point.y = max_point.y + dim_tem/2 + y1;
      std::cout << max_point << std::endl;
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
