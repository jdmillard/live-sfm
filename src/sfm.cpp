#include "sfm.h"

using namespace cv;

// Mat triangulate_Linear_LS(Mat mat_P_l, Mat mat_P_r, Mat warped_back_l, Mat warped_back_r)
// {
//     Mat A(4,3,CV_64FC1), b(4,1,CV_64FC1), X(3,1,CV_64FC1), X_homogeneous(4,1,CV_64FC1), W(1,1,CV_64FC1);
//     W.at<double>(0,0) = 1.0;
//     A.at<double>(0,0) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,0) - mat_P_l.at<double>(0,0);
//     A.at<double>(0,1) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,1) - mat_P_l.at<double>(0,1);
//     A.at<double>(0,2) = (warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,2) - mat_P_l.at<double>(0,2);
//     A.at<double>(1,0) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,0) - mat_P_l.at<double>(1,0);
//     A.at<double>(1,1) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,1) - mat_P_l.at<double>(1,1);
//     A.at<double>(1,2) = (warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,2) - mat_P_l.at<double>(1,2);
//     A.at<double>(2,0) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,0) - mat_P_r.at<double>(0,0);
//     A.at<double>(2,1) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,1) - mat_P_r.at<double>(0,1);
//     A.at<double>(2,2) = (warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,2) - mat_P_r.at<double>(0,2);
//     A.at<double>(3,0) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,0) - mat_P_r.at<double>(1,0);
//     A.at<double>(3,1) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,1) - mat_P_r.at<double>(1,1);
//     A.at<double>(3,2) = (warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,2) - mat_P_r.at<double>(1,2);
//     b.at<double>(0,0) = -((warped_back_l.at<double>(0,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,3) - mat_P_l.at<double>(0,3));
//     b.at<double>(1,0) = -((warped_back_l.at<double>(1,0)/warped_back_l.at<double>(2,0))*mat_P_l.at<double>(2,3) - mat_P_l.at<double>(1,3));
//     b.at<double>(2,0) = -((warped_back_r.at<double>(0,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,3) - mat_P_r.at<double>(0,3));
//     b.at<double>(3,0) = -((warped_back_r.at<double>(1,0)/warped_back_r.at<double>(2,0))*mat_P_r.at<double>(2,3) - mat_P_r.at<double>(1,3));
//     solve(A,b,X,DECOMP_SVD);
//     vconcat(X,W,X_homogeneous);
//     return X_homogeneous;
// }

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





void StructureFromMotion::triangulatePointsCustom(Mat frame_in, int idx_circle, std::vector<std::vector<Vec3f>> circles_all_u, std::vector<std::vector<int>> circles_hierarchy_all)
{
  // stereoRectify and R/T will give us the respective camera projections where
  // x1 = P1*X1
  // x2 = P2*X2
  // where x and X are camera frame and world frame homogenous coordinates

  // we use these relationships to create an Ax = 0 overdetermined system
  // where we can find the x based on the right singular vecor associated with
  // the smallest singular value

  // so ultimately, we maintain an A matrix for each ranked circle

  // get P1 and P2 using stereoRectify
  Mat R1, R2, P1, P2, Q;
  stereoRectify(  intrinsic, distortion,
                  intrinsic, distortion,
                  frame_in.size(),  R,  T,
                  R1, R2, P1, P2, Q );

  // initialize A_all to match the circles detected in the first frame
  if (A_all.size()==0)
  {
    A_all.resize(circles_all_u[0].size());
    X_all.resize(circles_all_u[0].size());
  }

  for (int i=0; i<A_all.size(); i++)
  {
    // i is the current rank being considered
    // if this rank number is found in both frames, then we'll update
    // the corresponding A matrix which is A_all[i]

    int idx_original, idx_current;
    // find the circles_all_u[0] index of the circle for this rank
    for (int j=0; j<circles_all_u[0].size(); j++)
    {
      if (circles_hierarchy_all[0][j] == i)
      idx_original = j;
    }
    // find the circles_all_u[idx] index of the circle for this rank
    idx_current = 500;
    for (int j=0; j<circles_all_u[idx].size(); j++)
    {
      if (circles_hierarchy_all[idx][j] == i)
      idx_current = j;
    }

    if (idx_current!=500)
    {
      // the current frame detected the circle for this rank
      //std::cout << "----" << std::endl;
      //std::cout << idx_original << std::endl;
      //std::cout << idx_current << std::endl;

      // use P1 and P2 along with the x and y camera frame values
      // to populate an A block

      Mat A(4,4,CV_64FC1);
      double x, y;

      // first do the original point with P1
      x = circles_all_u[0][idx_original][0];
      y = circles_all_u[0][idx_original][1];
      // first row (x*p2_vec - p0_vec)
      A.at<double>(0,0) = P1.at<double>(2,0)*x - P1.at<double>(0,0);
      A.at<double>(0,1) = P1.at<double>(2,1)*x - P1.at<double>(0,1);
      A.at<double>(0,2) = P1.at<double>(2,2)*x - P1.at<double>(0,2);
      A.at<double>(0,3) = P1.at<double>(2,3)*x - P1.at<double>(0,3);
      // second row (y*p2_vec - p1_vec)
      A.at<double>(1,0) = P1.at<double>(2,0)*y - P1.at<double>(1,0);
      A.at<double>(1,1) = P1.at<double>(2,1)*y - P1.at<double>(1,1);
      A.at<double>(1,2) = P1.at<double>(2,2)*y - P1.at<double>(1,2);
      A.at<double>(1,3) = P1.at<double>(2,3)*y - P1.at<double>(1,3);

      // now do the current point with P2
      x = circles_all_u[idx][idx_current][0];
      y = circles_all_u[idx][idx_current][1];
      // first row (x*p2_vec - p0_vec)
      A.at<double>(2,0) = P2.at<double>(2,0)*x - P2.at<double>(0,0);
      A.at<double>(2,1) = P2.at<double>(2,1)*x - P2.at<double>(0,1);
      A.at<double>(2,2) = P2.at<double>(2,2)*x - P2.at<double>(0,2);
      A.at<double>(2,3) = P2.at<double>(2,3)*x - P2.at<double>(0,3);
      // second row (y*p2_vec - p1_vec)
      A.at<double>(3,0) = P2.at<double>(2,0)*y - P2.at<double>(1,0);
      A.at<double>(3,1) = P2.at<double>(2,1)*y - P2.at<double>(1,1);
      A.at<double>(3,2) = P2.at<double>(2,2)*y - P2.at<double>(1,2);
      A.at<double>(3,3) = P2.at<double>(2,3)*y - P2.at<double>(1,3);

      // now vertically concatenate with A_all[i]
      if (A_all[i].rows==0)
      {
        A_all[i] = A;
      }
      else
      {
        vconcat(A_all[i], A, A_all[i]);
      }
    }
    else
    {
      // no A can be updated since there was no association for this circle
    }

    // now use the A matrices to update the 3d position estimation X
    // this is the same as minimizing A*X which is a matter of finding the
    // right-singular vector associated with the smallest singular value
    // (the bottom row of vt)

    // generate the SVD (*.u, *.w, *.vt are the resulting object members)
    // and extract the bottom row of vt (same as right column of v)
    if (A_all[i].rows>0)
    {
      SVD svd_decomp(A_all[i]);
      Mat Xh = svd_decomp.vt.row(svd_decomp.vt.rows - 1);

      // Xh is the 3d position in homogenous coordinates
      //std::cout << Xh.rows << std::endl;
      //std::cout << Xh.cols << std::endl;

      //Mat X(1,3,CV_64FC1);
      //std::cout << X.rows << std::endl;
      //std::cout << X.cols << std::endl;

      X.at<double>(0,0) = Xh.at<double>(0,0)/Xh.at<double>(0,3);
      X.at<double>(0,1) = Xh.at<double>(0,1)/Xh.at<double>(0,3);
      X.at<double>(0,2) = Xh.at<double>(0,2)/Xh.at<double>(0,3);

      std::cout << "---" << std::endl;
      std::cout << i << std::endl;
      std::cout << X << std::endl;

      X_all[i] = X;
    }

  }

  // all A matrices have been updated for circles that were associated
  // corresponding position estimates have been updated as well






}




/*
void StructureFromMotion::scaleTranslation(Mat frame_in, std::vector<std::vector<Vec3f>> circles_all_u, std::vector<std::vector<int>> circles_hierarchy_all, int idx_circle)
{
  // find the highest rank id that is found in both current and original frames
  // circles_hierarchy_all[0]
  // circles_hierarchy_all[idx]

  int rank_common = 0;
  int idx_original = 500; // index of the chosen original circle
  int idx_current = 500; // index of the chosen current circle
  bool contin = true;
  while (contin)
  {
    for (int i=0; i<circles_hierarchy_all[0].size(); i++)
    {
      if (circles_hierarchy_all[0][i] == rank_common)
      {
        idx_original = i;
      }
    }
    for (int i=0; i<circles_hierarchy_all[idx].size(); i++)
    {
      if (circles_hierarchy_all[idx][i] == rank_common)
      {
        idx_current = i;
      }
    }

    if (idx_original==500 || idx_current==500)
    {
      // the suggested common rank wasn't found in both
      rank_common++;
      int idx_original = 500;
      int idx_current = 500;
    }
    else
    {
      contin = false;
      //std::cout << "found mutual rank" << std::endl;
      //std::cout << rank_common << std::endl;
    }

    if (rank_common > idx_circle)
    {
      contin = false;
      std::cout << "CIRCLE ASSOCIATION ERROR" << std::endl;
    }
  }

  // now idx_current and idx_original reprsent the indices of matching
  // circles from the beginning to current sets of circles

  // circles_all_u[0][idx_original]  < original circle
  // circles_all_u[inx][idx_current] < current circle

  // get P1 and P2 using stereoRectify
  Mat R1, R2, P1, P2, Q;
  stereoRectify(  intrinsic, distortion,
                  intrinsic, distortion,
                  frame_in.size(),  R,  T,
                  R1, R2, P1, P2, Q );






  //pts1 needs to be vector of Point2f containing the center and edge of original
  //pts2 needs to be vector of Point2f containing the center and edge of current

  Point2f center_o = Point2f(circles_all_u[0][idx_original][0], circles_all_u[0][idx_original][1]);
  Point2f edge_o   = Point2f(circles_all_u[0][idx_original][0]+circles_all_u[0][idx_original][2], circles_all_u[0][idx_original][1]);

  Point2f center_c = Point2f(circles_all_u[idx][idx_current][0], circles_all_u[idx][idx_current][1]);
  Point2f edge_c   = Point2f(circles_all_u[idx][idx_current][0]+circles_all_u[idx][idx_current][2], circles_all_u[idx][idx_current][1]);

  // the first number is dimension of points
  // the second number is the number of points
  //Mat pts_out(4,2,CV_64FC1);
  //Mat pts1(2,2,CV_64FC1);
  //Mat pts2(2,2,CV_64FC1);


  std::vector<Point2f> pts1, pts2;
  Mat pts_out;
  pts1.push_back(center_o);
  pts1.push_back(edge_o);
  pts2.push_back(center_c);
  pts2.push_back(edge_c);


  triangulatePoints(P1, P2, pts1, pts2, pts_out);

  // pts_out is 2x4 (4,2)
  // 2 columns and 4 rows
  //std::cout << pts_out.cols << std::endl;
  //std::cout << pts_out.rows << std::endl;

  std::vector<Vec4f> test;
  std::vector<Vec3f> test2;

  for (int i=0; i<pts_out.cols; i++)
  {
    double val1 = pts_out.at<double>(0,i);
    double val2 = pts_out.at<double>(1,i);
    double val3 = pts_out.at<double>(2,i);
    double val4 = pts_out.at<double>(3,i);
    test.push_back(Vec4f(val1,val2,val3,val4));
  }

  convertPointsFromHomogeneous(test, test2);

  std::cout << T << std::endl;
  std::cout << test2[0] << std::endl;

  // from notes on paper - store an A for each hierarchy and add to it
  // the svd solution to AX = 0 will always be the answer.


  //std::cout << pts_out2.size() << std::endl;
  // we can create mutual centerpoints and radial points (assuming level camera)
  // find the 3d locatons
  // then use the distance to resolve scale ambiguity




}
*/
