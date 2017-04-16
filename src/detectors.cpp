#include "detectors.h"

using namespace cv;

/*
The class SphereDetector is the visual front end that provides the pixel locations
of spheres in the given frame. This can be replaced by any other class so long
as it generates information for the structureFromMotion object in a compatible
format.
*/

// perhaps this class is passed to the sfm object upon initialization
// I would need to look into the best way to do this, perhaps inheritance

SphereDetector::SphereDetector(int in_var)
{
  sep = in_var;

  // any other initialization steps?

}


void SphereDetector::newFrame(Mat frame_in)
{
  if (!init)
  {
    // algorithm has not yet been initialized, look to see if spheres are found
    // in the middle quarter of the image, if not keep looking

    //std::vector<Vec3f> circles;
    detectSpheres(frame_in);

    int inside = 0;
    int left = frame_in.cols/2 - frame_in.cols/4;
    int right = frame_in.cols/2 + frame_in.cols/4;
    int top = frame_in.rows/2 - frame_in.rows/4;
    int bottom = frame_in.rows/2 + frame_in.rows/4;

    for (int i=0; i<circles2.size(); i++)
    {
      if (circles2[i][0] > left && circles2[i][0] < right && circles2[i][1] > top && circles2[i][1] < bottom)
      {
        inside++;
      }
    }

    if (inside > 2)
    {
      init = true;
      std::cout << "spheres found, algorithm initialized" << std::endl;
    }
    else
    {
      std::cout << "searching for spheres in image center" << std::endl;
    }
  }
  else
  {
    // algorithm has been initialized; run normal course

    // track features
    featureTracker(frame_in);

    // detect spheres
    detectSpheres(frame_in);

    // associate circles
    circlesHierarchy(frame_in);

    // resolve scale, knowing the expected diameter of spheres
    if (idx>3)
    {
      // get rotation and translation
      getRotationTranslation();
      // perform triangulation
      triangulatePointsCustom(frame_in, idx_circle, circles_all_u, circles_hierarchy_all);
      //scaleTranslation(frame_in, circles_all_u, circles_hierarchy_all, idx_circle);
      drawStatus(frame_in);
    }

    // optional
    drawFeatures(frame_in, features_old);
    drawCircles(frame_in, circles2);

    if (features_old.size()<100)
    {
      // set algorithm to finish
      std::cout << "not enough features" << std::endl;
      more = false;
    }


    // get P1, P2 using stereorectify
    // triangulate points to get 3d positions in homogenous coordinates
    // convertPointsFromHomogeneous

    // https://stackoverflow.com/questions/31431047/3d-reconstruction-from-two-calibrated-cameras-where-is-the-error-in-this-pipel
    // http://www.morethantechnical.com/2012/01/04/simple-triangulation-with-opencv-from-harley-zisserman-w-code/




    // (all this needs to consider false and missing measurements)
    // some type of data association?
    // idea for this: first order circles by distance to center
    // then use nearest neighbor to associate across frames
    // only find 3d points from a to b for circles that both registered a measurement
    // only display 3d points that have a threshold of number of measurements


  }


}


void SphereDetector::detectSpheres(Mat frame_in)
{

  // perform image processing required to locate spheres in the current frame

  // convert to grayscale
  cvtColor(frame_in, frame, CV_BGR2GRAY);


  // blur the image to soften background
  int blur_size = 15;     // size of the blur kernel
  int sigma = 5;          // variance
  GaussianBlur(frame, frame, Size(blur_size, blur_size), sigma, sigma);


  // detect the edges
  double thresh1 = 80;    // if pixel gradient is higher than this, accepted
  double thresh2 = 40;    // if pixel gradient is lower than this, rejected
  Canny(frame, frame, thresh1, thresh2);


  // make detected edges thicker to make contours more continuous
	int morph_size = 2;
  Mat element = getStructuringElement(MORPH_RECT,
                                      Size(2*morph_size + 1, 2*morph_size+1),
                                      Point(morph_size, morph_size));
  dilate(frame, frame, element);


  // turn the edges into many series of points
  std::vector<std::vector<Point>> contours, contours2;
  findContours(frame, contours, RETR_LIST, CHAIN_APPROX_NONE);


  // fit circles to the points
  circles.clear();
  circles2.clear();
  circleFitter(contours, circles);


  // check the color variance
  colorVariance(frame_in, circles);


  // group similar circles' points together
  int minRadius = 15;
  int maxRadius = 40;
  groupCircles(circles, contours, contours2, minRadius, maxRadius);

  // derive circles with new point groups
  circleFitter(contours2, circles2);

  // another variance check?


  // undistort the circle center points
  if (init)
  {
    // generate a vector of the points
    std::vector<Point2f> centers;
    for (int i=0; i<circles2.size(); i++)
    {
      centers.push_back(Point2f(circles2[i][0], circles2[i][1]));
    }

    undistortPoints(centers, centers, intrinsic, distortion, noArray(), intrinsic);

    // now generate an undistorted vector of circles
    circles_u.clear();
    for (int i=0; i<centers.size(); i++)
    {
      circles_u.push_back(Vec3f(centers[i].x, centers[i].y, circles2[i][2]));
    }

    // save the undistorted circles
    circles_all_u.push_back(circles_u);

  }


  // TODO:
  // parameterize the center and radius thresholds for grouping - 40 and 40 below
  // parameterize the circleFitter thresholds
  // optimize
  // see what pre-processing can be cut out to save time

}





void SphereDetector::drawCircles(Mat img, std::vector<Vec3f>& circles)
{
  // the purpose of this method is to easily draw circles wherever detected

  Scalar color = Scalar(255, 0, 0);
  int thickness = 2;
  // cycle through each known circle
  for (int i=0; i<circles.size(); i++)
  {

    if (circles[i][0] > 0 && circles[i][0] < frame.cols &&
        circles[i][1] > 0 && circles[i][1] < frame.rows)
    {
      Point2f center = Point2f(circles[i][0],circles[i][1]);
      circle(img, center, circles[i][2], color, thickness );
    }
  }
}





void SphereDetector::drawStatus(Mat img)
{
  // the purpose of this method is to easily draw circles wherever detected

  Scalar color = Scalar(255, 0, 0);
  int thickness = 2;
  // cycle through each
  for (int i=0; i<X_all.size(); i++)
  {
    // i is the current rank

    // find the index of the current set of circles that matches this
    int idx_current = 500;
    for (int j=0; j<circles_hierarchy.size(); j++)
    {
      if (circles_hierarchy[j]==i)
      {
        idx_current = j;
      }
    }

    // idx_current is the index of circles_u that corresponds with i
    // use these coordinates to putText

    String num, x_str, y_str, z_str;
    Point2f place;
    num = std::to_string(i);
    x_str = "x="+std::to_string(X_all[i].at<double>(0,0)).substr(0,4);
    y_str = "y="+std::to_string(X_all[i].at<double>(0,1)).substr(0,4);
    z_str = "z="+std::to_string(X_all[i].at<double>(0,2)).substr(0,4);
    place.x = circles_u[idx_current][0];
    place.y = circles_u[idx_current][1];
    putText(img, num, place, FONT_HERSHEY_SIMPLEX, 0.7, color, thickness);
    int gap = 40;
    int spread = 30;
    double mag = 1;
    place.x = circles_u[idx_current][0]+gap;
    place.y = circles_u[idx_current][1]-gap;
    putText(img, x_str, place, FONT_HERSHEY_SIMPLEX, mag, color, thickness);
    place.y = place.y + spread;
    putText(img, y_str, place, FONT_HERSHEY_SIMPLEX, mag, color, thickness);
    place.y = place.y + spread;
    putText(img, z_str, place, FONT_HERSHEY_SIMPLEX, mag, color, thickness);


  }
}





void SphereDetector::circleFitter(std::vector<std::vector<Point>>& contours, std::vector<Vec3f>& circles)
{
  // the purpose of this method is to robustly and quickly parameterize
  // circles when given groups of points

  for (int i=0; i<contours.size(); i++)
  {
    // contours[i] is the current set of points
    // NOTE: do a check for >= 3 elements
    // first minimize the algebraic distance, this does not provide a clean,
    // accurate circle, but it provides a ballpark estimate to start with

    // populate B, which is data to be multiplied by the circle parameters
    Mat B =   Mat(contours[i].size(), 4, CV_64F);
    for (int k=0; k<contours[i].size(); k++)
    {
      // for each point, generate the corresponding jacobian entries
      double x1 = contours[i][k].x;
      double x2 = contours[i][k].y;

      B.at<double>(k, 0) = pow(x1, 2) + pow(x2, 2);
      B.at<double>(k, 1) = x1;
      B.at<double>(k, 2) = x2;
      B.at<double>(k, 3) = 1;
    }

    // now we want to solve for the parameters that minimize B*p
    // minimizing the algebraic error is a matter of finding the right-singular
    // vector associated with the smallest singular value: the bottom row of vt

    // generate the SVD (*.u, *.w, *.vt are the resulting object members)
    // and extract the bottom row of vt (same as right column of v)
    SVD svd_decomp(B);
    Mat p = svd_decomp.vt.row(svd_decomp.vt.rows - 1);

    // extract parameters, then reparameterize using coordinates and radius
    double a  = p.at<double>(0, 0);
    double b1 = p.at<double>(0, 1);
    double b2 = p.at<double>(0, 2);
    double c  = p.at<double>(0, 3);

    double x_0 = b1 / (-2 * a);
    double y_0 = b2 / (-2 * a);
    double r_0 = sqrt( pow(x_0, 2) + pow(y_0, 2) - c/a );

    // store the initial results
    //circles.push_back(Vec3f(x_0, y_0, r_0));

    // now do gauss-newton to refine circle parameters
    // now do gauss-newton to refine circle parameters
    // now do gauss-newton to refine circle parameters
    // now do gauss-newton to refine circle parameters

    // minimizing the squared error (the difference between global radius and
    // the distance from the center to a given point) is a matter of solving
    // a nonlinear least squares which needs to be done in steps using
    // gauss-newton optimization

    Mat u =   Mat(3, 1, CV_64F);                    // parameters
    Mat J =   Mat(contours[i].size(), 3, CV_64F);   // jacobian
    Mat res = Mat(contours[i].size(), 1, CV_64F);   // residuals

    // initialize the u vector with the algebraic distance minimization results
    u.at<double>(0, 0) = x_0;              // center x guess
    u.at<double>(1, 0) = y_0;              // center y guess
    u.at<double>(2, 0) = r_0;              // radius guess

    // alternative method - tends to yield bad results:
    //u.at<double>(0, 0) = contours[i][0].x +1; // center x (arbitrary)
    //u.at<double>(1, 0) = contours[i][0].y +1; // center y (arbitrary)
    //u.at<double>(2, 0) = 2;                   // radius   (arbitrary)

    int j = 0;
    double error = 1000;
    while ( j<10 && error > 20)
    {
      error = 0;
      // run for 50 iterations or until the squared error per point reaches
      // a desired threshold, whichever comes first.
      // https://www.emis.de/journals/BBMS/Bulletin/sup962/gander.pdf
      // https://en.wikipedia.org/wiki/Gauss%E2%80%93Newton_algorithm

      // populate J and residual based on each point
      for (int k=0; k<contours[i].size(); k++)
      {
        // for each point, generate the corresponding jacobian entries
        double u1 = u.at<double>(0,0);
        double u2 = u.at<double>(1,0);
        double x1 = contours[i][k].x;
        double x2 = contours[i][k].y;

        // calculate the distance to make math easier
        double dist = sqrt( pow((u1-x1),2) + pow((u2-x2),2) );

        J.at<double>(k, 0) = (u1 - x1)/dist;
        J.at<double>(k, 1) = (u2 - x2)/dist;
        J.at<double>(k, 2) = -1;

        // populate the residual - difference between distance and radius
        res.at<double>(k,0) = dist - u.at<double>(2,0); // residual
        error += pow(dist - u.at<double>(2,0), 2);
      }

      // find the pseudo inverse
      Mat J_inv = (J.t()*J).inv()*J.t();

      // gauss-newton step forward
      u = u - J_inv * res;

      // now have new u, repeat
      error = error/contours[i].size();
      j++;
    } // end of gauss newton loop

    // current section of points have been fitted, save associated center, r
    // manage here with circles vector of vectors

    circles.push_back(Vec3f(u.at<double>(0,0),
                            u.at<double>(1,0),
                            u.at<double>(2,0)));
  } // end of cycling through all contours
}





void SphereDetector::groupCircles(std::vector<Vec3f>& circles, std::vector<std::vector<Point>>& contours, std::vector<std::vector<Point>>& contours2, int minRadius, int maxRadius)
{
  // the purpose of this method is to group sets of points together that
  // share centerpoints and radii with in a tolerance

  //std::cout << circles.size() << std::endl;
  //std::cout << contours.size() << std::endl;

  for (int i = 0; i<circles.size(); i++)
  {
    // contours[i] is the current set of points
    // circles[i] is the current set of parameters
    // and they correspond to eachother


    // check to see that current circle is in the image and radii bounds
    if (circles[i][0] > 0 && circles[i][0] < frame.cols &&
        circles[i][1] > 0 && circles[i][1] < frame.rows &&
        circles[i][2] > minRadius && circles[i][2] < maxRadius)
    {
      // store the corresponding points;
      contours2.push_back(contours[i]);

      // cycle through the rest of the entries
      for (int j = 0; j<circles.size(); j++)
      {
        // check criteria
        double x_diff = circles[i][0] - circles[j][0];
        double y_diff = circles[i][1] - circles[j][1];
        double c_dist = sqrt( pow(x_diff, 2) + pow(y_diff, 2));
        double r_dist = circles[i][2] - circles[j][2];

        if (i!=j && c_dist < 20 && r_dist < 20)
        {
          // then add to current batch of points
          // contours2[contours2.size()-1] // is now the most recent vector of points
          // and set the radius so high the entry won't be considered again

          // a.insert(a.end(), b.begin(), b.end());
          int idx2 = contours2.size()-1;
          contours2[idx2].insert(contours2[idx2].end(), contours[j].begin(), contours[j].end());

          // set circle as visited
          circles[j][2] = 500;
        }
      }

      // mark the current circle as visited
      circles[i][2] = 500;
    }
    else
    {
      // current image does not satisfy bounds or was already considered
      circles[i][2] = 500;
    } // end of initial check
  } // end of main for loop
} // end of method





void SphereDetector::colorVariance(Mat frame_in, std::vector<Vec3f>& circles)
{
  // the purpose of this method is to check the color variance of the inside
  // of each circle to rule out false circles

  // clean up circles by checking the color variance at each location
  //namedWindow("The Frame2", CV_WINDOW_AUTOSIZE);
  //moveWindow("The Frame2", 50, 50);

  for (int i=0; i<circles.size(); i++)
  {
    double x = circles[i][0];
    double y = circles[i][1];
    double r = circles[i][2];
    double d = r/sqrt(2);


    if (x-d < 0 || x+d > frame_in.cols || y-d < 0 || y+d > frame_in.rows)
    {
      // circle lives outside the frame, disqualify the circle
      circles[i][2] = 500;
    }
    else
    {
      // circle position is good, perform variance check

      // select the square inside the current circle
      Mat inside(frame_in, Rect(x-d, y-d, 2*d, 2*d));

      // look at statistics of the subimage
      Mat mean, stddev;
      meanStdDev(inside, mean, stddev);

      // check statistics of each B, G, R
      // mean of each needs to be > 170
      // standard deviation of each needs to be < 55

      Mat mean2, stddev2;
      meanStdDev(stddev, mean2, stddev2);

      //std::cout << "---" << std::endl;
      //std::cout << "mean" << std::endl;
      //std::cout << mean << std::endl;
      //std::cout << "stddev" << std::endl;
      //std::cout << stddev << std::endl;
      //std::cout << stddev2.at<double>(0,0) << std::endl;

      double min1, max1;
      minMaxLoc(mean, &min1, &max1);

      double min2, max2;
      minMaxLoc(stddev, &min2, &max2);

      int rbg_thresh    = 170; // all rgb averages must be higher than this
      int stddev_thresh = 60;  // all rbg stddevs must be lower than this
      int stddev2_thresh= 3;   // stddev of stddev must be lower than this

      if ((min1 > rbg_thresh) && (max2 < stddev_thresh) && (stddev2.at<double>(0,0) < stddev2_thresh))
      {
        // NOTE: cleanup variable names, comments, and overall structure
        //std::cout << "SPHERE SPHERE SPHERE SPHERE SPHERE" << std::endl;
        //std::cout << "good circle" << std::endl;
      }
      else
      {
        // disqualify the current circle
        circles[i][2] = 500;
        //std::cout << "bad circle" << std::endl;
      }

      // display the frame
      //imshow("The Frame2", inside);

      /*
      // allow user to cycle through frames individually
      int key = waitKey();
      if (key == 110) {
        // the 'n' (next) key was pressed
      } else if (key == 27) {
        // the 'esc' key was pressed, end application
      }
      */

    } // end of circle edge check

  } // end of loop through circles





} // end of method





void SphereDetector::circlesHierarchy(Mat frame_in)
{
  // reset hierarchy
  circles_hierarchy.clear();
  circles_hierarchy.resize(circles_u.size());

  // set them all to 500
  for (int i=0; i<circles_hierarchy.size(); i++)
  {
    circles_hierarchy[i] = 500;
  }

  // if this is the first iteration, establish circle id by proximity to center
  if (idx ==0)
  {

    double dist_floor = 0;
    for (int j=0; j<circles_u.size(); j++)
    {
      // cycle through circles_u, putting values in circles_hierarchy vector
      double dist_min = 1000;
      int    dist_idx = 1000;
      for (int i=0; i<circles_u.size(); i++)
      {
        // find distance to center of image
        double x_term = pow(circles_u[i][0]-(frame_in.cols/2), 2);
        double y_term = pow(circles_u[i][1]-(frame_in.rows/2), 2);
        double dist = pow((x_term + y_term), 0.5);
        if (dist < dist_min && dist > dist_floor)
        {
          dist_min = dist;
          dist_idx = i;
        }
      }

      // now dist_min is the smallest distance and dist_idx is the index
      // associated with this circle

      circles_hierarchy[dist_idx] = idx_circle;
      idx_circle++;
      dist_floor = dist_min;
    }

    // now circles_hierarchy corresponds to circles_u
    circles_hierarchy_all.push_back(circles_hierarchy);


  }
  else
  {
    int window = std::min(idx, 8);
    // not the first frame
    // attempt to associate using undistorted nearest neighbor
    // using the last "int window" frames

    // cycle through known circle ids
    for (int i=0; i<idx_circle; i++)
    {
      // "i" is the current circle index
      // now find all the center locations associated with this index


      std::vector<Point2f> locations;
      locations.clear();
      // for this id, look at past circles for the last "int window" frames
      for (int j=1; j<=window; j++)
      {
        int found = 100;
        // circles_all_u[idx-j] is the current set of circles at hand
        // circles_hierarchy_all[idx-j] is the current set of hierarchies
        // find the index where "i" is found in circles_hierarchy_all[idx-j]
        for (int k=0; k<circles_hierarchy_all[idx-j].size(); k++)
        {
          // we care about when circles_hierarchy_all[idx-j][k]=i
          if (circles_hierarchy_all[idx-j][k]==i)
          {
            found = k;
          }
        }

        if (found!=100)
        {
          // found is now the index such that circles_all_u[idx-j][found]
          double xx = circles_all_u[idx-j][found][0];
          double yy = circles_all_u[idx-j][found][1];
          locations.push_back(Point2f(xx,yy));
        }
      }

      // now locations contains all the centers for the current circle index
      // average them

      if (locations.size() > 0)
      {
        double x_sum = 0;
        double y_sum = 0;
        for (int j=0; j<locations.size(); j++)
        {
          x_sum += locations[j].x;
          y_sum += locations[j].y;
        }
        x_sum = x_sum/locations.size();
        y_sum = y_sum/locations.size();

        // x_sum and y_sum are the average location of the considered circle id
        // now find the nearest neighbor of the current circles
        // cycle through circles_u
        double dist_min = 1000;
        int    dist_idx = 1000;
        for (int j=0; j<circles_u.size(); j++)
        {
          double x_term = pow(circles_u[j][0]-x_sum, 2);
          double y_term = pow(circles_u[j][1]-y_sum, 2);
          double dist = pow((x_term + y_term), 0.5);
          if (dist < dist_min)
          {
            dist_min = dist;
            dist_idx = j;
          }
        }
        //std::cout << dist_min << std::endl;

        // dist_idx is the index of the recent circles that is closest to the
        // current circle id and dist_min is the distance

        double dist_thresh = 20;
        if (dist_min < dist_thresh)
        {
          circles_hierarchy[dist_idx] = i;
        }
      }



    }

    // it's possible that not all current circles were associated
    for (int i=0; i<circles_hierarchy.size(); i++)
    {
      if (circles_hierarchy[i] == 500)
      {
        circles_hierarchy[i] = idx_circle;
        idx_circle++;
      }
    }

    // now circles_hierarchy is populated
    circles_hierarchy_all.push_back(circles_hierarchy);

    // there will be management method that looks at 3d locations and merges

    // what about circles that are added along the way?



  }
  //std::cout << "---" << std::endl;
  //for (int jj=0; jj<circles_hierarchy.size(); jj++)
  //{
  //  std::cout << circles_hierarchy[jj] << std::endl;
  //}
  //std::cout << "-" << std::endl;
  //std::cout << idx_circle << std::endl;
}
