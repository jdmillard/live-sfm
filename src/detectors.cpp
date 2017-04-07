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
  my_pri_val = in_var;
  my_pub_val = in_var;
  setValy(in_var);
}

int SphereDetector::getVal()
{
  return my_pri_val;
}

void SphereDetector::newFrame(Mat frame_in)
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


  // turn the edges into a series of points
  std::vector<std::vector<Point>> contours, contours2;
  findContours(frame, contours, RETR_LIST, CHAIN_APPROX_NONE);


  // fit circles to the points
  std::vector<Vec3f> circles, circles2;
  circleFitter(contours, circles);


  // group similar circles' points together
  int minRadius = 30;
  int maxRadius = 80;
  groupCircles(circles, contours, contours2, minRadius, maxRadius);


  // derive circles with new point groups
  circleFitter(contours2, circles2);



  // TODO:
  // add the color variance check - see if it can be added early - before grouping to save computation
  // parameterize the center and radius thresholds for grouping - 40 and 40 below
  // parameterize the circleFitter thresholds
  // optimize
  // see what pre-processing can be cut out to save time





  /*

  // clean up circles by checking the color variance at each location
  //namedWindow("The Frame2", CV_WINDOW_AUTOSIZE);
  //moveWindow("The Frame2", 50, 50);

  std::vector<Vec3f> circles2;
  for (int i=0; i<circles.size(); i++)
  {
    double x = circles[i][0];
    double y = circles[i][1];
    double r = circles[i][2];
    double d = r/sqrt(2);

    // select the square inside the current circle
    Mat inside(frame_in, Rect(x-d, y-d, 2*d, 2*d));

    // NOTE: need logic to prevent roi occuring outside frame
    // need a clean way to make it an automatic disqualification
    // consider making the circle cleanup a dedicated method
    // further, in the sphere initialization, add roi logic to circles perhaps?
    // test performance and timing, get lowres video from webcam in demo room
    // idea: create a mode that allows for circle tuning?

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

    // there will be misses and false positives... the sfm must accomodate this

    if ((min1 > rbg_thresh) && (max2 < stddev_thresh) && (stddev2.at<double>(0,0) < stddev2_thresh))
    {
      // NOTE: cleanup variable names, comments, and overall structure
      //std::cout << "SPHERE SPHERE SPHERE SPHERE SPHERE" << std::endl;
      circles2.push_back(circles[i]);
    }

    // display the frame
    //imshow("The Frame2", inside);


    // allow user to cycle through frames individually
    int key = waitKey();
    if (key == 110) {
      // the 'n' (next) key was pressed
    } else if (key == 27) {
      // the 'esc' key was pressed, end application
    }


  }


  */


  // currently the circles that appear white are the alebraic
  // circles that appear blue are after the geometric

  //drawCircles(frame, circles);
  cvtColor(frame, frame, CV_GRAY2BGR);
  drawCircles(frame_in, circles2);


  //frame = frame_in;

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
    while ( j<50 && error > 20)
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

        if (i!=j && c_dist < 40 && r_dist < 40)
        {
          // then add to current batch of points
          // contours2[contours2.size()-1] // is now the most recent vector of points
          // and set the radius so high the entry won't be considered again

          // a.insert(a.end(), b.begin(), b.end());
          int idx = contours2.size()-1;
          contours2[idx].insert(contours2[idx].end(), contours[j].begin(), contours[j].end());

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
