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

	// perform open and close to clean up the foreground
	int morph_size = 4;     // high number makes circles boxy
  Mat element = getStructuringElement(MORPH_RECT,
                                      Size(2*morph_size + 1, 2*morph_size+1),
                                      Point(morph_size, morph_size));
  morphologyEx(frame, frame, MORPH_OPEN, element );

  // detect the edges
  double thresh1 = 80;    // hysteresis threshold 1, low as possible
  double thresh2 = 80;    // hysteresis threshold 2, low as possible
  Canny(frame, frame, thresh1, thresh2);

  // at this point, clean edges are expected around the circles

  // blur the resulting white lines in order to find circles easier
  int blur_size2 = 15;    // size of the blur kernel
  int sigma2 = 3;         // variance
  GaussianBlur(frame, frame, Size(blur_size2, blur_size2), sigma2, sigma2);

  // perform HoughCircles on the edges
  std::vector<Vec3f> circles;
  double dp = 2;          // inverse ratio of resolutions
  double minDist = 40;    // minimum distance between circle centers (relaxed)
  double param1 = 80;     // hysteresis thresholds for canny
  double param2 = 60;     // smaller means more false circles detected
  int minRadius = 20;     // minimum radius (relaxed)
  int maxRadius = 100;    // minimum radius (relaxed)
  HoughCircles(frame, circles, CV_HOUGH_GRADIENT,
               dp, minDist, param1, param2,
               minRadius, maxRadius);



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

    /*
    // allow user to cycle through frames individually
    int key = waitKey();
    if (key == 110) {
      // the 'n' (next) key was pressed
    } else if (key == 27) {
      // the 'esc' key was pressed, end application
    }
    */

  }






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
    Point2f center = Point2f(circles[i][0],circles[i][1]);
    circle(img, center, circles[i][2], color, thickness );
  }
  //std::cout << "worked" << std::endl;
}
