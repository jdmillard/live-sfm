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
  Mat element = getStructuringElement(MORPH_RECT, Size(2*morph_size + 1, 2*morph_size+1), Point(morph_size, morph_size));
  morphologyEx(frame, frame, MORPH_OPEN, element );

  // detect the edges
  double thresh1 = 80;    // hysteresis threshold 1, low as possible
  double thresh2 = 80;    // hysteresis threshold 2, low as possible
  Canny(frame, frame, thresh1, thresh2);

  // at this point, clean edges are expected around the circles

  // blur the resulting white lines in order to find circles easier
  int blur_size2 = 35;    // size of the blur kernel
  int sigma2 = 5;         // variance
  GaussianBlur(frame, frame, Size(blur_size2, blur_size2), sigma2, sigma2);

  // perform HoughCircles on the edges
  std::vector<Vec3f> circles;
  double dp = 2;
  double minDist = 30;
  double param1 = 50;
  double param2 = 140;
  int minRadius = 0;
  int maxRadius = 110;
  HoughCircles(frame, circles, CV_HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);


  cvtColor(frame, frame, CV_GRAY2BGR);

  drawCircles(frame, circles);
  //frame = frame_in;

  //frame = frame_in;

}

void SphereDetector::drawCircles(Mat img, std::vector<Vec3f>& circles)
{
  std::cout << "---" << std::endl;
  for (int i=0; i<circles.size(); i++)
  {
    std::cout << "worked" << std::endl;
    circle(img, Point2f(circles[i][0],circles[i][1]), circles[i][2],  Scalar(0, 255, 0), 2 );
  }
  //std::cout << "worked" << std::endl;
}
