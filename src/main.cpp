#include "main.h"

using namespace cv;



int main(int argc, char **argv)
{
  Mat img_a, img_b;


  // here construct the logic for the in-comming video
  // initialize the sfm object
  // new frames are passed to the sfm object (real-time logic?)
  // inside the object, the processing occurs

  // consider having 2 classes: 1 for processing and the other for sfm
  // leaning towards hough circles



  // create the video object
  bool from_recorded = true;
  VideoCapture video;
  if (from_recorded)
  {
    // use frames from a pre-recorded video
    std::string filename = "../images/vid.mp4";
    std::cout << filename << std::endl;
    video.open(filename);
  }
  else
  {
    // use frames from a local webcam
    video.open(0);
  }

  namedWindow("The Frame", CV_WINDOW_AUTOSIZE);
  moveWindow("The Frame", 50, 50);


  SphereDetector nextTest(13);

  std::cout << nextTest.my_pub_val << std::endl;
  std::cout << nextTest.my_pub_valy << std::endl;
  std::cout << nextTest.getVal() << std::endl;
  std::cout << nextTest.getValy() << std::endl;


  Mat frame;

  while (true)
  {
    video >> frame;
    std::cout << frame.size() << std::endl;

    imshow("The Frame", frame);

    int key = waitKey();
    if (key == 110) {
      // the 'n' (next) key was pressed
    } else if (key == 27) {
      // the 'esc' key was pressed, end application
      std::cout << "terminating" << std::endl;
      return -1;
    }




  }


  std::cout << "terminating" << std::endl;
}
