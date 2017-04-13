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
    std::string filename = "../images/vid2.webm";
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

  // the value going in is the frame separation between keyframes
  SphereDetector sfm(5);


  Mat frame;

  while (true)
  {
    // update frame and check that it isn't empty
    video >> frame;
    if (frame.empty())
    {
      break;
    }

    sfm.newFrame(frame);

    // display the frame
    imshow("The Frame", frame);

    // allow user to cycle through frames individually
    int key = waitKey();
    if (key == 110) {
      // the 'n' (next) key was pressed
    } else if (key == 27) {
      // the 'esc' key was pressed, end application
      std::cout << "terminating" << std::endl;
      return -1;
    }




  } // end of while loop for video
  std::cout << "terminating" << std::endl;
} // end of main
