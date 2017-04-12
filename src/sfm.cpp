#include "sfm.h"

using namespace cv;


void StructureFromMotion::newFrame(Mat frame_in)
{
  // this method is to be overwritten by the derived class
  // it is supposed to perform logic based on the frame id to determine if
  // the current frame is a keyframe
  std::cout << "base class method needs to be overwritten!" << std::endl;
}
