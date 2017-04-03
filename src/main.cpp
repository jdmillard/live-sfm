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

  myTestClass myRenameClass(4);
  myTestClass myOtherClass(15);
  std::cout << myRenameClass.my_pub_val << std::endl;
  std::cout << myOtherClass.getVal() << std::endl;
  std::cout << "----" << std::endl;

  circleFinder nextTest(13);
  circleFinder nextTest2(14);
  std::cout << nextTest.my_pub_val << std::endl;
  std::cout << nextTest2.getVal() << std::endl;
  std::cout << "----" << std::endl;

  std::cout << "terminating" << std::endl;
}
