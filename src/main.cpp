#include "main.h"

using namespace cv;



int main(int argc, char **argv)
{
  Mat img_a, img_b;

  myTestClass myRenameClass(4);
  myTestClass myOtherClass(15);
  std::cout << myRenameClass.my_pub_val << std::endl;
  std::cout << myOtherClass.getVal() << std::endl;

  std::cout << "terminating" << std::endl;
}
