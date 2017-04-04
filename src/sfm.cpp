#include "sfm.h"

using namespace cv;

void myTestClass::setValy(int set_val_int)
{
  my_pri_valy = set_val_int;
  my_pub_valy = set_val_int;
}

int myTestClass::getValy()
{
  return my_pri_valy;
}
