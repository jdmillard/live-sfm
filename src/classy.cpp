#include "classy.h"

using namespace cv;

myTestClass::myTestClass(int in_var)
{
  my_pri_val = in_var;
  my_pub_val = in_var;
}

int myTestClass::getVal()
{
  return my_pri_val;
}
