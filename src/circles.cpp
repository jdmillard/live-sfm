#include "circles.h"

using namespace cv;

circleFinder::circleFinder(int in_var)
{
  my_pri_val = in_var;
  my_pub_val = in_var;
}

int circleFinder::getVal()
{
  return my_pri_val;
}
