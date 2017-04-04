#include "sfm.h"

using namespace cv;

void StructureFromMotion::setValy(int set_val_int)
{
  my_pri_valy = set_val_int;
  my_pub_valy = set_val_int;
}

int StructureFromMotion::getValy()
{
  return my_pri_valy;
}
