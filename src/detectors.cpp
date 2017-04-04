#include "detectors.h"

using namespace cv;

/*
The class circleFinder is the visual front end that provides the pixel locations
of spheres in the given frame. This can be replaced by any other class so long
as it generates information for the structureFromMotion object in a compatible
format.
*/

// perhaps this class is passed to the sfm object upon initialization
// I would need to look into the best way to do this, perhaps inheritance

circleFinder::circleFinder(int in_var)
{
  my_pri_val = in_var;
  my_pub_val = in_var;
}

int circleFinder::getVal()
{
  return my_pri_val;
}
