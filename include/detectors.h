#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>

#include "sfm.h"

class SphereDetector: public StructureFromMotion
{
	public:
		SphereDetector(int in_var);
	  int getVal();
	  int my_pub_val;

	private:
		int my_pri_val;

};
