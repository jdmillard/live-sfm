#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>

#include "sfm.h"

// note about additional detectors
// decide convention for notes in .ccp or .h

class SphereDetector: public StructureFromMotion
{
	public:
		SphereDetector(int in_var);
	  int getVal();
	  int my_pub_val;
		void newFrame(cv::Mat frame_in);
		cv::Mat frame;

	private:
		int my_pri_val;

};
