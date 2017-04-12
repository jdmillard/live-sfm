#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>

class StructureFromMotion
{
	public:
		void newFrame(cv::Mat frame_in);

		bool init = false;
		int idx; // frame index
		int sep; // keyframe separation


	  int my_pub_val_base;

	private:
		int my_pri_val_base;

};
