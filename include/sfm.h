#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>

class StructureFromMotion
{
	public:
		void newFrame(cv::Mat frame_in);
		void featureTracker(cv::Mat frame_in);
		void drawFeatures(cv::Mat img, std::vector<cv::Point2f>& features);

		bool init = false;
		int idx; // frame index
		int sep; // keyframe separation

		std::vector<cv::Point2f>							features_cur;
		std::vector<std::vector<cv::Point2f>> features_all;

		cv::Mat frame_gray_old;


	  int my_pub_val_base;

	private:
		int my_pri_val_base;

};
