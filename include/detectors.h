#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>
#include <math.h>

#include "sfm.h"

// note about additional detectors
// decide convention for notes in .ccp or .h

class SphereDetector: public StructureFromMotion
{
	public:
		SphereDetector(int in_var);
		void newFrame(cv::Mat frame_in);
		void detectSpheres(cv::Mat frame_in);
		void drawCircles(cv::Mat img, std::vector<cv::Vec3f>& circles);
		void circleFitter(std::vector<std::vector<cv::Point>>& contours, std::vector<cv::Vec3f>& circles);
		void groupCircles(std::vector<cv::Vec3f>& circles, std::vector<std::vector<cv::Point>>& contours, std::vector<std::vector<cv::Point>>& contours2, int minRadius, int maxRadius);
		void colorVariance(cv::Mat frame_in, std::vector<cv::Vec3f>& circles);
    void circlesHierarchy(cv::Mat frame_in);
		cv::Mat frame, frame_exp;

    int idx_circle = 0;

	  std::vector<cv::Vec3f> circles;
		std::vector<cv::Vec3f> circles2;
		std::vector<cv::Vec3f> circles_u;
    std::vector<int>       circles_hierarchy;

    std::vector<std::vector<int>        circles_hierarchy_all;
		std::vector<std::vector<cv::Vec3f>> circles_all_u;

	  int my_pub_val_derived;

	private:
		int my_pri_val_derived;

};
