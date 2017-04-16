#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>

class StructureFromMotion
{
	public:
		void newFrame(cv::Mat frame_in);
		void featureTracker(cv::Mat frame_in);
		void drawFeatures(cv::Mat img, std::vector<cv::Point2f>& features);
		void loadCalibration();
		void cleanFeatures();
		void getRotationTranslation();
    void triangulatePointsCustom(cv::Mat frame_in, int idx_circle, std::vector<std::vector<cv::Vec3f>> circles_all_u, std::vector<std::vector<int>> circles_hierarchy_all);
    //void scaleTranslation(cv::Mat frame_in, std::vector<std::vector<cv::Vec3f>> circles_all_u, std::vector<std::vector<int>> circles_hierarchy_all, int idx_circle);
		cv::Mat intrinsic, distortion;

		bool init = false;
		bool edge = false;
		bool more = true;

		int idx; // frame index
		int sep; // keyframe separation

		std::vector<uchar> 										features_mask;
		std::vector<cv::Point2f>							features_old;
		std::vector<cv::Point2f>							features_new;
		std::vector<cv::Point2f>							features_new_u;
		std::vector<std::vector<cv::Point2f>> features_all;
		std::vector<std::vector<cv::Point2f>> features_all_u;

		cv::Mat 							R;
		cv::Mat 							T;
    cv::Mat               X = cv::Mat(1,3,CV_64FC1);

    std::vector<cv::Mat> A_all;
    std::vector<cv::Mat> X_all; // doesn't carry a history, this "all" is just the current collection

		cv::Mat frame_gray_old;


	  int my_pub_val_base;

	private:
		int my_pri_val_base;

};
