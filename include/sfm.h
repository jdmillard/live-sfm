#include <opencv2/opencv.hpp>
#include <string> // needed for setting std::strings and using to_string
#include <fstream>

class StructureFromMotion
{
	public:
	  int getValy();
		void setValy(int set_val_int);
	  int my_pub_valy;

	private:
		int my_pri_valy;

};
