#include <kinjo/arm/ArmFactory.hpp>

#include <opencv2/highgui/highgui.hpp>
//#include <cstdio>
#include <iostream>

const int slider_max = 100;

//not yet needed::eventHandler for slider(s)
void on_trackbar(int, void*){}

void jacoMove(std::shared_ptr<kinjo::arm::Arm> Arm){

	
	cv::Vec3f vector = cv::Vec3f(
		cv::getTrackbarPos("X", "kinjo"),
		cv::getTrackbarPos("Y", "kinjo"),
		cv::getTrackbarPos("Z", "kinjo")
	);
	if (Arm->initialized){
		Arm->moveTo(vector);
		std::printf("moving to %i,%i,%i is done done.\n",
			cv::getTrackbarPos("X", "kinjo"),
			cv::getTrackbarPos("Y", "kinjo"),
			cv::getTrackbarPos("Z", "kinjo")
			);
	}
	else
	{
		std::printf("Arm not Connected");
	}

}



int main(int argc, char* argv[]){
	
	cv::namedWindow("kinjo", cv::WINDOW_AUTOSIZE);
	
	cv::createTrackbar("X", "kinjo", 0, slider_max, on_trackbar);
	cv::createTrackbar("Y", "kinjo", 0, slider_max, on_trackbar);
	cv::createTrackbar("Z", "kinjo", 0, slider_max, on_trackbar);
	std::shared_ptr<kinjo::arm::Arm> Arm;
	
	try
	{
		Arm = kinjo::arm::ArmFactory::getInstance();	
		//return 0;
    }
    catch(std::exception const & e)
    {
        std::cerr << e.what() << std::endl;
		//return 1;
    }
    catch(...)
    {
        std::cerr << "Unknown exception!" << std::endl;
		//return 1;
    }
	
	
	//press esc to close window
	//press space to move the jacoArm
	for (;;){
		int i = cv::waitKey(10);
		if (i >= 0){
			if (i == 27){
				cv::destroyAllWindows();
				return 1;
			}
			if (i == 32){
				jacoMove(Arm);
			}
		}
	}

	return 0;
}