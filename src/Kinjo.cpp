#include <kinjo/arm/ArmFactory.hpp>

#include <opencv2/highgui/highgui.hpp>
//#include <cstdio>
#include <iostream>

const int slider_max = 50;

//not yet needed::eventHandler for slider(s)
void on_trackbar(int, void*){}

void jacoMove(std::shared_ptr<kinjo::arm::Arm> Arm){

	
	cv::Vec3f vector = cv::Vec3f(
		cv::getTrackbarPos("X", "kinjo"),
		cv::getTrackbarPos("Y", "kinjo"),
		cv::getTrackbarPos("Z", "kinjo")
	);
	std::printf("moving to %i,%i,%i ...\n",
		cv::getTrackbarPos("X", "kinjo"),
		cv::getTrackbarPos("Y", "kinjo"),
		cv::getTrackbarPos("Z", "kinjo")
		);
	if (Arm != nullptr){
		Arm->moveTo(vector);

	}
	
	std::printf("moving done. New \"exact\" Position: %f,%f,%f\n",
		Arm->getPosition()[0], 
		Arm->getPosition()[1], 
		Arm->getPosition()[2]
		);

}



int main(int argc, char* argv[]){
	
	int posX;
	int posY;
	int posZ;


	cv::namedWindow("kinjo", cv::WINDOW_AUTOSIZE);
	
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

	posX = (int)Arm->getPosition()[0];
	posY = (int)Arm->getPosition()[1];
	posZ = (int)Arm->getPosition()[2];
	
	cv::createTrackbar("X", "kinjo", &posX, slider_max, on_trackbar);
	cv::createTrackbar("Y", "kinjo", &posY, slider_max, on_trackbar);
	cv::createTrackbar("Z", "kinjo", &posZ, slider_max, on_trackbar);

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