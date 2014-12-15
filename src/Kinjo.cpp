#include <kinjo/arm/ArmFactory.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <cstdio>

const int slider_max = 100;
int x;
int y;
int z;
bool armReady;

//not yet needed::eventHandler for slider(s)
void on_trackbar(int, void*){}

void jacoMove(std::shared_ptr<kinjo::arm::Arm> Arm){
	
	if (armReady){
		cv::Vec3f vector = cv::Vec3f(x, y, z);
		Arm->moveTo(vector);
		std::printf("moving to %i,%i,%i... done.\n", x, y, z);
		
	}
	else{
		std::printf("movement failed because arm not ready\n");
	}
}



int main(int argc, char* argv[]){
	
	x = 0;
	y = 0;
	z = 0;
	armReady = false;

	cv::namedWindow("kinjo", cv::WINDOW_AUTOSIZE);
	
	cv::createTrackbar("X", "kinjo", &x, slider_max, on_trackbar);
	cv::createTrackbar("Y", "kinjo", &y, slider_max, on_trackbar);
	cv::createTrackbar("Z", "kinjo", &z, slider_max, on_trackbar);
	std::shared_ptr<kinjo::arm::Arm> Arm;
	
	try
	{
		Arm = kinjo::arm::ArmFactory::getInstance();	
		armReady = true;
		//return 0;
    }
    catch(std::exception const & e)
    {
        std::cerr << e.what() << std::endl;
		armReady = false;
		//return 1;
    }
    catch(...)
    {
        std::cerr << "Unknown exception!" << std::endl;
		armReady = false;
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
				int i = 42;
				jacoMove(Arm);
			}
		}
	}

	return 0;
}