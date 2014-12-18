#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/vision/OpenNiVision.hpp>

#include <kinjo/recognition/Recognition.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

static std::string const g_sWindowTitleDepth("Vision (Depth)");
static std::string const g_sWindowTitleColor("Vision (RGB)");
static std::string const g_sWindowTitleArm("Arm");
static const int g_iRefreshIntervallMs(100);


void jacoMove(kinjo::arm::Arm* Arm, int x, int y, int z){
	if (!Arm->initialized) {
		std::cerr << "Arm not Connected!\n";
		return;
	}

	std::printf("moving to %i,%i,%i ...\n", x, y, z);

	float fx = static_cast<float>(x);
	float fy = static_cast<float>(y);
	float fz = static_cast<float>(z);
	cv::Vec3f position = cv::Vec3f(fx, fy, fz);

	Arm->moveTo(position);
	cv::Vec3f actual = Arm->getPosition();
	std::printf("moving done. New \"exact\" Position: %.4f,%.4f,%.4f\n",
		actual[0], actual[1], actual[2]);
}

void updateCoordinates(int event, int x, int y, int /*flags*/, void *param)
{
	cv::Point* point = static_cast<cv::Point*>(param);

	switch(event)
	{
	case CV_EVENT_LBUTTONUP:
		*point = cv::Point(x, y);
		break;
	}
}

void displayCoordinates(cv::Mat& image, cv::Point const & point, cv::Scalar const & color)
{
	circle(image, point, 3, color);
	circle(image, point, 7, color);
}

void displayDistance(cv::Mat& image, cv::Point const & point, cv::Scalar const & color, ushort distance)
{
	float dist_cm = static_cast<float>(distance) / 10.0f;
	std::stringstream stream;
	stream << "[ " << dist_cm << "cm ]";
	cv::Point shifted = point + cv::Point(15, -5);
	putText(image, stream.str(), shifted, cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
}

int main(int /*argc*/, char* /*argv*/[]){

	try
	{
		// Initialize the arm.
		std::shared_ptr<kinjo::arm::Arm> Arm = kinjo::arm::ArmFactory::getInstance();

		// Create the arm movement window.
		int posX = 0;
		int posY = 0;
		int posZ = 0;
		cv::namedWindow(g_sWindowTitleArm, cv::WINDOW_AUTOSIZE);

		// Get the current arm position.
		if (Arm->initialized){
			cv::Vec3f initial = Arm->getPosition();
			posX = (int) initial[0];
			posY = (int) initial[1];
			posZ = (int) initial[2];
		}

		int const slider_max = 50;
		cv::createTrackbar("X", g_sWindowTitleArm, &posX, slider_max);
		cv::createTrackbar("Y", g_sWindowTitleArm, &posY, slider_max);
		cv::createTrackbar("Z", g_sWindowTitleArm, &posZ, slider_max);

		// Create the vision.
		std::shared_ptr<kinjo::vision::Vision> vision = std::make_shared<kinjo::vision::OpenNiVision>();

		cv::Point point(-1, -1);
		cv::namedWindow(g_sWindowTitleDepth);
		cv::namedWindow(g_sWindowTitleColor);
		cv::setMouseCallback(g_sWindowTitleDepth, updateCoordinates, &point);
		cv::setMouseCallback(g_sWindowTitleColor, updateCoordinates, &point);

		cv::Scalar const depthColor = cv::Scalar(255 << 8);
		cv::Scalar const rgbColor = cv::Scalar(0, 255, 0);

		int key = -1;

		cv::Mat depth, rgb;
		do
		{
			// Get the vision images.
			depth = vision->getDepth();
			rgb = vision->getRgb();

			// Find the yellow ball.
//			kinjo::recognition::getCalibrationObjectVisionPosition(rgb, depth);

			// Show depth, color and depth of selected point.
			if(0 <= point.x && point.x < depth.cols
				&& 0 <= point.y && point.y < depth.rows)
			{
				displayCoordinates(depth, point, depthColor);
				displayCoordinates(rgb, point, rgbColor);

				ushort distance = depth.at<ushort>(point.y, point.x);
				if(distance > 0)
				{
					displayDistance(depth, point, depthColor, distance);
					displayDistance(rgb, point, rgbColor, distance);
				}
			}
			imshow(g_sWindowTitleDepth, depth * 10);
			imshow(g_sWindowTitleColor, rgb);

			key = cv::waitKey(g_iRefreshIntervallMs);

			// Move the arm when pressing space.
			if(key == 32)
			{
				jacoMove(Arm.get(), posX, posY, posZ);
			}

		} while(key != 27);

		cv::destroyAllWindows();

		return 0;
	}
	catch (std::exception const & e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	catch (...)
	{
		std::cerr << "Unknown exception!" << std::endl;
		return 1;
	}
}
