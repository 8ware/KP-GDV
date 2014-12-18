#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/vision/OpenNiVision.hpp>

#include <kinjo/recognition/Recognition.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

static std::string const g_sWindowTitleDepth("Vision (Depth)");
static std::string const g_sWindowTitleColor("Vision (RGB)");
static std::string const g_sWindowTitleArm("Arm");
static const int g_iRefreshIntervallMs(100);

//not yet needed::eventHandler for slider(s)
void on_trackbar(int, void*){}

void jacoMove(kinjo::arm::Arm * Arm){

	cv::Vec3f vector = cv::Vec3f(
		static_cast<float>(cv::getTrackbarPos("X", "kinjo")),
		static_cast<float>(cv::getTrackbarPos("Y", "kinjo")),
		static_cast<float>(cv::getTrackbarPos("Z", "kinjo"))
	);
	std::printf("moving to %i,%i,%i ...\n",
		static_cast<float>(cv::getTrackbarPos("X", "kinjo")),
		static_cast<float>(cv::getTrackbarPos("Y", "kinjo")),
		static_cast<float>(cv::getTrackbarPos("Z", "kinjo"))
	);
	
	if (Arm->initialized){
		Arm->moveTo(vector);
		std::printf("moving done. New \"exact\" Position: %f,%f,%f\n",
			Arm->getPosition()[0],
			Arm->getPosition()[1],
			Arm->getPosition()[2]
			);
	}
	else
	{
		std::printf("Arm not Connected");
	}

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
	float dist_cm = static_cast<float>(distance / 10);
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
			posX = (int)Arm->getPosition()[0];
			posY = (int)Arm->getPosition()[1];
			posZ = (int)Arm->getPosition()[2];
		}

		int const slider_max = 50;
		cv::createTrackbar("X", g_sWindowTitleArm, &posX, slider_max, on_trackbar);
		cv::createTrackbar("Y", g_sWindowTitleArm, &posY, slider_max, on_trackbar);
		cv::createTrackbar("Z", g_sWindowTitleArm, &posZ, slider_max, on_trackbar);

		// Create the vision.
		std::shared_ptr<kinjo::vision::Vision> vision = std::make_shared<kinjo::vision::OpenNiVision>();

		cv::Point point(-1, -1);
		cv::namedWindow(g_sWindowTitleDepth);
		cv::namedWindow(g_sWindowTitleColor);
		setMouseCallback(g_sWindowTitleDepth, updateCoordinates, &point);
		setMouseCallback(g_sWindowTitleColor, updateCoordinates, &point);

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
			kinjo::recognition::getCalibrationObjectVisionPosition(rgb, depth);

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
				jacoMove(Arm.get());
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
