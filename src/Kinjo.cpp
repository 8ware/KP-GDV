#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/vision/OpenNiVision.hpp>
#include <kinjo/calibration/Calibration.hpp>
#include <kinjo/recognition/Recognition.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <cmath>		// std::modf

static std::string const g_sWindowTitleDepth("Vision (Depth)");
static std::string const g_sWindowTitleColor("Vision (RGB)");
static std::string const g_sWindowTitleArm("Arm");
static const int g_iRefreshIntervallMs(100);

#define KINJO_ARM_DEBUG

#ifdef KINJO_ARM_DEBUG
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
#endif

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
	cv::circle(image, point, 3, color);
	cv::circle(image, point, 7, color);
}

/**
* Render the text at the center.
**/
void renderTextCenter(
	cv::Mat & image, 
	cv::Scalar const & color, 
	std::string const & sText, 
	double fFontScale,
	int iThickness)
{
	int const fontFace(
		cv::FONT_HERSHEY_SIMPLEX);

	int iBaseline(0);
	cv::Size textSize(
		cv::getTextSize(
			sText,
			fontFace,
			fFontScale, iThickness,
			&iBaseline));
	iBaseline += iThickness;

	cv::Point const textOrg(
		(image.cols - textSize.width)/2,
		(image.rows + textSize.height)/2);

	cv::putText(
		image,
		sText, textOrg,
		fontFace, fFontScale,
		color, iThickness);
}

/**
 * Render the given 3d-vector at the given point.
 **/
void displayPosition(cv::Mat& image, cv::Point const & point, cv::Scalar const & color, cv::Vec3f const & v3fVisionPosition)
{
	std::stringstream stream;
	stream << "[" << v3fVisionPosition[0u] << ", " << v3fVisionPosition[1u] << ", " << v3fVisionPosition[2u] << "] cm";
	cv::Point const shifted = point + cv::Point(15, -5);
	cv::putText(image, stream.str(), shifted, cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
}

/**
 * The application states.
 **/
enum class ApplicationState
{
	Uncalibrated,
	Calibration,
	Calibrated,
};

int main(int /*argc*/, char* /*argv*/[]){

	try
	{
		ApplicationState applicationState(ApplicationState::Uncalibrated);

		// Initialize the arm.
		std::shared_ptr<kinjo::arm::Arm> arm(kinjo::arm::ArmFactory::getInstance());
		// Create the vision.
		std::shared_ptr<kinjo::vision::Vision> vision(std::make_shared<kinjo::vision::OpenNiVision>());
		// Create the calibrator.
		std::shared_ptr<kinjo::calibration::Calibrator> calibrator(std::make_shared<kinjo::calibration::Calibrator>(
			arm.get(),
			vision.get()));


		// Initialize the main windows.
		cv::Point point(-1, -1);
		cv::namedWindow(g_sWindowTitleDepth);
		cv::namedWindow(g_sWindowTitleColor);
		cv::setMouseCallback(g_sWindowTitleDepth, updateCoordinates, &point);
		cv::setMouseCallback(g_sWindowTitleColor, updateCoordinates, &point);

		cv::Scalar const depthColor(cv::Scalar(255 << 8));
		cv::Scalar const rgbColor(cv::Scalar(0, 255, 0));

		int key = -1;
		cv::Mat matDepth, matRgb;

#ifdef KINJO_ARM_DEBUG
		// Create the arm movement window.
		int posX = 0;
		int posY = 0;
		int posZ = 0;
		cv::namedWindow(g_sWindowTitleArm, cv::WINDOW_AUTOSIZE);

		// Get the current arm position.
		if(arm->initialized)
		{
			cv::Vec3f initial = arm->getPosition();
			posX = (int)initial[0];
			posY = (int)initial[1];
			posZ = (int)initial[2];
		}

		int const slider_max(500);
		cv::createTrackbar("X", g_sWindowTitleArm, &posX, slider_max);
		cv::createTrackbar("Y", g_sWindowTitleArm, &posY, slider_max);
		cv::createTrackbar("Z", g_sWindowTitleArm, &posZ, slider_max);
#endif

		do
		{
			// Update the vision images.
			vision->updateImages(false);
			vision->getDepth().copyTo(matDepth);
			vision->getRgb().copyTo(matRgb);


			if(applicationState == ApplicationState::Uncalibrated)
			{
				// Render the text centered.
				renderTextCenter(matRgb, rgbColor, "Press 'c' to start calibration!", 1.0, 3);

				// Start calibration after pressing c.
				if(key == 'c')
				{
					applicationState = ApplicationState::Calibration;

					std::size_t const uiCalibrationPointCount(5);
					std::size_t const uiCalibrationRotationCount(10);
					std::size_t const uiRecognitionAttemptCount(5);
					// TODO: This should be asynchronous!
					/*calibrator->calibrate(
						uiCalibrationPointCount,
						uiCalibrationRotationCount,
						uiRecognitionAttemptCount);*/
				}
			}


			else if(applicationState == ApplicationState::Calibration)
			{
				// NOTE: Searching for the calibration object and rendering it influences the speed negatively!
				std::pair<cv::Vec2f, float> const calibrationObjectPositionPx(
					kinjo::recognition::getCalibrationObjectVisionPositionPx(matRgb));

				// If recognition was successfull.
				if(calibrationObjectPositionPx.second != 0.0f)
				{
					cv::Point const v2iCenter(
						cvRound(calibrationObjectPositionPx.first[0]),
						cvRound(calibrationObjectPositionPx.first[1]));
					int const iRadius(cvRound(calibrationObjectPositionPx.second));
					// Draw the center point.
					cv::circle(matRgb, v2iCenter, 3, cv::Scalar(0, 255, 0), -1, CV_AA, 0);
					// Draw the circle.
					cv::circle(matRgb, v2iCenter, iRadius, cv::Scalar(255, 0, 0), 3, CV_AA, 0);

					// Get the 3d position from the 2d point.
					cv::Vec3f const v3fVisionPosition(
						vision->estimatePositionFromImagePointPx(v2iCenter));

					// Display its coordinates.
					displayPosition(matDepth, v2iCenter, depthColor, v3fVisionPosition);
					displayPosition(matRgb, v2iCenter, rgbColor, v3fVisionPosition);
				}

				// If the calibration is finished.
				if(calibrator->getIsValidTransformationAvailable())
				{
					applicationState = ApplicationState::Calibrated;
				}
			}


			else if(applicationState == ApplicationState::Calibrated)
			{
				cv::Matx44f rigidBodyTransformation(calibrator->getRigidBodyTransformation());

				// TODO: Implement object catching.
			}

#ifdef KINJO_ARM_DEBUG
			// Move the arm when pressing space.
			if(key == 32)
			{
				jacoMove(arm.get(), posX, posY, posZ);
			}
#endif

			// Show depth, color and selected point.
			if(0 <= point.x && point.x < matDepth.cols
				&& 0 <= point.y && point.y < matDepth.rows)
			{
				displayCoordinates(matDepth, point, depthColor);
				displayCoordinates(matRgb, point, rgbColor);

				cv::Vec3f const v3fVisionPosition(
					vision->estimatePositionFromImagePointPx(point));
				if(v3fVisionPosition[2u] > 0)
				{
					displayPosition(matDepth, point, depthColor, v3fVisionPosition);
					displayPosition(matRgb, point, rgbColor, v3fVisionPosition);
				}
			}
			// Scale the depth image by 10 to make it more visible.
			cv::imshow(g_sWindowTitleDepth, matDepth * 10);
			cv::imshow(g_sWindowTitleColor, matRgb);

			// Get currently pressed keys and wait to restrict the framerate.
			key = cv::waitKey(g_iRefreshIntervallMs);

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
