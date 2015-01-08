/**
* This code has been developed during the WS 14/15 KP-CGV at the TU-Dresden
**/

// Under windows, opencv includes <windows.h> which defines min and max as macros.
// To enable their usage as functions we have to prevent this.
#define NOMINMAX

#include <kinjo/RenderHelper.hpp>
#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/vision/OpenNiVision.hpp>
#include <kinjo/calibration/Calibration.hpp>
#include <kinjo/recognition/Recognition.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>		// std::cout
#include <cmath>		// std::modf
#include <limits>		// std::numeric_limits
#include <cstdint>		// std::uint16_t

static std::string const g_sWindowTitleDepth("Vision (Depth)");
static std::string const g_sWindowTitleColor("Vision (RGB)");
static std::string const g_sWindowTitleArm("Arm");
static const int g_iRefreshIntervallMs(100);

#define KINJO_ARM_DEBUG
//#define KINJO_NO_ARM

#ifndef KINJO_NO_ARM
#ifdef KINJO_ARM_DEBUG
/**
* Moves the arm to the desired position.
**/
void jacoMove(kinjo::arm::Arm* Arm, int x, int y, int z){

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

void jacoRotateBy(kinjo::arm::Arm* Arm, int x, int y, int z){

	std::printf("moving to %i,%i,%i ...\n", x, y, z);

	float fx = static_cast<float>(x);
	float fy = static_cast<float>(y);
	float fz = static_cast<float>(z);
	cv::Vec3f rotation = cv::Vec3f(fx, fy, fz);
	Arm->rotateBy(rotation);
	cv::Vec3f actual = Arm->getRotation();
	std::printf("moving done. New \"exact\" Rotation: %.4f,%.4f,%.4f\n",
		actual[0], actual[1], actual[2]);
}

#endif
#endif

/**
* The mouse position change callback.
**/
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

/**
* The application states.
**/
enum class ApplicationState
{
	Uncalibrated,
	Calibration,
	Calibrated,
};

/**
* The application entry point.
**/
int main(int /*argc*/, char* /*argv*/[])
{
	try
	{
		ApplicationState applicationState(ApplicationState::Uncalibrated);

#ifndef KINJO_NO_ARM
		// Initialize the arm.
		std::shared_ptr<kinjo::arm::Arm> arm(kinjo::arm::ArmFactory::getInstance());
#endif
		// Create the vision.
		std::shared_ptr<kinjo::vision::Vision> vision(std::make_shared<kinjo::vision::OpenNiVision>());

#ifndef KINJO_NO_ARM
		// Create the calibrator.
		std::shared_ptr<kinjo::calibration::Calibrator> calibrator(std::make_shared<kinjo::calibration::Calibrator>(
			arm.get(),
			vision.get()));
#endif

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


#ifndef KINJO_NO_ARM
#ifdef KINJO_ARM_DEBUG
		// Create the arm movement window.
		int posX = 0;
		int posY = 0;
		int posZ = 0;
		int rotZ = 0;
		cv::namedWindow(g_sWindowTitleArm, cv::WINDOW_AUTOSIZE);

		// Get the current arm position.
		cv::Vec3f initial = arm->getPosition();
		posX = (int)initial[0];
		posY = (int)initial[1];
		posZ = (int)initial[2];

		int const slider_max(500);
		cv::createTrackbar("X", g_sWindowTitleArm, &posX, slider_max);
		cv::createTrackbar("Y", g_sWindowTitleArm, &posY, slider_max);
		cv::createTrackbar("Z", g_sWindowTitleArm, &posZ, slider_max);

		cv::createTrackbar("Rotate", g_sWindowTitleArm, &rotZ, 360);
#endif
#endif

		do
		{
			// Update the vision images.
			vision->updateImages(false);
			vision->getDepth().copyTo(matDepth);
			vision->getRgb().copyTo(matRgb);


#ifndef KINJO_NO_ARM
			if(applicationState == ApplicationState::Uncalibrated)
			{
				// Render the text centered.
				kinjo::renderTextCenter(matRgb, rgbColor, "Press 'c' to start calibration!", 1.0, 3);

				// Start calibration after pressing c.
				if(key == 'c')
				{
					applicationState = ApplicationState::Calibration;

					std::size_t const uiCalibrationPointCount(5);
					std::size_t const uiCalibrationRotationCount(10);
					std::size_t const uiRecognitionAttemptCount(5);

					// Start the calibration thread.
					calibrator->calibrateAsync(
						uiCalibrationPointCount,
						uiCalibrationRotationCount,
						uiRecognitionAttemptCount);
				}
			}


			else if(applicationState == ApplicationState::Calibration)
			{
				// Render the text centered.
				kinjo::renderTextCenter(matRgb, rgbColor, "Calibrating...", 1.0, 3);

				// NOTE: Searching for the calibration object and rendering it influences the speed negatively!
				std::pair<cv::Vec2f, float> const calibrationObjectPositionPx(
					kinjo::recognition::getCalibrationObjectVisionPositionPx(matRgb));

				// If recognition was successfull.
				if(calibrationObjectPositionPx.second > 0.0f)
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
					renderPosition(matDepth, v2iCenter, depthColor, v3fVisionPosition);
					renderPosition(matRgb, v2iCenter, rgbColor, v3fVisionPosition);
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
				jacoRotateBy(arm.get(), 0, 0, rotZ);
			}
#endif
#endif

			// Show depth, color and selected point.
			if(0 <= point.x && point.x < matDepth.cols
				&& 0 <= point.y && point.y < matDepth.rows)
			{
				kinjo::renderDoubleCircle(matDepth, point, depthColor);
				kinjo::renderDoubleCircle(matRgb, point, rgbColor);

				cv::Vec3f const v3fVisionPosition(
					vision->estimatePositionFromImagePointPx(point));
				if(v3fVisionPosition[2u] > 0)
				{
					kinjo::renderPosition(matDepth, point, depthColor, v3fVisionPosition);
					kinjo::renderPosition(matRgb, point, rgbColor, v3fVisionPosition);
				}
			}
			// Scale the depth image to use the whole 16-bit and make it more visible.
			float fKinectMaxDepthMm(5000.0f);
			float fImageValueScale(std::numeric_limits<std::uint16_t>::max()/fKinectMaxDepthMm);
			cv::imshow(g_sWindowTitleDepth, matDepth * fImageValueScale);
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
