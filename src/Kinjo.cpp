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
#include <kinjo/Config/Config.hpp>
#include <kinjo/calibration/RandomCalibrationPointGenerator.hpp>
#include <kinjo/recognition/ColorBasedCircleRecognizer.hpp>
#include <kinjo/recognition/ManualRecognizer.hpp>
#ifndef _MSC_VER
#include <kinjo/mock/DirectoryBasedDataProvider.hpp>
#include <kinjo/mock/TestdataMock.hpp>
#endif

#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f
#include <opencv2/highgui/highgui.hpp>

#include <iostream>		// std::cout
#include <cmath>		// std::modf
#include <limits>		// std::numeric_limits
#include <cstdint>		// std::uint16_t

//#define KINJO_NO_ARM

namespace kinjo
{
	static std::string const g_sWindowTitleDepth("Vision (Depth)");
	static std::string const g_sWindowTitleColor("Vision (RGB)");
	static std::string const g_sWindowTitleArm("Arm");
	static const int g_iRefreshIntervallMs(50);

	/**
	 * The data structure that is filled in the callback.
	 **/
	struct MouseCallback
	{
		cv::Point point;
		bool pointChanged;
	};

	/**
	 * The mouse position change callback.
	 **/
	void mouseCallback(int event, int x, int y, int /*flags*/, void *param)
	{
		MouseCallback* mouseState = static_cast<MouseCallback*>(param);

		switch(event)
		{
		case CV_EVENT_LBUTTONUP:
			mouseState->point = cv::Point(x, y);
			mouseState->pointChanged = true;
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

	int run(
		arm::Arm* arm,
		vision::Vision* vision,
		calibration::Calibrator* calibrator,
		recognition::Recognizer* recognizer)
	{
		ApplicationState applicationState(ApplicationState::Uncalibrated);

		// Initialize the main windows.
		MouseCallback mouseState;
		mouseState.point = cv::Point(-1, -1);
		mouseState.pointChanged = false;
		cv::namedWindow(g_sWindowTitleDepth);
		cv::namedWindow(g_sWindowTitleColor);
		cv::setMouseCallback(g_sWindowTitleDepth, mouseCallback, &mouseState);
		cv::setMouseCallback(g_sWindowTitleColor, mouseCallback, &mouseState);

		cv::Scalar const depthColor(cv::Scalar(255 << 8));
		cv::Scalar const rgbColor(cv::Scalar(0, 255, 0));

		int key = -1;
		cv::Mat matDepth, matRgb;

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
				renderTextCenter(matRgb, rgbColor, "Press 'c' to start calibration!", 1.0, 3);

				// Start calibration after pressing c.
				if(key == 'c')
				{
					applicationState = ApplicationState::Calibration;

					std::size_t const uiCalibrationPointCount(8);
					std::size_t const uiCalibrationRotationCount(3);

					// Move arm to its start position.
					arm->moveToStartPosition(true);

					// Start the calibration thread.
					calibrator->calibrateAsync(
						uiCalibrationPointCount,
						uiCalibrationRotationCount);
				}
			}


			else if(applicationState == ApplicationState::Calibration)
			{
				// Render the text centered.
				kinjo::renderTextCenter(matRgb, rgbColor, "Calibrating...", 1.0, 3);

				auto const pColorBasedCircleRecognizer(dynamic_cast<kinjo::recognition::ColorBasedCircleRecognizer*>(recognizer));
				if(pColorBasedCircleRecognizer)
				{
					std::pair<cv::Point, float> const calibrationObjectPositionPx(
						pColorBasedCircleRecognizer->estimateCalibrationObjectImagePointPxAndRadius(matRgb));

					// If recognition was successfull.
					if(calibrationObjectPositionPx.second > 0.0f)
					{
						cv::Point const v2iCenter(calibrationObjectPositionPx.first);
						int const iRadius(cvRound(calibrationObjectPositionPx.second));
						// Draw the center point.
						cv::circle(matRgb, v2iCenter, 3, cv::Scalar(0, 255, 0), -1, CV_AA, 0);
						// Draw the circle.
						cv::circle(matRgb, v2iCenter, iRadius, cv::Scalar(255, 0, 0), 3, CV_AA, 0);

						// Get the 3d position from the 2d point.
						cv::Vec3f const v3fVisionPosition(
							vision->estimateVisionPositionFromImagePointPx(v2iCenter));

						// Display its coordinates.
						renderPosition(matDepth, v2iCenter, depthColor, v3fVisionPosition);
						renderPosition(matRgb, v2iCenter, rgbColor, v3fVisionPosition);
					}
				}

				// If the calibration is finished.
				if(calibrator->getIsValidTransformationAvailable())
				{
					applicationState = ApplicationState::Calibrated;

					std::cout << "rigidBodyTransformation: " << std::endl << calibrator->getRigidBodyTransformation() << std::endl;
				}
			}


			else if(applicationState == ApplicationState::Calibrated)
			{
				// \TODO: This movement should be asynchronous to show it immediately.
				if(mouseState.pointChanged)
				{
					mouseState.pointChanged = false;

					cv::Matx44f const mat44fRigidBodyTransformation(calibrator->getRigidBodyTransformation());

					std::cout << "Click: image pixel position: " << mouseState.point << std::endl;

					cv::Vec3f const v3fVisionPosition(
						vision->estimateVisionPositionFromImagePointPx(mouseState.point));

					if(v3fVisionPosition[2]>0.0f)
					{
						//arm->openFingers();

						std::cout << "Click: vision position: " << v3fVisionPosition << std::endl;

						cv::Vec3f const v3fArmPosition(mat44fRigidBodyTransformation * v3fVisionPosition);

						std::cout << "Click: resulting arm position: " << v3fArmPosition << std::endl;

						arm->moveTo(v3fArmPosition);

						//arm->closeFingers();
					}
					else
					{
						std::cout << "Click: Can not move there! Unknown depth!" << std::endl;
					}
				}
			}
#endif

			// Show depth, color and selected point.
			if(0 <= mouseState.point.x && mouseState.point.x < matDepth.cols
				&& 0 <= mouseState.point.y && mouseState.point.y < matDepth.rows)
			{
				renderDoubleCircle(matDepth, mouseState.point, depthColor);
				renderDoubleCircle(matRgb, mouseState.point, rgbColor);

				cv::Vec3f const v3fVisionPosition(
					vision->estimateVisionPositionFromImagePointPx(mouseState.point));
				if(v3fVisionPosition[2u] > 0)
				{
					renderPosition(matDepth, mouseState.point, depthColor, v3fVisionPosition);
					renderPosition(matRgb, mouseState.point, rgbColor, v3fVisionPosition);
				}
			}

#ifndef KINJO_NO_ARM
			// Render current arm position(in arm coordinate system).
			cv::Vec3f const v3fArmPosition(
				arm->getPosition());
			renderPosition(matRgb, cv::Point(0, 30), rgbColor, v3fArmPosition);
#endif

			// Scale the depth image to use the whole 16-bit and make it more visible.
			// This value seems not to be correct...
			//float const fMaxVisionDepthValue(static_cast<float>(vision->getMaxDepthValue()));
			float const fMaxVisionDepthValue(4000.0f);
			float const fImageValueScale(
				static_cast<float>(std::numeric_limits<std::uint16_t>::max())/fMaxVisionDepthValue);
			cv::imshow(g_sWindowTitleDepth, matDepth * fImageValueScale);
			cv::imshow(g_sWindowTitleColor, matRgb);

			// Get currently pressed keys and wait to restrict the framerate.
			key = cv::waitKey(g_iRefreshIntervallMs);

		} while(key != 27);

		cv::destroyAllWindows();

		return 0;
	}
}

/**
 * The application entry point.
 **/
int main(int argc, char* argv[])
{
	try
	{
		std::shared_ptr<kinjo::arm::Arm> arm;
		std::shared_ptr<kinjo::vision::Vision> vision;
		std::shared_ptr<kinjo::calibration::CalibrationPointGenerator> calibrationPointGenerator;
		std::shared_ptr<kinjo::calibration::Calibrator> calibrator;
		
		//exception if config.json not in built project folder
		Config h("config.json");

		std::shared_ptr<kinjo::recognition::Recognizer> recognizer(std::make_shared<kinjo::recognition::ColorBasedCircleRecognizer>());
		//std::shared_ptr<kinjo::recognition::Recognizer> recognizer(std::make_shared<kinjo::recognition::ManualRecognizer>());
		
#ifndef _MSC_VER
		std::shared_ptr<kinjo::mock::DirectoryBasedDataProvider> dataProvider;
#endif
		if (argc < 2) {

#ifndef KINJO_NO_ARM
			arm = kinjo::arm::ArmFactory::getInstance();
#endif
			vision = std::make_shared<kinjo::vision::OpenNiVision>();

#ifndef KINJO_NO_ARM
			calibrationPointGenerator = std::make_shared<kinjo::calibration::RandomCalibrationPointGenerator>();
#endif
		} else {
#ifndef _MSC_VER
			std::string directory = argv[1];
			dataProvider = std::make_shared<kinjo::mock::DirectoryBasedDataProvider>(directory);

			std::shared_ptr<kinjo::mock::TestdataMock>  = 
				std::make_shared<kinjo::mock::TestdataMock> mock(dataProvider.get());

			arm = std::dynamic_pointer_cast<kinjo::arm::Arm>(mock);
			vision = std::dynamic_pointer_cast<kinjo::arm::Arm>(mock);
			calibrationPointGenerator = std::dynamic_pointer_cast<kinjo::arm::Arm>(mock);

			cv::Vec3f position = calibrationPointGenerator->getNextCalibrationPoint();
			arm->moveTo(position);

#else
			throw std::logic_error("DirectoryBasedDataProvider not supported on windows!");
#endif
		}

		calibrator = std::make_shared<kinjo::calibration::Calibrator>(
			arm.get(),
			vision.get(),
			recognizer.get(),
			calibrationPointGenerator.get());
		
		return kinjo::run(arm.get(), vision.get(), calibrator.get(), recognizer.get());
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

