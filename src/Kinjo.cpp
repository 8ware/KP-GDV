/**
* This code has been developed during the WS 14/15 KP-CGV at the TU-Dresden
**/

// Under windows, opencv includes <windows.h> which defines min and max as macros.
// To enable their usage as functions we have to prevent this.
#define NOMINMAX

#include <kinjo/RenderHelper.hpp>
#include <kinjo/arm/JacoArm.hpp>
#include <kinjo/arm/MovementGuardOne.hpp>
#include <kinjo/vision/OpenNiVision.hpp>
#include <kinjo/calibration/AutomaticCalibrator.hpp>
#include <kinjo/calibration/HardCodedCalibrator.hpp>
#include <kinjo/calibration/RandomCalibrationPointGenerator.hpp>
#include <kinjo/recognition/ColorBasedCircleRecognizer.hpp>
#include <kinjo/recognition/ManualRecognizer.hpp>
#ifndef _MSC_VER
#include <kinjo/mock/DirectoryBasedDataProvider.hpp>
#include <kinjo/mock/TestdataMock.hpp>
#endif
#include <kinjo/config/Config.hpp>

#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f
#include <opencv2/highgui/highgui.hpp>
#include <easylogging++.h>

#include <cmath>		// std::modf
#include <limits>		// std::numeric_limits
#include <cstdint>		// std::uint16_t

INITIALIZE_EASYLOGGINGPP

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
	
	/**
	 * The main kinjo function.
	 **/
	int run(
		arm::Arm* arm,
		vision::Vision* vision,
		calibration::Calibrator* calibrator,
		recognition::Recognizer* recognizer,
		std::string matrixFileName,
		float graspXOffset, float graspYOffset, float graspZOffset)
	{
		// At least a vision is required to have minimum functionality.
		assert(vision);

		// Uncalibrated is the initial state.
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
		cv::Scalar const rgbGray(200, 200, 200);

		int key = -1;
		cv::Mat matDepth, matRgb;

		do
		{
			// Update the vision images.
			vision->updateImages(false);
			vision->getDepth().copyTo(matDepth);
			vision->getRgb().copyTo(matRgb);

			// If there is no arm, the application runs only with minimum functionality (camera view only)
			if(arm)
			{
				if(applicationState == ApplicationState::Uncalibrated)
				{
					// Render the text centered.
					renderTextCenter(matRgb, rgbColor, "Press 'c' to start calibration!", 1.0, 3);

					// Start calibration after pressing c.
					if(key == 'c')
					{
						applicationState = ApplicationState::Calibration;

						// Move arm to its start position.
						arm->moveToStartPosition(true);

						// Start the calibration thread.
						calibrator->calibrateAsync();
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

						cv::Matx44f calibMatrix = calibrator->getRigidBodyTransformation();
						LOG(INFO) << "rigidBodyTransformation: " << calibMatrix;

						kinjo::config::Config::writeMatrixToFile(matrixFileName,calibMatrix);
					}
				}


				else if(applicationState == ApplicationState::Calibrated)
				{
					// \TODO: This movement should be asynchronous to show it immediately.
					if(mouseState.pointChanged)
					{
						mouseState.pointChanged = false;

						cv::Matx44f const mat44fRigidBodyTransformation(calibrator->getRigidBodyTransformation());
						
						LOG(INFO) << "Click: image pixel position: " << mouseState.point;

						cv::Vec3f const v3fVisionPosition(
							vision->estimateVisionPositionFromImagePointPx(mouseState.point));

						if(v3fVisionPosition[2]>0.0f)
						{
							//arm->moveToStartPosition(false);
							//arm->openFingers();
							
							LOG(INFO) << "Click: vision position: " << v3fVisionPosition;

							cv::Vec3f v3fArmPosition(mat44fRigidBodyTransformation * v3fVisionPosition);
							LOG(INFO) << "Click: resulting arm position: " << v3fArmPosition;

							v3fArmPosition[0] += graspXOffset;
							v3fArmPosition[1] += graspYOffset;
							v3fArmPosition[2] += graspZOffset;
							LOG(INFO) << "Click: arm position + offset: " << v3fArmPosition;

							//cv::Vec3f const cap = arm->getPosition();
							//cv::Vec3f const zInvariant(v3fArmPosition[0], v3fArmPosition[1], cap[2]);

							//arm->moveTo(zInvariant);
							//arm->moveTo(v3fArmPosition);

							arm->GrabItem(v3fArmPosition);

							//arm->closeFingers();

							//arm->moveToStartPosition(true);
						}
						else
						{
							LOG(WARNING) << "Click: Can not move there! Unknown depth!";
						}
					}
				}

				// Render current arm position(in arm coordinate system).
				cv::Vec3f const v3fArmPosition(
					arm->getPosition());
				renderPosition(matRgb, cv::Point(0, 30), rgbColor, v3fArmPosition);
			}

			// Render a raster into the image.
			renderRaster(matRgb, rgbGray);

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
		// Initialize logging.
		START_EASYLOGGINGPP(argc, argv);

		// Declare the components being used by the kinjo application.
		std::shared_ptr<kinjo::arm::Arm> arm;
		std::shared_ptr<kinjo::vision::Vision> vision;
		std::shared_ptr<kinjo::calibration::CalibrationPointGenerator> calibrationPointGenerator;
		std::shared_ptr<kinjo::calibration::Calibrator> calibrator;
		std::shared_ptr<kinjo::recognition::Recognizer> recognizer;
#ifndef _MSC_VER
		std::shared_ptr<kinjo::mock::DirectoryBasedDataProvider> dataProvider;
#endif

		std::string sConfigPath;
		// If there is no command line argument, take the config from the current directory.
		if (argc < 2) {
			sConfigPath = "config.json";
		}
		else {
			sConfigPath = argv[1u];
		}

		// Load the configuration.
		kinjo::config::Config config(sConfigPath);

		// In the minimal vision only mode we only load the vision component.
		bool const bMinimalVisionOnly(config.getBool("other", "bMinimalVisionOnly"));
		if(bMinimalVisionOnly) {
			vision = std::make_shared<kinjo::vision::OpenNiVision>();
		}
		else
		{
			// Load a recognizer if the calibration is not hard coded.
			bool const bUseHardCodedCalibration(config.getBool("calibration", "bUseHardCodedCalibration"));
			if(!bUseHardCodedCalibration) {
				bool const bUseManualRecognizer(config.getBool("recognition", "bUseManualRecognizer"));
				if(bUseManualRecognizer) {
					recognizer = std::make_shared<kinjo::recognition::ManualRecognizer>();
				}
				else {
					std::size_t const uiRecognitionAttemptPerRotationCount(static_cast<std::size_t>(config.getInt("colorBasedCircleRecognizer", "uiRecognitionAttemptCount")));
					int const iMorphSizeDilatePx(config.getInt("colorBasedCircleRecognizer", "iMorphSizeDilatePx"));
					int const iMorphSizeErodePx(config.getInt("colorBasedCircleRecognizer", "iMorphSizeErodePx"));
					int const iGaussianBlurFilterWidthHalf(config.getInt("colorBasedCircleRecognizer", "iGaussianBlurFilterWidthHalf"));
					int const iInvRatioAccuSize(config.getInt("colorBasedCircleRecognizer", "iInvRatioAccuSize"));
					int const iMinCircleDistImageHeightPercent(config.getInt("colorBasedCircleRecognizer", "iMinCircleDistImageHeightPercent"));
					int const iCannyEdgeThreshold(config.getInt("colorBasedCircleRecognizer", "iCannyEdgeThreshold"));
					int const iHoughAccuThreshold(config.getInt("colorBasedCircleRecognizer", "iHoughAccuThreshold"));
					int const iMinCircleRadiusImageHeightPercent(config.getInt("colorBasedCircleRecognizer", "iMinCircleRadiusImageHeightPercent"));
					int const iMaxCircleRadiusImageHeightPercent(config.getInt("colorBasedCircleRecognizer", "iMaxCircleRadiusImageHeightPercent"));
					int const iMinHuePercent(config.getInt("colorBasedCircleRecognizer", "iMinHuePercent"));
					int const iMaxHuePercent(config.getInt("colorBasedCircleRecognizer", "iMaxHuePercent"));
					int const iMinSatPercent(config.getInt("colorBasedCircleRecognizer", "iMinSatPercent"));
					int const iMinValPercent(config.getInt("colorBasedCircleRecognizer", "iMinValPercent"));
					recognizer = std::make_shared<kinjo::recognition::ColorBasedCircleRecognizer>(
						uiRecognitionAttemptPerRotationCount,
						iMorphSizeDilatePx,
						iMorphSizeErodePx,
						iGaussianBlurFilterWidthHalf,
						iInvRatioAccuSize,
						iMinCircleDistImageHeightPercent,
						iCannyEdgeThreshold,
						iHoughAccuThreshold,
						iMinCircleRadiusImageHeightPercent,
						iMaxCircleRadiusImageHeightPercent,
						iMinHuePercent,
						iMaxHuePercent,
						iMinSatPercent,
						iMinValPercent);
				}
			}

			// Load the arm and the vision.
			bool const bUseMockImplementation(config.getBool("mock", "bUseMockImplementation"));
			if (!bUseMockImplementation) {
				std::list<std::shared_ptr<kinjo::arm::MovementGuard>> MovGuardList;
				MovGuardList.push_back(std::make_shared<kinjo::arm::MovementGuardOne>());
				arm = std::make_shared<kinjo::arm::JacoArm>(MovGuardList);

				// Load a random calibration point generator if the calibration is not hard coded.
				if(!bUseHardCodedCalibration) {
					std::size_t const uiRandomCalibrationPointGeneratorSeed(config.getInt("calibration", "uiRandomCalibrationPointGeneratorSeed"));
					calibrationPointGenerator = std::make_shared<kinjo::calibration::RandomCalibrationPointGenerator>(
						uiRandomCalibrationPointGeneratorSeed);
				}

				vision = std::make_shared<kinjo::vision::OpenNiVision>();

			} else {
#ifndef _MSC_VER
				std::string const sMockDataDirectory(config.getString("mock", "sMockDataDirectory"));
				dataProvider = std::make_shared<kinjo::mock::DirectoryBasedDataProvider>(sMockDataDirectory);

				std::shared_ptr<kinjo::mock::TestdataMock> mock = 
					std::make_shared<kinjo::mock::TestdataMock>(dataProvider.get());

				arm = std::dynamic_pointer_cast<kinjo::arm::Arm>(mock);
				vision = std::dynamic_pointer_cast<kinjo::vision::Vision>(mock);
				calibrationPointGenerator = std::dynamic_pointer_cast<kinjo::calibration::CalibrationPointGenerator>(mock);

				cv::Vec3f position = calibrationPointGenerator->getNextCalibrationPoint();
				arm->moveTo(position);
#else
				throw std::logic_error("DirectoryBasedDataProvider not supported on windows!");
#endif
			}

			// Load a calibrator.
			if(bUseHardCodedCalibration) {
				std::string const sMatrixFilePath(config.getString("hardCodedCalibrator", "matrixFileName"));
				cv::Matx44f mat44fRigidBodyTransformation(kinjo::config::Config::readMatrixFromFile(sMatrixFilePath));
				calibrator = std::make_shared<kinjo::calibration::HardCodedCalibrator>(mat44fRigidBodyTransformation);
			}
			else {
				std::size_t const uiCalibrationPointCount(static_cast<std::size_t>(config.getInt("automaticCalibrator", "uiCalibrationPointCount")));
				std::size_t const uiCalibrationRotationCount(static_cast<std::size_t>(config.getInt("automaticCalibrator", "uiCalibrationRotationCount")));
				std::size_t const uiMinimumValidPositionsAfterFilteringPercent(static_cast<std::size_t>(config.getInt("automaticCalibrator", "uiMinimumValidPositionsAfterFilteringPercent")));
				float const fMaximumFilterEuclideanDistancePointToAverage(config.getFloat("automaticCalibrator", "fMaximumFilterEuclideanDistancePointToAverage"));
				calibrator = std::make_shared<kinjo::calibration::AutomaticCalibrator>(
					arm.get(),
					vision.get(),
					recognizer.get(),
					calibrationPointGenerator.get(),
					uiCalibrationPointCount,
					uiCalibrationRotationCount,
					uiMinimumValidPositionsAfterFilteringPercent,
					fMaximumFilterEuclideanDistancePointToAverage);
			}
		}

		std::string matrixFileName = config.getString("hardCodedCalibrator", "matrixFileName");

		float graspXOffset = config.getFloat("grasping", "x-offset");
		float graspYOffset = config.getFloat("grasping", "y-offset");
		float graspZOffset = config.getFloat("grasping", "z-offset");
		
		// Execute the kinjo main loop.
		return kinjo::run(arm.get(), vision.get(), calibrator.get(), recognizer.get(),
				matrixFileName, graspXOffset, graspYOffset, graspZOffset);
	}
	catch (std::exception const & e)
	{
		LOG(FATAL) << e.what();
		std::cin.ignore();
		return 1;
	}
	catch (...)
	{
		LOG(FATAL) << "Unknown exception!";
		std::cin.ignore();
		return 1;
	}
}

