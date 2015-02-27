/**
* This code has been developed during the WS 14/15 KP-CGV at the TU-Dresden
**/

// Under windows, opencv includes <windows.h> which defines min and max as macros.
// To enable their usage as functions we have to prevent this.
#define NOMINMAX

#include <kinjo/RenderHelper.hpp>
#include <kinjo/arm/JacoArm.hpp>
#include <kinjo/arm/CylindricMovementGuard.hpp>
#include <kinjo/vision/OpenNiVision.hpp>
#include <kinjo/calibration/AutomaticCalibrator.hpp>
#include <kinjo/calibration/HardCodedCalibrator.hpp>
#include <kinjo/calibration/RandomCalibrationPointGenerator.hpp>
#include <kinjo/recognition/ColorBasedCircleRecognizer.hpp>
#include <kinjo/recognition/ManualRecognizer.hpp>
#include <kinjo/mock/DirectoryBasedDataProvider.hpp>
#include <kinjo/mock/TestdataMock.hpp>
#include <kinjo/config/Config.hpp>

#include <kinjo/app/InitialState.hpp>
#include <kinjo/app/ColoredCircleBasedCalibrationState.hpp>
#include <kinjo/app/ReadyState.hpp>
#include <kinjo/app/GraspState.hpp>

#include <kinjo/grasp/Grasper.hpp>
#include <kinjo/grasp/SimplePickAndDropGrasper.hpp>
#include <kinjo/grasp/PickStrategy.hpp>
#include <kinjo/grasp/DropStrategy.hpp>
#include <kinjo/grasp/SimpleTablePickStrategy.hpp>
#include <kinjo/grasp/BoxDropStrategy.hpp>
#include <kinjo/grasp/HandDropStrategy.hpp>
#include <kinjo/grasp/SimpleTableDropStrategy.hpp>

#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f
#include <opencv2/highgui/highgui.hpp>
#include <easylogging++.h>

#include <cmath>		// std::modf
#include <limits>		// std::numeric_limits
#include <cstdint>		// std::uint16_t


INITIALIZE_EASYLOGGINGPP


static const int ESCAPE = 27;

namespace kinjo
{
	static std::string const g_sWindowTitleDepth("Vision (Depth)");
	static std::string const g_sWindowTitleColor("Vision (RGB)");
	static const int g_iRefreshIntervallMs(50);

	/**
	 * The data structure that is filled in the callback.
	 **/
	struct MouseCallback
	{
		cv::Point point;
		bool pointChanged;
		app::State* state;
	};

	/**
	 * The mouse position change callback.
	 **/
	void mouseCallback(int event, int x, int y, int /*flags*/, void *param)
	{
		MouseCallback* mouseState = static_cast<MouseCallback*>(param);
		mouseState->state->process(event, cv::Point(x, y));

		// TODO only trigger callbacks if point is within image range
		switch(event)
		{
		case CV_EVENT_LBUTTONUP:
			mouseState->point = cv::Point(x, y);
			mouseState->pointChanged = true;
			break;
		case CV_EVENT_RBUTTONUP:
			mouseState->point = cv::Point(-1, -1);
		}
	}


	struct DepthView {
		void (*invoke)(MouseCallback&, DepthView&, bool&);
	};

	void enableDepthView(MouseCallback& mouseState, DepthView& depthView, bool& showDepthView);
	void disableDepthView(MouseCallback& mouseState, DepthView& depthView, bool& showDepthView);

	void enableDepthView(MouseCallback& mouseState, DepthView& depthView, bool& showDepthView) {
		cv::namedWindow(g_sWindowTitleDepth);
		cv::setMouseCallback(g_sWindowTitleDepth, mouseCallback, &mouseState);

		depthView.invoke = &disableDepthView;
		showDepthView = true;
	}

	void disableDepthView(MouseCallback& mouseState, DepthView& depthView, bool& showDepthView) {
		cv::destroyWindow(g_sWindowTitleDepth);

		depthView.invoke = &enableDepthView;
		showDepthView = false;
	}


	struct FingerState {
		void (*invoke)(arm::Arm*, FingerState&);
	};

	void openFingers(arm::Arm* arm, FingerState& state);
	void closeFingers(arm::Arm* arm, FingerState& state);

	void openFingers(arm::Arm* arm, FingerState& state) {
		LOG(DEBUG) << "Opening fingers...";
		arm->openFingers();
		state.invoke = &closeFingers;
	}

	void closeFingers(arm::Arm* arm, FingerState& state) {
		LOG(DEBUG) << "Closing fingers...";
		arm->closeFingers();
		state.invoke = &openFingers;
	}


	/**
	 * The main kinjo function.
	 **/
	int run(app::State* state, arm::Arm* arm, vision::Vision* vision,
			bool showDepthImage, bool showGrid) {
		int key = -1;
		cv::Mat depthImage, rgbImage;

		DepthView depthView;
		depthView.invoke = showDepthImage ? &enableDepthView : &disableDepthView;

		FingerState fingers;
		fingers.invoke = &openFingers;
		// TODO prevent change of finger state if grasping or calibrating
		// TODO open/close fingers asynchronously

		const cv::Scalar rgbColor(0, 255, 0);
		const cv::Scalar rgbGray(200, 200, 200);
		const cv::Scalar depthColor(255 << 8);

		MouseCallback mouseState;
		mouseState.point = cv::Point(-1, -1);
		mouseState.pointChanged = false;
		mouseState.state = state;
		cv::namedWindow(g_sWindowTitleColor);
		cv::setMouseCallback(g_sWindowTitleColor, mouseCallback, &mouseState);

		depthView.invoke(mouseState, depthView, showDepthImage);

		do
		{
			if (key == 'd')
				depthView.invoke(mouseState, depthView, showDepthImage);

			if (key == 'f')
				fingers.invoke(arm, fingers);

			if (key == 'g')
				showGrid ^= 1;

			state->process(key);
			app::State* tmp = state;
			state = state->getNext();
			if (tmp != state) {
				LOG(INFO) << "State changed from " << tmp->getDesignator() << " to " << state->getDesignator();
				state->initialize();
			}
			mouseState.state = state;

			// Update the vision images.
			vision->updateImages(false);
			vision->getDepth().copyTo(depthImage);
			vision->getRgb().copyTo(rgbImage);

			state->process(rgbImage, depthImage);

			const std::string designator = state->getDesignator();
			std::stringstream designatorStream;
			designatorStream << "STATE[ " << designator << " ]";
			cv::putText(rgbImage, designatorStream.str(), cv::Point(15, 25),
					cv::FONT_HERSHEY_SIMPLEX, 0.3, rgbColor, 1, CV_AA);

			const std::string details = state->getDetails();
			if (!details.empty()) {
				std::stringstream detailsStream;
				detailsStream << details;
				cv::putText(rgbImage, detailsStream.str(), cv::Point(15, 40),
						cv::FONT_HERSHEY_SIMPLEX, 0.3, rgbColor, 1, CV_AA);
			}

			const std::vector<std::string> infos = state->getInfos();
			renderInfos(rgbImage, infos);

			// Render current arm position(in arm coordinate system).
			cv::Vec3f const v3fArmPosition = arm->getPosition();
			cv::Point posPoint(0 + rgbImage.cols - 220, 30);
			renderPosition(rgbImage, posPoint, rgbColor, v3fArmPosition, "ARM");

			// Render a raster into the image.
			if (showGrid)
				renderRaster(rgbImage, rgbGray);

			// Show depth, color and selected point.
			if(0 <= mouseState.point.x && mouseState.point.x < depthImage.cols
				&& 0 <= mouseState.point.y && mouseState.point.y < depthImage.rows)
			{
				renderDoubleCircle(depthImage, mouseState.point, depthColor);
				renderDoubleCircle(rgbImage, mouseState.point, rgbColor);

				cv::Vec3f const v3fVisionPosition(
					vision->estimateVisionPositionFromImagePointPx(mouseState.point));
				if(v3fVisionPosition[2u] > 0)
				{
					renderPosition(depthImage, mouseState.point, depthColor, v3fVisionPosition);
					renderPosition(rgbImage, mouseState.point, rgbColor, v3fVisionPosition);
					cv::Point visPoint(0 + rgbImage.cols - 215, 45);
					renderPosition(rgbImage, visPoint, rgbColor, v3fVisionPosition, "VIS");
				}
			}

			if (showDepthImage) {
				// Scale the depth image to use the whole 16-bit and make it more visible.
				// This value seems not to be correct...
				//float const fMaxVisionDepthValue(static_cast<float>(vision->getMaxDepthValue()));
				float const fMaxVisionDepthValue(4000.0f);
				float const fImageValueScale(
					static_cast<float>(std::numeric_limits<std::uint16_t>::max())/fMaxVisionDepthValue);
				cv::imshow(g_sWindowTitleDepth, depthImage * fImageValueScale);
			}

			cv::imshow(g_sWindowTitleColor, rgbImage);

			key = cv::waitKey(g_iRefreshIntervallMs);
		} while (key != ESCAPE);

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
		std::shared_ptr<kinjo::mock::DirectoryBasedDataProvider> dataProvider;

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

		el::Configurations logConf(config.getString("Logging", "ConfigFile"));
		el::Loggers::reconfigureAllLoggers(logConf);

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
				MovGuardList.push_back(std::make_shared<kinjo::arm::CylindricMovementGuard>());
				arm = std::make_shared<kinjo::arm::JacoArm>(MovGuardList);

				// Load a random calibration point generator if the calibration is not hard coded.
				if(!bUseHardCodedCalibration) {
					std::size_t const uiRandomCalibrationPointGeneratorSeed(config.getInt("calibration", "uiRandomCalibrationPointGeneratorSeed"));
					calibrationPointGenerator = std::make_shared<kinjo::calibration::RandomCalibrationPointGenerator>(
						uiRandomCalibrationPointGeneratorSeed);
				}

				vision = std::make_shared<kinjo::vision::OpenNiVision>();

			} else {
				std::string const sMockDataDirectory(config.getString("mock", "sMockDataDirectory"));
				dataProvider = std::make_shared<kinjo::mock::DirectoryBasedDataProvider>(sMockDataDirectory);

				std::shared_ptr<kinjo::mock::TestdataMock> mock = 
					std::make_shared<kinjo::mock::TestdataMock>(dataProvider.get());

				arm = std::dynamic_pointer_cast<kinjo::arm::Arm>(mock);
				vision = std::dynamic_pointer_cast<kinjo::vision::Vision>(mock);
				calibrationPointGenerator = std::dynamic_pointer_cast<kinjo::calibration::CalibrationPointGenerator>(mock);

				cv::Vec3f position = calibrationPointGenerator->getNextCalibrationPoint();
				arm->moveTo(position);
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


		kinjo::grasp::PickStrategy* picker = new kinjo::grasp::SimpleTablePickStrategy(arm.get());
		kinjo::grasp::DropStrategy* dropper = new kinjo::grasp::BoxDropStrategy(arm.get());
		kinjo::grasp::Grasper* grasper = new kinjo::grasp::SimplePickAndDropGrasper(picker, dropper);

		std::string matrixFileName = config.getString("hardCodedCalibrator", "matrixFileName");

		bool showDepthImage = config.getBool("GUI", "ShowDepthImage");
		bool showGrid = config.getBool("GUI", "ShowGrid");

		int graspXOffset = config.getInt("grasping", "x-offset");
		int graspYOffset = config.getInt("grasping", "y-offset");
		int graspZOffset = config.getInt("grasping", "z-offset");

		kinjo::app::State* initStatePtr;
		kinjo::app::State* calibStatePtr;
		kinjo::app::State* readyStatePtr;
		kinjo::app::State* graspStatePtr;

		std::shared_ptr<kinjo::app::State> initState;
		initState = std::make_shared<kinjo::app::InitialState>(&calibStatePtr);
		std::shared_ptr<kinjo::app::State> calibState = std::make_shared<kinjo::app::ColoredCircleBasedCalibrationState>(
					&initStatePtr, &readyStatePtr, arm.get(), calibrator.get(), vision.get(), recognizer.get());
		std::shared_ptr<kinjo::app::State> readyState = std::make_shared<kinjo::app::ReadyState>(
				&calibStatePtr, &graspStatePtr, calibrator.get(), matrixFileName);
		std::shared_ptr<kinjo::app::State> graspState = std::make_shared<kinjo::app::GraspState>(&readyStatePtr,
				grasper, vision.get(), calibrator.get(), &graspXOffset, &graspYOffset, &graspZOffset);

		initStatePtr = initState.get();
		calibStatePtr = calibState.get();
		readyStatePtr = readyState.get();
		graspStatePtr = graspState.get();

//		initOffsetSliders(&graspXOffset, &graspYOffset, &graspZOffset);

		// Execute the kinjo main loop.
		return kinjo::run(initState.get(), arm.get(), vision.get(), showDepthImage, showGrid);
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

