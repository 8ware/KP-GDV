#pragma once

#include <queue>

#include <opencv2/imgproc/imgproc.hpp>

#include <kinjo/app/State.hpp>
#include <kinjo/arm/Arm.hpp>
#include <kinjo/vision/Vision.hpp>
#include <kinjo/calibration/Calibrator.hpp>


namespace kinjo {
namespace app {

/**
 * This class represents the grasp state of the application.
 */
class GraspState : public State {

public:

	GraspState(app::State** readyState,
			arm::Arm* arm, vision::Vision* vision, calibration::Calibrator* calibrator,
			int* xOffset, int* yOffset, int* zOffset);

	void initialize() override {}
	void process(cv::Mat& rgbImage, cv::Mat& depthImage) override;
	void process(int mouseEvent, cv::Point point) override;
	void process(int key) override;

private:

	app::State** readyState;
	
	arm::Arm* arm;
	vision::Vision* vision;
	calibration::Calibrator* calibrator;

	int* xOffset;
	int* yOffset;
	int* zOffset;

	std::queue<cv::Vec3f> queue;
	cv::Vec3f currentTargetPosition;
	bool activeThread = false;

	void graspAsync();
	void grasp();

};

} // end namespace app
} // end namespace kinjo

