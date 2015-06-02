#pragma once

#include <opencv2/imgproc/imgproc.hpp>

#include <kinjo/app/CalibrationState.hpp>
#include <kinjo/vision/Vision.hpp>
#include <kinjo/recognition/Recognizer.hpp>


namespace kinjo {
namespace app {

class ColoredCircleBasedCalibrationState : public CalibrationState {

public:

	ColoredCircleBasedCalibrationState(
			app::State** initState, app::State** readyState,
			arm::Arm* arm, calibration::Calibrator* calibrator,
			vision::Vision* vision, recognition::Recognizer* recognizer);

	void process(cv::Mat& rgbImage, cv::Mat& depthImage) override;

private:

	vision::Vision* vision;
	recognition::Recognizer* recognizer;

};

} // end namespace app
} // end namespace kinjo

