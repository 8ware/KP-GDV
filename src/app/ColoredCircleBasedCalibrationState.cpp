#include <kinjo/app/ColoredCircleBasedCalibrationState.hpp>

#include <kinjo/recognition/ColorBasedCircleRecognizer.hpp>
#include <kinjo/RenderHelper.hpp>


namespace kinjo {
namespace app {

ColoredCircleBasedCalibrationState::ColoredCircleBasedCalibrationState(
		app::State** initState, app::State** readyState,
		arm::Arm* arm, calibration::Calibrator* calibrator,
		vision::Vision* vision, recognition::Recognizer* recognizer)
: CalibrationState(initState, readyState, arm, calibrator) {
	this->vision = vision;
	this->recognizer = recognizer;
}

void ColoredCircleBasedCalibrationState::process(cv::Mat& rgbImage, cv::Mat& depthImage) {
	CalibrationState::process(rgbImage, depthImage);

	auto const cbcRecognizer(dynamic_cast<recognition::ColorBasedCircleRecognizer*>(recognizer));
	if(cbcRecognizer)
	{
		std::pair<cv::Point, float> const calibrationObjectPositionPx
				= cbcRecognizer->estimateCalibrationObjectImagePointPxAndRadius(rgbImage);

		// If recognition was successfull.
		if(calibrationObjectPositionPx.second > 0.0f)
		{
			cv::Point const v2iCenter(calibrationObjectPositionPx.first);
			int const iRadius(cvRound(calibrationObjectPositionPx.second));
			// Draw the center point.
			cv::circle(rgbImage, v2iCenter, 3, cv::Scalar(0, 255, 0), -1, CV_AA, 0);
			// Draw the circle.
			cv::circle(rgbImage, v2iCenter, iRadius, cv::Scalar(255, 0, 0), 3, CV_AA, 0);

			// Get the 3d position from the 2d point.
			cv::Vec3f const v3fVisionPosition
					= vision->estimateVisionPositionFromImagePointPx(v2iCenter);

			// Display its coordinates.
			renderPosition(depthImage, v2iCenter, cv::Scalar(255 << 8), v3fVisionPosition);
			renderPosition(rgbImage, v2iCenter, cv::Scalar(0, 255, 0), v3fVisionPosition);
		}
	}
}

} // end namespace app
} // end namespace kinjo

