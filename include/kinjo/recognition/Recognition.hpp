#pragma once

#include <opencv2/imgproc/imgproc.hpp>

#include <utility>	// std::pair

namespace kinjo
{
    namespace recognition
    {
        /**
        * \return The position of the calibration object in the image.
        **/
		std::pair<cv::Vec2f, float> getCalibrationObjectVisionPositionPx(
			cv::Mat const & matRgb);
    }
}