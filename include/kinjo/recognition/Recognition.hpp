#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo
{
    namespace recognition
    {
        /**
        * \return The position of the calibration object in the image.
        **/
		cv::Vec3f getCalibrationObjectVisionPosition(
			cv::Mat const & matRgb,
			cv::Mat const & matDepth);
    }
}