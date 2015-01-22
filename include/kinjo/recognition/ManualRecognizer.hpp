#pragma once

#include <kinjo/recognition/Recognizer.hpp>

namespace kinjo
{
	namespace recognition
	{
		class ManualRecognizer : public Recognizer
		{
		public:
			/**
			 * \return The position of the calibration object in the image and its radius.
			 **/
			virtual cv::Point estimateCalibrationObjectImagePointPx(
				cv::Mat const & matRgb) const override;
		};
	}
}