#pragma once

#include <kinjo/recognition/Recognizer.hpp>

namespace kinjo
{
	namespace recognition
	{
		class ColorBasedCircleRecognizer : public Recognizer
		{
		public:
			/**
			 * \return The position of the calibration object in the image.
			 **/
			virtual cv::Point estimateCalibrationObjectImagePointPx(
				cv::Mat const & matRgb) const override;

			/**
			 * \return The position of the calibration object in the image and its radius.
			 **/
			std::pair<cv::Point, float> estimateCalibrationObjectImagePointPxAndRadius(
				cv::Mat const & matRgb) const;
			
			/**
			 * \return The number of recognition attempts that should be made.
			 **/
			virtual std::size_t getRecommendedRecognitionAttempCount() const override;
		};
	}
}