#pragma once

#include <opencv2/core/core.hpp>

#include <utility>	// std::pair

namespace kinjo {
namespace recognition {

	class Recognizer
	{
	public:
		/**
		 * \return The position of the calibration object in the image.
		 **/
		virtual cv::Point estimateCalibrationObjectImagePointPx(
			cv::Mat const & matRgb) const = 0;
			
		/**
		 * \return The number of recognition attempts that should be made.
		 **/
		virtual std::size_t getRecommendedRecognitionAttempCount() const = 0;
	};

}
}