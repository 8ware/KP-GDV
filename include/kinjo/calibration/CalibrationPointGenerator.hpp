#pragma once

#include <opencv2/core/core.hpp>

namespace kinjo
{
	namespace calibration
	{
		/**
		* Generates points for calibration.
		**/
		class CalibrationPointGenerator
		{
		public:
			/**
			* \return The next calibration point.
			**/
			virtual cv::Vec3f getNextCalibrationPoint() const = 0;
		};
	}
}