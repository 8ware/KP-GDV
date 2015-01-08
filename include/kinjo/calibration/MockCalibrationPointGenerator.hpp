#pragma once

#include <kinjo/calibration/CalibrationPointGenerator.hpp>

namespace kinjo
{
	namespace calibration
	{
		/**
		 * Generates random points for calibration.
		 **/
		class MockCalibrationPointGenerator : public CalibrationPointGenerator
		{
		public:
			/**
			 * Constructor.
			 **/
			MockCalibrationPointGenerator();

			/**
			 * \return The next calibration point.
			 **/
			virtual cv::Vec3f getNextCalibrationPoint() const override;

		private:
			std::size_t mutable m_uiCurrentPoint;
			std::vector<cv::Vec3f> m_vv3fPositions;
		};
	}
}