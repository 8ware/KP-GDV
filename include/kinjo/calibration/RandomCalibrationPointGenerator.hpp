#pragma once

#include <kinjo/calibration/CalibrationPointGenerator.hpp>

namespace kinjo
{
	namespace calibration
	{
		/**
		 * Generates random points for calibration.
		 **/
		class RandomCalibrationPointGenerator : public CalibrationPointGenerator
		{
		public:
			/**
			 * Constructor.
			 **/
			RandomCalibrationPointGenerator(
				std::size_t const & uiSeed);

			/**
			 * \return The next calibration point.
			 **/
			virtual cv::Vec3f getNextCalibrationPoint() const override;

		private:
			cv::RNG mutable m_Rng;
		};
	}
}