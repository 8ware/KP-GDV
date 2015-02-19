#pragma once

#include <kinjo/calibration/RandomCalibrationPointGenerator.hpp>

namespace kinjo {
namespace calibration {

	/**
	 *
	 **/
	RandomCalibrationPointGenerator::RandomCalibrationPointGenerator(
		std::size_t const & uiSeed) :
			m_Rng(uiSeed)
	{}
	/**
	 * 
	 **/
	cv::Vec3f RandomCalibrationPointGenerator::getNextCalibrationPoint() const
	{
		auto const fPi(std::atan2(0, -1));
		// Clock-wise rotation angle when looking from the top. 
		auto const fTheta(m_Rng.uniform(0.0, 2.0*fPi));
		// Because we can not come too close to the arm base we have to keep a minimum distance.
		auto const fDist(m_Rng.uniform(350.0, 450.0));

		return cv::Vec3f(
			static_cast<float>(fDist * std::cos(fTheta)),
			static_cast<float>(fDist * std::sin(fTheta)),
			static_cast<float>(m_Rng.uniform(50.0, 300.0)));
	}

}
}