#pragma once

#include <kinjo/calibration/MockCalibrationPointGenerator.hpp>

namespace kinjo
{
	namespace calibration
	{
		/**
		 *
		 **/
		MockCalibrationPointGenerator::MockCalibrationPointGenerator() :
			m_uiCurrentPoint(0u)
		{
			m_vv3fPositions = std::vector<cv::Vec3f>{
				cv::Vec3f(150.0f, 150.0f, 100.0f),	// \TODO: Replace with valid positions.
				cv::Vec3f(-150.0f, 150.0f, 100.0f),
				cv::Vec3f(-150.0f, -150.0f, 100.0f),
				cv::Vec3f(150.0f, -150.0f, 100.0f)};
		}

		/**
		 * 
		 **/
		cv::Vec3f MockCalibrationPointGenerator::getNextCalibrationPoint() const
		{
			// For now just sequentially iterate through the available positions.
			return m_vv3fPositions[(m_uiCurrentPoint++) % m_vv3fPositions.size()];
		}
	}
}