#include <kinjo/calibration/HardCodedCalibrator.hpp>

namespace kinjo
{
    namespace calibration
    {
        /**
         *
         **/
        HardCodedCalibrator::HardCodedCalibrator()
		{}
		/**
		 *
		 **/
		bool HardCodedCalibrator::getIsValidTransformationAvailable() const
		{
			return true;
		}
        /**
         *
         **/
		cv::Matx44f HardCodedCalibrator::getRigidBodyTransformation() const
        {
			return cv::Matx44f(
				1.0f, 0.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 1.0f, -1100.0f,
				0.0f, 1.0f, 0.0f, 0.0f,
				0.0f, 0.0f, 0.0f, 1.0f);
		}
		/**
		 *
		 **/
		void HardCodedCalibrator::calibrateAsync()
		{
			// Nothing to do.
		}
	}
}