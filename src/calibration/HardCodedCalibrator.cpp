#include <kinjo/calibration/HardCodedCalibrator.hpp>

namespace kinjo
{
    namespace calibration
    {
        /**
         *
         **/
        HardCodedCalibrator::HardCodedCalibrator(
			cv::Matx44f const & mat44fRigidBodyTransformation):
				m_mat44fRigidBodyTransformation(mat44fRigidBodyTransformation)
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
			return m_mat44fRigidBodyTransformation;
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