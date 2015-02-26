#pragma once

#include <kinjo/calibration/Calibrator.hpp>

namespace kinjo {
namespace calibration {

	/**
	 * Allows calibration of arm to vision.
	 **/
	class HardCodedCalibrator : public Calibrator
	{
	public:
		/**
		 * Constructor.
		 **/
		HardCodedCalibrator(
			cv::Matx44f const & mat44fRigidBodyTransformation);

		/**
		 * \return If there is a valid transformation available.
		 **/
		virtual bool getIsValidTransformationAvailable() const override;

		/**
		 * \return The current rigid body transformation betwween vision and arm.
		 **/
		virtual cv::Matx44f getRigidBodyTransformation() const override;
			
		/**
		 * Calibrates the vision and the arm.
		 * The calibration object has to be grabbed before!
		 * This function returns immediately and performs the calibration asynchronously.
		 * The calibration has finished when getIsValidTransformationAvailable returns true.
		 **/
		virtual void calibrateAsync() override;

		/**
		 * Actually do nothing.
		 */
		void reset() override {}

	private:
		cv::Matx44f m_mat44fRigidBodyTransformation;
	};

}
}
