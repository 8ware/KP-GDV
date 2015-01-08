#pragma once

#include <kinjo/arm/Arm.hpp>
#include <kinjo/vision/Vision.hpp>
#include <kinjo/calibration/CalibrationPointGenerator.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <thread>

namespace kinjo
{
    namespace calibration
    {
        /**
        * Allows calibration of arm to vision.
        **/
        class Calibrator
        {
		public:
            /**
             * Constructor.
             **/
            Calibrator(
                arm::Arm * const pArm, 
                vision::Vision * const pVision,
				CalibrationPointGenerator * const pCalibrationPointGenerator);

			/**
			 * Copy assignment operator.
			 **/
			Calibrator & operator=(Calibrator const &) = delete;

			/**
			 * \return If there is a valid transformation available.
			 **/
			bool getIsValidTransformationAvailable() const;

            /**
             * \return The current rigid body transformation betwween vision and arm.
             **/
			cv::Matx44f getRigidBodyTransformation() const;

            /**
             * Calibrates the vision and the arm.
             * The calibration object has to be grabbed before!
			 * This function returns immediately and performs the calibration asynchronously.
			 * The calibration has finished when getIsValidTransformationAvailable returns true.
             **/
            void calibrateAsync(
                std::size_t const uiCalibrationPointCount, 
				std::size_t const uiCalibrationRotationCount,
				std::size_t const uiRecognitionAttemptCount);

		private:

			/**
			* Calibrates the vision and the arm.
			**/
			void calibrationThreadMain(
				std::size_t const uiCalibrationPointCount,
				std::size_t const uiCalibrationRotationCount,
				std::size_t const uiRecognitionAttemptCount);

            /**
             * \return The averaged position of the calibration object in the vision over multiple frames/rotations.
			 *		   The vector is zero if it was not recognized.
             **/
            cv::Vec3f getAveragedCalibrationObjectVisionPosition(
				std::size_t const uiCalibrationRotationCount,
				std::size_t const uiRecognitionAttemptCount) const;

            /**
             * \return Estimates the rigid body transformation from the given point correspondences.
             **/
			static cv::Matx44f estimateRigidBodyTransformation(
				std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences);

		private:
			std::thread m_Thread;

			cv::Matx44f m_matCurrentRigidBodyTransformation;

            bool m_bCalibrationAvailable;

			arm::Arm * const m_pArm;
            vision::Vision * const m_pVision;
			CalibrationPointGenerator * const m_pCalibrationPointGenerator;
        };
    
    }
}
