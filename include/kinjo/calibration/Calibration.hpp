#pragma once

#include <kinjo/arm/Arm.hpp>
#include <kinjo/vision/Vision.hpp>

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo
{
    namespace calibration
    {
        /**
        * Allows calibration of arm to vision.
        **/
        class Calibrator
        {
            /**
            * Constructor.
            **/
            Calibrator(
                arm::Arm * const pArm, 
                vision::Vision * const pVision);

			/**
			* Copy-assignment.
			**/
			Calibrator & operator=(
				Calibrator const & calibrator) = delete;

            /**
            * \return The current rigid body transformation betwween vision and arm.
            **/
			cv::Matx44f getRigidBodyTransformation() const;

            /**
            * Calibrates the vision and the arm.
            * The calibration object has to be grabbed before!
            **/
            void calibrate(
                std::size_t const uiCalibrationPointCount, 
				std::size_t const uiCalibrationRotationCount);

        private:
            /**
            * \return The averaged position of the calibration object in the vision over multiple frames.
            **/
            cv::Vec3f getAveragedCalibrationObjectVisionPosition(
                std::size_t const uiCalibrationRotationCount) const;

            /**
            * \return Estimates the rigid body trsansformation from the given point correspondences.
            **/
			static cv::Matx44f estimateRigidBodyTransformation(
				std::vector<std::pair<cv::Vec3f, cv::Vec3f>> const & vv2v3fCorrespondences);

            cv::Vec3f getRandomArmPosition() const;
            cv::Vec3f getRandomArmRotation() const;

			cv::Matx44f m_matCurrentRigidBodyTransformation;

            bool m_bCalibrationAvailable;

			arm::Arm * const m_pArm;
            vision::Vision * const m_pVision;
        };
    
    }
}