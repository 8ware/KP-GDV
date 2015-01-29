#pragma once

#include <opencv2/core/core.hpp>

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
			 * \return If there is a valid transformation available.
			 **/
			virtual bool getIsValidTransformationAvailable() const = 0;

            /**
             * \return The current rigid body transformation betwween vision and arm.
             **/
			virtual cv::Matx44f getRigidBodyTransformation() const = 0;
			
            /**
             * Calibrates the vision and the arm.
             * The calibration object has to be grabbed before!
			 * This function returns immediately and performs the calibration asynchronously.
			 * The calibration has finished when getIsValidTransformationAvailable returns true.
             **/
            virtual void calibrateAsync() = 0;
		};
	}
}
