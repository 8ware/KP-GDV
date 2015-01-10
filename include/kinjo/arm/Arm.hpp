#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
    namespace arm {

        /**
        * this is a Arm of any kind
        * not our Special Jaco
        **/
        class Arm
        {

        public:
            /**
            * \param vector the absolute position in centimetres.
            */
            virtual void moveTo(cv::Vec3f vector) = 0;

			/**
			* \param hasFingersClosed true-> fingers are closed, false -> fingers are opened
			**/
			virtual void moveToStartPosition(bool hasFingersClosed) = 0;

            /**
            * \param vector the absolute grab rotation in Degree
            */
            virtual void rotateTo(cv::Vec3f vector) = 0;

            /**
            * \param vector the relative position in centimetres.
            */
            virtual void moveBy(cv::Vec3f vector) = 0;

            /**
            * \param vector the relative Rotation in Degree.
            */
            virtual void rotateBy(cv::Vec3f vector) = 0;

			virtual void rotateHandBy(float MultiplesOfPI) = 0;

            /**
             * \return the absolute position in centimetres.
             */
            virtual cv::Vec3f getPosition() const = 0;

            /**
            * \return the absolute Rotation in Degree.
            **/
            virtual cv::Vec3f getRotation() const = 0;

            virtual void openFingers() = 0;
            virtual void closeFingers() = 0;

        };
    
    }
}

