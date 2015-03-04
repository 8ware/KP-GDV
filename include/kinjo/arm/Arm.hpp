#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
namespace arm {

    /** A virtual Arm
	 *
	 * This is a virtual Arm interface class which needs to be implemented
	 * by a real arm
	 * In this Project the Arm is implemented by JacoArm
	 */
    class Arm
    {

    public:
        /**
         * A member function which sends the Arm to the desired Position
		 * in the Arms cartesian coordinate system
		 * \param vector the absolute position in millimeters.
         */
        virtual void moveTo(cv::Vec3f vector) = 0;

		/**
		 * This member function sends the Arm to a designated StartPosition.
		 * \param CloseYourFingers true-> fingers close, false -> fingers open
		 */
		virtual void moveToStartPosition(bool CloseYourFingers) = 0;

        /**
		 * member function to move the arm for a certain distance
         * \param vector the relative position in millimeters.
         */
        virtual void moveBy(cv::Vec3f vector) = 0;

		/**
		 * member function to rotate the hand(yaw,pitch,rotation)
		 * \param vector the absolute grab rotation
		 */
		virtual void rotateTo(cv::Vec3f vector) = 0;

        /**
		 * member function to rotate the Arm relatively
         * \param vector the relative Rotation
         */
        virtual void rotateBy(cv::Vec3f vector) = 0;

		/**
		 * rotate hand only
		 * \param negative left rotation, positive right rotation
		 */
		virtual void rotateHandBy(float MultiplesOfPI) = 0;

        /**
         * \return the absolute position in millimeters.
         */
        virtual cv::Vec3f getPosition() const = 0;

        /**
         * \return the absolute Rotation in Degree.
         */
        virtual cv::Vec3f getRotation() const = 0;

		/**
		 * open fingers
		 */
        virtual void openFingers() = 0;

		/**
		 * close fingers
         */
		virtual void closeFingers() = 0;

    };
    
}
}

