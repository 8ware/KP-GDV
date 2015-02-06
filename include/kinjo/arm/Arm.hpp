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
		* \param CloseYourFingers true-> fingers will close, false -> fingers will open
		**/
		virtual void moveToStartPosition(bool CloseYourFingers) = 0;

        /**
        * \param vector the relative position in centimetres.
        */
        virtual void moveBy(cv::Vec3f vector) = 0;

		/**
		* \param vector the absolute grab rotation in Degree
		*/
		virtual void rotateTo(cv::Vec3f vector) = 0;

        /**
        * \param vector the relative Rotation in Degree.
        */
        virtual void rotateBy(cv::Vec3f vector) = 0;

		/**
		*  rotate hand only
		* \param negative left rotation, positive right rotation
		*/
		virtual void rotateHandBy(float MultiplesOfPI) = 0;

        /**
            * \return the absolute position in centimetres.
            */
        virtual cv::Vec3f getPosition() const = 0;

        /**
        * \return the absolute Rotation in Degree.
        **/
        virtual cv::Vec3f getRotation() const = 0;

		/**
		* open fingers
		**/
        virtual void openFingers() = 0;
		/**
		* close fingers
        **/
		virtual void closeFingers() = 0;

		/**
		* /param ItemPostion, Position of the Item in the Arms Coordinate System!
		**/
		virtual void GrabItem(cv::Vec3f ItemPosition) = 0;

		/**
		* /param DropPosition, position where to Drop the Item, in Arm coordinate System
		* /param DropHeight, height in which the Item will be dropped in milimeter
		* for example a Bottle will need a higher dropHeight than the tennisball, or if we drop the ball inside a Box
		* we need to add even more height
		* It is recommended to add a Dephtoffset in vision when placing the ball into a Box
		**/
		virtual void DropItem(cv::Vec3f DropPosition, int DropHeight) = 0;
    };
    
}
}

