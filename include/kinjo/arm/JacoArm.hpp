#pragma once

#include <kinjo/arm/Arm.hpp>

#include <memory>

// forward declarations
namespace KinDrv {
    class JacoArm;
}

namespace kinjo {
    namespace arm {

        class JacoArm final : public Arm
        {

        public:
            JacoArm();

            /**
            * \param vector the absolute position in centimetres.
            */
            void moveTo(cv::Vec3f vector) override;

            /**
            * \param vector the absolute grab rotation in Degree
            */
            void rotateTo(cv::Vec3f vector) override;

            /**
            * \param vector the relative position in centimetres.
            */
            void moveBy(cv::Vec3f vector) override;

            /**
            * \param vector the relative Rotation in Degree.
            */
            void rotateBy(cv::Vec3f vector) override;

            /**
            * \return the absolute position in centimetres.
            */
            cv::Vec3f getPosition() const override;
            /**
            * \return the absolute rotation in degree
            **/
            cv::Vec3f getRotation() const override;

            void openFingers() override;
            void closeFingers() override;

        private:
            /**
            * Helper to wait for the arm finish moving.
            **/
            void waitArmFinishMovement() const;
            /**
            * Helper to find out if the arm is moving.
            * \return True if the arm is moving (or tries to move).
            **/
            bool isArmMoving() const;

			/**
			* Helper to get a bool if 2 float values are the same
			* \return True if both values are the same
			**/
			bool DiffIsZero(float X, float Y) const;

        private:
            std::shared_ptr<KinDrv::JacoArm> TheJacoArm;
        };//class
    
    }
} //namespace

