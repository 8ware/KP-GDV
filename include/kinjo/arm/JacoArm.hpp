#pragma once

#include <kinjo/arm/Arm.hpp>

#include <kinjo/arm/MovementGuard.hpp>

#include <list>

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
			JacoArm(std::list<std::shared_ptr<MovementGuard>> MovGuardList);
			virtual ~JacoArm() = default;

            /**
            * \param vector the absolute position in centimetres.
            */
            void moveTo(cv::Vec3f vector) override;

			/**
			* \param hasFingersClosed true-> fingers are closed, false -> fingers are opened
			**/
			void moveToStartPosition(bool hasFingersClosed) override;

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

			void rotateHandBy(float MultiplesOfPI) override;

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

			/**
			* Helper to find out if a line between 2 Points intersects with a Circle at 0,0,0
			* \param startPos
			* \param endPos
			* \param CircleRadius
			* \return true if intersection, false if no intersection
			**/
			bool LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos, float CircleRadius);

        private:
            std::shared_ptr<KinDrv::JacoArm> TheJacoArm;
			std::list<std::shared_ptr<MovementGuard>> MovGuardList;
			//std::shared_ptr<MovementGuard> MovGuard;
        };//class
    
    }
} //namespace

