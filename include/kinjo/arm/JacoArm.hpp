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

	/**
	 * Class which implements Arm for Kinova Jaco
	 */
    class JacoArm final : public Arm
    {

    public:
		/**
		 * Contructor
		 * \param MovGuardList a list of Movementguard implementations to
		 *        prevent Arm from coliding with objects or itself
		 * \param maxMovementErrorDeviation float which represents the 
		 *        inaccuracy the arm can have without causing an error
		 */
		JacoArm(std::list<std::shared_ptr<MovementGuard>> MovGuardList,
			float maxMovementErrorDeviation);

		/**
		 * default Destructor
		 */
		virtual ~JacoArm() = default;

        void moveTo(cv::Vec3f vector) override;

		void moveToStartPosition(bool hasFingersClosed) override;

        void rotateTo(cv::Vec3f vector) override;

        void moveBy(cv::Vec3f vector) override;

        void rotateBy(cv::Vec3f vector) override;

		void rotateHandBy(float pi) override;

        cv::Vec3f getPosition() const override;
        
		cv::Vec3f getRotation() const override;

        void openFingers() override;
        
		void closeFingers() override;

    private:
		/**
		 * Helper to wait for the arm finish moving.
		 */
		void waitArmFinishMovement() const;

		/**
         * Helper to wait for the fingers finish moving.
         */
		void waitFingersFinishMovement() const;

		/**
		 * Helper to find out if the arm is moving.
		 * \return True if the arm is moving (or tries to move).
		 */
		bool isArmMoving() const;
		
		/**
         * Helper to find out if the Fingers are moving.
         * \return True if the arm is moving (or tries to move).
         */
		bool areFingersMoving() const;

		/**
		 * Helper to get a bool if 2 float values are the same
		 * \return True if both values are the same
		 */
		bool DiffIsZero(float X, float Y) const;

		/**
		 * Helper to lower the hand carefully
		 * \param Distance to lower the Hand in Millimeter
		 */
		void LowerHand(int Distance) const;

    private:
		/**
		 * Maximum deviation between reached and desired position indicating an
		 * probable error.
		 */
		float maxMovementErrorDeviation;

		/**
		 * The reference to the JacoArm API
		 * The shared pointer represents the Jaco Arm
		 */
        std::shared_ptr<KinDrv::JacoArm> TheJacoArm;

		/**
		 * The List of movement guards that are considered
		 */
		std::list<std::shared_ptr<MovementGuard>> MovGuardList;

    };//class
} //namespace arm
} //namespace kinjo

