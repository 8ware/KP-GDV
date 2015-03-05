#pragma once

#include <kinjo/arm/Arm.hpp>

#include <memory>

// forward declarations
namespace KinDrv {
	class MockArm;
}

namespace kinjo {
namespace mock {

class MockArm final : public arm::Arm
{

public:
/**
* \param config values to simulate arm behaviour
*/
MockArm(
	float offset_position,
	float offset_rotation,
	float offset_fingers,
	float initial_position_X,
	float initial_position_Y,
	float initial_position_Z,
	float initial_rotation_X,
	float initial_rotation_Y,
	float initial_rotation_Z,
	float initial_fingerposition_X,
	float initial_fingerposition_Y,
	float initial_fingerposition_Z);
	
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
* \return alway false because mockArm is instantly at goal position.
**/
bool isArmMoving() const;

/**
* Helper to get a bool if 2 float values are the same
* \return True if both values are the same
**/
bool DiffIsZero(float X, float Y) const;

private:			
	//state representation of arm
	mutable cv::Vec3f cart_pos;
	mutable cv::Vec3f cart_rot;
	mutable cv::Vec3f cart_fin;
	//accuracy reduction
	mutable float pos_offset = 0;
	mutable float rot_offset = 0;
	mutable float fin_offset = 0;
		
};//class

} //end of mock namespace
} //end of kinjo namespace
