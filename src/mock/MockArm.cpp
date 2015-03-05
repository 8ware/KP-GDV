#include <kinjo/mock/MockArm.hpp>

//#include <libkindrv/kindrv.h>   //// KinDrv::MockArm

#include <thread>
#include <iostream>

namespace kinjo {
namespace mock {

/**
* \param parameters to simulate arm behaviour	
**/
MockArm::MockArm(	
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
	float initial_fingerposition_Z)
{
	pos_offset = offset_position;
	rot_offset = offset_rotation;
	fin_offset = offset_fingers;
	cart_pos = cv::Vec3f(initial_fingerposition_X, initial_fingerposition_Y, initial_fingerposition_Z);
	cart_rot = cv::Vec3f(initial_rotation_X, initial_rotation_Y, initial_rotation_Z);
	cart_fin = cv::Vec3f(initial_fingerposition_X, initial_fingerposition_Y, initial_fingerposition_Z); 
		
	std::cout << "MockArm found, using Mock Arm." << std::endl;
}

/**
* moves mockArm to vector with inaccuracy offset
**/
void MockArm::moveTo(cv::Vec3f vector)
{
	cart_pos[0] = vector[0] / 1000.0f + pos_offset;
	cart_pos[1] = vector[1] / 1000.0f + pos_offset;
	cart_pos[2] = vector[2] / 1000.0f + pos_offset;
	waitArmFinishMovement();
}

/**
* rotates mockArm to vector with inaccuracy offset
**/
void MockArm::rotateTo(cv::Vec3f vector)
{
	cart_rot[0] = vector[0] / 1000.0f + rot_offset;
	cart_rot[1] = vector[1] / 1000.0f + rot_offset;
	cart_rot[2] = vector[2] / 1000.0f + rot_offset;
	waitArmFinishMovement();
}

/**
* moves mockArm by vector with inaccuracy offset
**/
void MockArm::moveBy(cv::Vec3f vector)
{
	cart_pos[0] += vector[0] / 1000.0f + pos_offset;
	cart_pos[1] += vector[1] / 1000.0f + pos_offset;
	cart_pos[2] += vector[2] / 1000.0f + pos_offset;
	waitArmFinishMovement();
}

/**
* rotates mockArm by vector with inaccuracy offset
**/
void MockArm::rotateBy(cv::Vec3f vector)
{
	cart_rot[0] += vector[0] / 1000.0f + rot_offset;
	cart_rot[1] += vector[1] / 1000.0f + rot_offset;
	cart_rot[2] += vector[2] / 1000.0f + rot_offset;

	waitArmFinishMovement();
}

/**
* opens mockArm fingers to vector with inaccuracy offset
**/
void MockArm::openFingers()
{
	cart_fin[0] = 55 + fin_offset;
	cart_fin[1] = 55 + fin_offset;
	cart_fin[2] = 55 + fin_offset;
	waitArmFinishMovement();
}

/**
* closes mockArm fingers to vector with inaccuracy offset
**/
void MockArm::closeFingers()
{
	cart_fin[0] = 0 + fin_offset;
	cart_fin[1] = 0 + fin_offset;
	cart_fin[2] = 0 + fin_offset;
	waitArmFinishMovement();
}

cv::Vec3f MockArm::getRotation() const
{
	return cart_rot;
}

cv::Vec3f MockArm::getPosition() const
{
	return cart_pos;
}

/**
* wait method to make system wait until mockArm finished its current movement
**/
void MockArm::waitArmFinishMovement() const
{
	while (isArmMoving())
	{
		//don't fire too many commands at the arm
		std::this_thread::sleep_for(std::chrono::milliseconds(200));
	}
}

/**
* \return always false because mockArm is instantly at goal position
**/
bool MockArm::isArmMoving() const
{
	return false;
}

/**
* helper method to check if X, Y are equal
**/
bool MockArm::DiffIsZero(float X, float Y) const{
	return (static_cast<int> (X * 1000.0f) == static_cast<int>(Y * 1000.0f));
}
}//end of namespace mock
}//end of namespace kinjo
