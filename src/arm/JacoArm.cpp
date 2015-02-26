#pragma once

#include <kinjo/arm/JacoArm.hpp>
#include <kinjo/arm/MovementGuardOne.hpp>
#include <libkindrv/kindrv.h>   // KinDrv::JacoArm

#include <cmath>
#include <thread>
#include <iostream>

namespace kinjo {
namespace arm {

	static const float pi = std::atan2(0, -1);

	JacoArm::JacoArm(std::list<std::shared_ptr<MovementGuard>> MovGuardList)
	{
		TheJacoArm = std::make_shared<KinDrv::JacoArm>();
		this->MovGuardList = MovGuardList;
		std::cout << "JacoArm found, using Jaco Arm." << std::endl;
	}

	void JacoArm::moveTo(cv::Vec3f vector)
	{
		KinDrv::jaco_position_t position_now = TheJacoArm->get_cart_pos();
		cv::Vec3f Position_start;
		Position_start[0] = position_now.position[0];
		Position_start[1] = position_now.position[1];
		Position_start[2] = position_now.position[2];
		cv::Vec3f Position_end;
		Position_end[0] = vector[0] / 1000;
		Position_end[1] = vector[1] / 1000;
		Position_end[2] = vector[2] / 1000;
		int handling = -1;
		cv::Vec3f PosOfDetour = Position_start;
			
		for (std::list<std::shared_ptr<MovementGuard>>::iterator it = MovGuardList.begin(); it != MovGuardList.end(); it++){
			std::shared_ptr<MovementGuard> MovGuard = std::static_pointer_cast<MovementGuard>(*it);

			MovGuard->Handle_Deathzones(Position_start, Position_end, &handling, &PosOfDetour);
			if (handling != 0)
				break;
		}
		KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
		cv::Vec3f actual2;
		switch (handling)
		{
		case 0:
			// 0 everything went fine, next thing is to actually move
			//works with milimeter, accuracy is bad though
			printf("Lets Move!\n");
			position.position[0] = vector[0] / 1000;
			position.position[1] = vector[1] / 1000;
			position.position[2] = vector[2] / 1000;
			TheJacoArm->start_api_ctrl();
			TheJacoArm->set_target_cart(position.position[0], position.position[1], position.position[2],
				position.rotation[0], position.rotation[1], position.rotation[2],
				position.finger_position[0], position.finger_position[1], position.finger_position[2]);
			waitArmFinishMovement();
			TheJacoArm->set_target_cart(position.position, position.finger_position);
			//waitArmFinishMovement();
			TheJacoArm->stop_api_ctrl();
			std::printf("position we were to move to: %.4f,%.4f,%.4f\n", vector[0], vector[1], vector[2]);
			actual2 = getPosition();
			std::printf("moving done. New   Position: %.4f,%.4f,%.4f\n",
				actual2[0], actual2[1], actual2[2]); 

			if (std::sqrtf((vector[0] - actual2[0]) * (vector[0] - actual2[0])) > 30 ||
				std::sqrtf((vector[1] - actual2[1]) * (vector[1] - actual2[1])) > 30 ||
				std::sqrtf((vector[2] - actual2[2]) * (vector[2] - actual2[2])) > 30) {
				printf("movement failed! RETRY\n");
				// printf("status: %s\n", TheJacoArm->get_status());
				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				moveTo(getPosition());
				moveTo(vector);
				//moveToStartPosition(true);
			}
			break;
		case 1:
			// 1 simple detour(no problem)
			printf("taking detour\n");
			moveTo(PosOfDetour * 1000);
			moveTo(Position_end * 1000);
			break;
		case 2:
			// 2 endpoint in a deadzone
			printf("Endpoint is in deadzone\n");
			break;
		case 3:
			// 3 startpoint in a deadzone
			printf("Startpoint is in a deadzone\n");
			break;
		default:
			printf("Something went terribly wrong in deadzone handling!\n");
			break;
		}

		//If a Intersection took Place, we dont Move!
		//LineCircleIntersection Calls moveTo for a better way around the dead zone or
		//doesn't in case endpoint is inside Deadzone

#ifdef _DEBUG
		cv::Vec3f actual = getPosition();
		std::printf("moving done. New \"exact\" Position: %.4f,%.4f,%.4f\n",
			actual[0], actual[1], actual[2]);
#endif


	} //moveto

	void JacoArm::moveToStartPosition(bool hasFingersClosed){
		//move arm and rotate it to the starting position we want.
		TheJacoArm->start_api_ctrl();
		if (!hasFingersClosed) {
			moveTo({ 0.0f, -500.0f, 400.0f });
			waitArmFinishMovement();
			TheJacoArm->set_target_cart(0.0f, -0.5f, 0.4f, pi, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
		}
		else {
			moveTo({ 0.0f, -500.0f, 400.0f });
			waitArmFinishMovement();
			TheJacoArm->set_target_cart(0.0f, -0.5f, 0.4f, pi, 0.0f, 0.0f, 40.0f, 40.0f, 40.0f);
		}
		waitArmFinishMovement();
		TheJacoArm->stop_api_ctrl();
	}

	void JacoArm::rotateTo(cv::Vec3f vector)
	{
		TheJacoArm->start_api_ctrl();
		KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
		//TODO: calculate Degree vector to euler
		position.rotation[0] = vector[0];
		position.rotation[1] = vector[1];
		position.rotation[2] = vector[2];
		TheJacoArm->set_target_cart(
			position.position[0], position.position[1], position.position[2],
			position.rotation[0], position.rotation[1], position.rotation[2],
			position.finger_position[0], position.finger_position[1], position.finger_position[2]);
		waitArmFinishMovement();
		TheJacoArm->stop_api_ctrl();

	}
	void JacoArm::moveBy(cv::Vec3f vector)
	{
		TheJacoArm->start_api_ctrl();
		KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
		position.position[0] += vector[0] / 1000;
		position.position[1] += vector[1] / 1000;
		position.position[2] += vector[2] / 1000;
		TheJacoArm->set_target_cart(position.position, position.finger_position);
		waitArmFinishMovement();
		TheJacoArm->stop_api_ctrl();
	}
	void JacoArm::rotateBy(cv::Vec3f vector)
	{
		TheJacoArm->start_api_ctrl();
		std::printf("rotating to %f,%f,%f ...\n", vector[0], vector[1], vector[2]);
		// vector 0-360 0-360 0-360
		KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
		//TODO: calculate Degree vector to euler
		position.rotation[0] += vector[0];
		position.rotation[1] += vector[1];
		position.rotation[2] += vector[2];
		TheJacoArm->set_target_cart(
			position.position[0], position.position[1], position.position[2],
			position.rotation[0], position.rotation[1], position.rotation[2],
			position.finger_position[0], position.finger_position[1], position.finger_position[2]);
		waitArmFinishMovement();

		cv::Vec3f actual = getRotation();
		std::printf("moving done. New \"exact\" Rotation: %.4f,%.4f,%.4f\n",
			actual[0], actual[1], actual[2]);

		TheJacoArm->stop_api_ctrl();

	}


	void JacoArm::rotateHandBy(float MultiplesOfPI)
	{
		TheJacoArm->start_api_ctrl();
		KinDrv::jaco_position_t position = TheJacoArm->get_ang_pos();
		KinDrv::jaco_position_t position2 = TheJacoArm->get_cart_pos();
		printf("joints: %f,%f,%f,%f,%f,%f \n", position.joints[0], position.joints[1],
			position.joints[2], position.joints[3], position.joints[4],	position.joints[5]);
		position.joints[1] -= 1.4; //this offset prevents the arm from dropping
		position.joints[2] += 0.6; //this offset prevents the arm from dropping
		position.joints[5] += 180 * MultiplesOfPI / pi;
		//printf("MultiplesOfPI: %f\n", MultiplesOfPI);
		//printf("rotation: %f\n", (position.joints[5]+(180 * MultiplesOfPI / pi)));
		if (position.joints[5] > 8000 || position.joints[5] < -8000){
			printf("WARNING: AND ROTATION NEAR MAXIMUM! RESTART ARM OR EXPECT WRONG HANDROTATION SOON");
		}
		if (position.joints[5] > 9500){
			printf("WARNING: AND ROTATION NEAR MAXIMUM! ARM WOULD CRASH SOON, ROTATE HAND BACK TO NORMAL");
			position.joints[5] = 360;
		}

		TheJacoArm->set_target_ang(position.joints, position2.finger_position);
		waitArmFinishMovement();
		TheJacoArm->stop_api_ctrl();
		TheJacoArm->set_control_cart();
	}

	cv::Vec3f JacoArm::getRotation() const
	{
		cv::Vec3f rotation;
		rotation[0] = TheJacoArm->get_cart_pos().rotation[0];
		rotation[1] = TheJacoArm->get_cart_pos().rotation[1];
		rotation[2] = TheJacoArm->get_cart_pos().rotation[2];
		return rotation;
	}
	cv::Vec3f JacoArm::getPosition() const
	{
		cv::Vec3f position;
		// *1000 makes it Milimeter
		position[0] = TheJacoArm->get_cart_pos().position[0] * 1000;
		position[1] = TheJacoArm->get_cart_pos().position[1] * 1000;
		position[2] = TheJacoArm->get_cart_pos().position[2] * 1000;
		return position;
	}

	void JacoArm::openFingers()
	{
		TheJacoArm->start_api_ctrl();
		KinDrv::jaco_position_t position = TheJacoArm->get_ang_pos();
		position.finger_position[0] = 0;
		position.finger_position[1] = 0;
		position.finger_position[2] = 0;
		//printf("joints: %f,%f,%f,%f,%f,%f \n", position.joints[0], position.joints[1], position.joints[2],
		//	position.joints[3], position.joints[4], position.joints[5]);
		position.joints[1] -= 1.4; //this offset prevents the arm from dropping
		position.joints[2] += 0.6; //this offset prevents the arm from dropping
		//TheJacoArm->set_target_cart(position.position, position.finger_position);
		TheJacoArm->set_target_ang(position.joints, position.finger_position);
		waitFingersFinishMovement();
		TheJacoArm->stop_api_ctrl();
		TheJacoArm->set_control_cart();
	}
	void JacoArm::closeFingers()
	{
		TheJacoArm->start_api_ctrl();
		KinDrv::jaco_position_t position = TheJacoArm->get_ang_pos();
		position.finger_position[0] = 55;
		position.finger_position[1] = 55;
		position.finger_position[2] = 55;
		//printf("joints: %f,%f,%f,%f,%f,%f \n", position.joints[0], position.joints[1],	position.joints[2],
		//	position.joints[3],	position.joints[4], position.joints[5]);
		position.joints[1] -= 1.4; //this offset prevents the arm from dropping
		position.joints[2] += 0.6; //this offset prevents the arm from dropping
		TheJacoArm->set_target_ang(position.joints, position.finger_position);
		waitFingersFinishMovement();
		TheJacoArm->stop_api_ctrl();
		TheJacoArm->set_control_cart();
	}

	void JacoArm::waitArmFinishMovement() const
	{
		while (isArmMoving())
		{
			//don't fire too many commands at the arm
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}

	
	bool JacoArm::isArmMoving() const
	{
		KinDrv::jaco_position_t position1 = TheJacoArm->get_ang_pos();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		KinDrv::jaco_position_t position2 = TheJacoArm->get_ang_pos();
		//Position
		if (!DiffIsZero(position1.position[0], position2.position[0]) ||
			!DiffIsZero(position1.position[1], position2.position[1]) ||
			!DiffIsZero(position1.position[2], position2.position[2]) ||
			//Rotation
			!DiffIsZero(position1.rotation[0], position2.rotation[0]) ||
			!DiffIsZero(position1.rotation[1], position2.rotation[1]) ||
			!DiffIsZero(position1.rotation[2], position2.rotation[2]))
			return true;
		return false;
	}

	void JacoArm::waitFingersFinishMovement() const {
		while (areFingersMoving()){
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}
	}
	bool JacoArm::areFingersMoving() const {
		KinDrv::jaco_position_t position1 = TheJacoArm->get_cart_pos();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		KinDrv::jaco_position_t position2 = TheJacoArm->get_cart_pos();
		if (!DiffIsZero(position1.finger_position[0], position2.finger_position[0]) ||
			!DiffIsZero(position1.finger_position[1], position2.finger_position[1]) ||
			!DiffIsZero(position1.finger_position[2], position2.finger_position[2]))
			return true;
		return false;
	}

	bool JacoArm::DiffIsZero(float X, float Y) const{
		return (static_cast<int> (X * 1000) == static_cast<int>(Y * 1000));
	}

	void JacoArm::GrabItem(cv::Vec3f ItemPosition){
		moveToStartPosition(false);
		openFingers();

		cv::Vec3f const cap = getPosition();
		cv::Vec3f const zInvariant(ItemPosition[0], ItemPosition[1], cap[2]+200);
		moveTo(zInvariant);
		moveTo(ItemPosition);
		closeFingers();
		moveToStartPosition(true);

		printf("Grab Item done! \n");
	}

	void JacoArm::DropItem(cv::Vec3f DropPosition, int DropHeight){
		moveToStartPosition(true);

		cv::Vec3f const cap = getPosition();
		cv::Vec3f const zInvariant(DropPosition[0], DropPosition[1], cap[2]);
		moveTo(zInvariant);
		DropPosition[2] += DropHeight;
		moveTo(DropPosition);
		openFingers();
		moveToStartPosition(false);
		printf("Drop Item done! \n");
	}

}//arm
}//kinjo

