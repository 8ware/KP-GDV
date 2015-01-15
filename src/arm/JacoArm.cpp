#pragma once

#include <kinjo/arm/JacoArm.hpp>

#include <libkindrv/kindrv.h>   // KinDrv::JacoArm

#include <thread>
#include <iostream>

namespace kinjo {
	namespace arm {

		static const float pi = 3.14159265358979323846f;

		JacoArm::JacoArm()
		{
			TheJacoArm = std::make_shared<KinDrv::JacoArm>();
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
			Position_end[0] = vector[0]/1000;
			Position_end[1] = vector[1]/1000;
			Position_end[2] = vector[2]/1000;
			if (LineCircleIntersection(Position_start, Position_end, 0.2f)){
				//If a Intersection took Place, we dont Move!
				//LineCircleIntersection Calls moveTo for a better way around the dead zone or
				//doesn't in case endpoint is inside Deadzone
#ifdef _DEBUG
				std::printf("Arm took detour\n");
#endif
			}
			else{
#ifdef _DEBUG
				std::printf("moving to %f,%f,%f ...\n", vector[0], vector[1], vector[2]);
#endif
				//works with milimeter, accuracy is bad though
				KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
				position.position[0] = vector[0] / 1000;
				position.position[1] = vector[1] / 1000;
				position.position[2] = vector[2] / 1000;
				TheJacoArm->start_api_ctrl();
				TheJacoArm->set_target_cart(position.position, position.finger_position);
				waitArmFinishMovement();
				TheJacoArm->stop_api_ctrl();
#ifdef _DEBUG
				cv::Vec3f actual = getPosition();
				std::printf("moving done. New \"exact\" Position: %.4f,%.4f,%.4f\n",
					actual[0], actual[1], actual[2]);
#endif
			} //else

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


		}
		void JacoArm::moveBy(cv::Vec3f vector)
		{
			// TODO: Test
			KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
			position.position[0] += vector[0] / 1000;
			position.position[1] += vector[1] / 1000;
			position.position[2] += vector[2] / 1000;
			TheJacoArm->set_target_cart(position.position, position.finger_position);
			waitArmFinishMovement();
		}
		void JacoArm::rotateBy(cv::Vec3f vector)
		{
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

		}


		void JacoArm::rotateHandBy(float MultiplesOfPI)
		{
			KinDrv::jaco_joystick_axis_t axes = { 0 };

			float waittime;
			//decide which way to turn

			if (MultiplesOfPI < 0) {
				axes.wrist_rot = -2.5f;
				waittime = (MultiplesOfPI / pi) * -1;
			}
			else {
				axes.wrist_rot = 2.5f;
				waittime = (MultiplesOfPI / pi);
			}

			TheJacoArm->start_api_ctrl();
			//wait 50ms to make sure wrist rot is set correctly
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			TheJacoArm->move_joystick_axis(axes);
			//TODO rotating the arm for 8 happens to be 'exactly' 180 degrees or 360, depends on... find out!
			std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(waittime * 4000)));
			TheJacoArm->release_joystick();
			TheJacoArm->stop_api_ctrl();
			axes.wrist_rot = 0.f;
			waitArmFinishMovement();
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
			KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
			//TODO: test if the numbers are right
			position.finger_position[0] = 0;
			position.finger_position[1] = 0;
			position.finger_position[2] = 0;
			TheJacoArm->set_target_cart(position.position, position.finger_position);
			waitArmFinishMovement();
		}
		void JacoArm::closeFingers()
		{
			KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
			//TODO: test if the numbers are right
			position.finger_position[0] = 55;
			position.finger_position[1] = 55;
			position.finger_position[2] = 55;
			TheJacoArm->set_target_cart(position.position, position.finger_position);
			waitArmFinishMovement();
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

		bool JacoArm::DiffIsZero(float X, float Y) const{
			return (static_cast<int> (X * 1000) == static_cast<int>(Y * 1000));
		}

		bool JacoArm::LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos, float CircleRadius){
			//check if start or endpoint are inside the circle
			if (sqrt(startPos[0] * startPos[0] + startPos[1] * startPos[1] + startPos[2] * startPos[2]) < CircleRadius) {
				printf("StartPos inside Deadzone!\n");
				return true;
			}
			if (sqrt(endPos[0] * endPos[0] + endPos[1] * endPos[1] + endPos[2] * endPos[2]) < CircleRadius){
				printf("EndPos inside Deadzone! \n");
				return true;
			}
					

			//now that we know both points are outside the circle, lets begin
			//g: x-> = S + Lambda * (E - S) = S + Lambda * B
			//define short variable names
			//calc Vector B
			cv::Vec3f S = startPos,
				B = endPos - startPos,
				P; //P is Point on g closest to 0 0 0 or 0 0 0 in the worst case
			float Lambda;

			//in case StartPoint and EndPoint are the same
			if (B[0] == 0.0f && B[1] == 0.0f)
				return false;

			//since we ignore the z level:
			S[2] = 0.0f;
			B[2] = 0.0f;
			//calc closest point on g to 0,0,0
			Lambda = (0 - S.dot(B)) / (B.dot(B));
			P = S + Lambda * B;
			float distance = sqrt(P.dot(P)); //Distance between 0 0 0 and the closest point
			//if thats lesser than the Radius AND between Startpoint and Enpoint, we have an intersection
			if (distance < CircleRadius && Lambda>0.0f && Lambda < 1.0f){
				
				//now we know we intersect the deadzone
				//So we tell the system to move to another position first

				//we need a new z coordinate
				S = startPos;
				B = endPos - startPos;
				Lambda = (0 - S.dot(B)) / (B.dot(B));
				P[2] = S[2] + Lambda * B[2];
				//Normalize
				if (sqrt(P.dot(P)) == 0){
					//in this case P is exactly on 0,0,0, so we need a Plan B
					//very unlikely but possible...
					//we rotate direction Vector B 90 degree
					P[0] = B[1];
					P[1] = B[0];
				}

				P = P / sqrt(P.dot(P));
				printf("Calculating Detour\n");
				moveTo(P * CircleRadius * 2.3f * 1000);
				moveTo(endPos*1000);
				return true;
			}
			return false;
		}
	}//arm

}//kinjo

