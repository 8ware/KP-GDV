#pragma once

#include <kinjo/arm/JacoArm.hpp>

#include <libkindrv/kindrv.h>   // KinDrv::JacoArm

#include <thread>
#include <iostream>					// std::cout

namespace kinjo {
	namespace arm {

		static const float pi = 3.14159265358979323846f;

        JacoArm::JacoArm()
        {
			try {
				TheJacoArm = std::make_shared<KinDrv::JacoArm>();
				std::cout << "JacoArm found, using Jaco Arm." << std::endl;
				initialized = true;
			}
			catch (KinDrv::KinDrvException e)
			{
				std::cout << "No Jaco Arm found." << std::endl;
				initialized = false;
			}
        }

        void JacoArm::moveTo(cv::Vec3f vector)
        {
			//works with milimeter, accuracy is bad though
			KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
			position.position[0] = vector[0]/1000;
			position.position[1] = vector[1]/1000;
			position.position[2] = vector[2]/1000;
			TheJacoArm->start_api_ctrl();
			TheJacoArm->set_target_cart(position.position, position.finger_position);
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
            // TODO: implement!
        }

        cv::Vec3f JacoArm::getRotation() const
        {
            return cv::Vec3f();
        }
        cv::Vec3f JacoArm::getPosition() const
        {
            cv::Vec3f position;
            // *1000 makes it Milimeter
            position[0] = TheJacoArm->get_cart_pos().position[0]*1000;
            position[1] = TheJacoArm->get_cart_pos().position[1]*1000;
            position[2] = TheJacoArm->get_cart_pos().position[2]*1000;
            return position;
        }

        void JacoArm::openFingers()
        {
            KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
            //TODO: test if the numbers are right
            position.finger_position[0] = 55;
            position.finger_position[1] = 55;
            position.finger_position[2] = 55;
            TheJacoArm->set_target_cart(position.position, position.finger_position);
            waitArmFinishMovement();
        }
        void JacoArm::closeFingers()
        {
            KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
            //TODO: test if the numbers are right
            position.finger_position[0] = 0;
            position.finger_position[1] = 0;
            position.finger_position[2] = 0;
            TheJacoArm->set_target_cart(position.position, position.finger_position);
            waitArmFinishMovement();
        }

        void JacoArm::waitArmFinishMovement() const
        {
            while(isArmMoving())
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
    
    }
}

