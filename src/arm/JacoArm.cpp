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
			// TODO: Test
			KinDrv::jaco_position_t position = TheJacoArm->get_cart_pos();
			position.position[0] = vector[0]/100;
			position.position[1] = vector[1]/100;
			position.position[2] = vector[2]/100;
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
			position.position[0] += vector[0] / 100;
			position.position[1] += vector[1] / 100;
			position.position[2] += vector[2] / 100;
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
            // *100 makes it Centimeter
            position[0] = TheJacoArm->get_cart_pos().position[0]*100;
            position[1] = TheJacoArm->get_cart_pos().position[1]*100;
            position[2] = TheJacoArm->get_cart_pos().position[2]*100;
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
            //usualy the arm returns exact the same coordinates if it did not move
            //however we are dealing with floats here, better turn them to integer
            /////Position
            int X_diff = static_cast<int> ((position1.position[0] * 1000) - (position2.position[0] * 1000));
            int Y_diff = static_cast<int> ((position1.position[1] * 1000) - (position2.position[1] * 1000));
            int Z_diff = static_cast<int> ((position1.position[2] * 1000) - (position2.position[2] * 1000));
            if(X_diff != 0 || Y_diff != 0 || Z_diff != 0)
                return true;
            /////Rotation
            X_diff = static_cast<int> ((position1.rotation[0] * 1000) - (position2.rotation[0] * 1000));
            Y_diff = static_cast<int> ((position1.rotation[1] * 1000) - (position2.rotation[1] * 1000));
            Z_diff = static_cast<int> ((position1.rotation[2] * 1000) - (position2.rotation[2] * 1000));
            if(X_diff != 0 || Y_diff != 0 || Z_diff != 0)
                return true;
            return false;
        }
    
    }
}

