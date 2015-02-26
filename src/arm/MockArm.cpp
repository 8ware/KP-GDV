#include <kinjo/arm/MockArm.hpp>

//#include <libkindrv/kindrv.h>   //// KinDrv::MockArm

#include <thread>
#include <iostream>

namespace kinjo {
	namespace arm {

		static const float pi = 3.14159265358979323846f;

		MockArm::MockArm()
		{
			//any start values
			pos_offset = 0.1f;
			rot_offset = 0.1f;
			fin_offset = 10.0f;
			cart_pos = cv::Vec3f(0.050f, 0.050f, 0.050f);
			cart_rot = cv::Vec3f(1.0f, 1.0f, 1.0f);
			cart_fin = cv::Vec3f(10.0f, 10.0f, 10.0f); 
			
			std::cout << "MockArm found, using Mock Arm." << std::endl;
		}

		void MockArm::moveTo(cv::Vec3f vector)
		{
			//works with milimeter, accuracy is bad though
			cart_pos[0] = vector[0] / 1000.0f + pos_offset;
			cart_pos[1] = vector[1] / 1000.0f + pos_offset;
			cart_pos[2] = vector[2] / 1000.0f + pos_offset;
			waitArmFinishMovement();
		}
		void MockArm::rotateTo(cv::Vec3f vector)
		{
			cart_rot[0] = vector[0] / 1000.0f + rot_offset;
			cart_rot[1] = vector[1] / 1000.0f + rot_offset;
			cart_rot[2] = vector[2] / 1000.0f + rot_offset;
			waitArmFinishMovement();
		}
		void MockArm::moveBy(cv::Vec3f vector)
		{
			cart_pos[0] += vector[0] / 1000.0f + pos_offset;
			cart_pos[1] += vector[1] / 1000.0f + pos_offset;
			cart_pos[2] += vector[2] / 1000.0f + pos_offset;
			waitArmFinishMovement();
		}
		void MockArm::rotateBy(cv::Vec3f vector)
		{
			// TODO: Test
			cart_rot[0] += vector[0] / 1000.0f + rot_offset;
			cart_rot[1] += vector[1] / 1000.0f + rot_offset;
			cart_rot[2] += vector[2] / 1000.0f + rot_offset;

			waitArmFinishMovement();
		}

		void MockArm::openFingers()
		{
			//TODO: test if the numbers are right
			cart_fin[0] = 55 + fin_offset;
			cart_fin[1] = 55 + fin_offset;
			cart_fin[2] = 55 + fin_offset;
			waitArmFinishMovement();
		}
		void MockArm::closeFingers()
		{
			//TODO: test if the numbers are right
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

		void MockArm::waitArmFinishMovement() const
		{
			while (isArmMoving())
			{
				//don't fire too many commands at the arm
				std::this_thread::sleep_for(std::chrono::milliseconds(200));
			}
		}

		bool MockArm::isArmMoving() const
		{
			return false;
		}

		bool MockArm::DiffIsZero(float X, float Y) const{
			return (static_cast<int> (X * 1000.0f) == static_cast<int>(Y * 1000.0f));
		}
	}
}