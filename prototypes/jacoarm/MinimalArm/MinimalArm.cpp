// MinimalArm.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//
#include <iostream>           // std::cout
#include <memory>             // std::unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm
#include <thread>


KinDrv::jaco_position_t ArmMoveTo(KinDrv::JacoArm* arm, float Position[3], float Rotation[3], float Finger[3]){
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();
	Position[0] = Position[0] * 0.01;
	Position[1] = Position[1] * 0.01;
	Position[2] = Position[2] * 0.01;
	Pos1.position[0] = Position[0];
	Pos1.position[1] = Position[1];
	Pos1.position[2] = Position[2];

	Pos1.rotation[0] = Rotation[0];
	Pos1.rotation[1] = Rotation[1];
	Pos1.rotation[2] = Rotation[2];

	Pos1.finger_position[0] = Finger[0];
	Pos1.finger_position[1] = Finger[1];
	Pos1.finger_position[2] = Finger[2];

	KinDrv::jaco_position_t PosNow = arm->get_cart_pos();
	float Diff_Position[3] = { 5, 5, 5 };
	float Diff_Rotation[3] = { 5, 5, 5 };
	float Diff_Finger[3] = { 5, 5, 5 };

	arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);

	while (Diff_Position[0] > 0.0005 ||
		Diff_Position[1] > 0.0005 ||
		Diff_Position[2] > 0.0005 ||
		Diff_Rotation[0] > 0.0005 ||
		Diff_Rotation[1] > 0.0005 ||
		Diff_Rotation[2] > 0.0005 ||
		Diff_Finger[0] > 0.0005 ||
		Diff_Finger[1] > 0.0005 ||
		Diff_Finger[2] > 0.0005
		)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		PosNow = arm->get_cart_pos();
		Diff_Position[0] = sqrt(pow(Position[0] - PosNow.position[0], 2));
		Diff_Position[1] = sqrt(pow(Position[1] - PosNow.position[1], 2));
		Diff_Position[2] = sqrt(pow(Position[2] - PosNow.position[2], 2));
		Diff_Rotation[0] = sqrt(pow(Rotation[0] - PosNow.rotation[0], 2));
		Diff_Rotation[1] = sqrt(pow(Rotation[1] - PosNow.rotation[1], 2));
		Diff_Rotation[2] = sqrt(pow(Rotation[2] - PosNow.rotation[2], 2));
		Diff_Finger[0] = sqrt(pow(Finger[0] - PosNow.finger_position[0], 2));
		Diff_Finger[1] = sqrt(pow(Finger[1] - PosNow.finger_position[1], 2));
		Diff_Finger[2] = sqrt(pow(Finger[2] - PosNow.finger_position[2], 2));

		arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);

		//std::cin.ignore();
	}
	Pos1 = arm->get_cart_pos();
	arm->stop_api_ctrl();
	
	return Pos1;
}

int main(int argc, char* argv[])
{
	KinDrv::init_usb();
	KinDrv::JacoArm* arm;

	try{
		arm = new KinDrv::JacoArm();		
	}
	catch (KinDrv::KinDrvException e){
		std::cout << e.what() << std::endl;

		std::cin.ignore();
		return 0;
	}
	std::cout << "Arm found!" << std::endl;
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();
	
	printf("position: %f, %f, %f \n", Pos1.position[0], Pos1.position[1], Pos1.position[2]);
	printf("rotation: %f, %f, %f \n", Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2]);
	printf("Finger:   %f, %f, %f \n", Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

	std::cin.ignore();
	float Position[3] = { 0, -0.50, 0.50 };
	float Rotation[3] = { 3, 0, -3 };
	float Finger[3] = { 10, 10, 10 };
	
	arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);
	
//	Pos1 = ArmMoveTo(arm, Position, Rotation, Finger);
	Pos1 = arm->get_cart_pos();
	printf("position: %f, %f, %f \n", Pos1.position[0], Pos1.position[1], Pos1.position[2]);
	printf("rotation: %f, %f, %f \n", Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2]);
	printf("Finger:   %f, %f, %f \n", Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

	std::cin.ignore(); 
	return 0;
}
