// MinimalArm.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//
#include <iostream>           // std::cout
#include <memory>             // std::unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm

int main(int argc, char* argv[])
{
	KinDrv::init_usb();
	std::unique_ptr<KinDrv::JacoArm> arm;
/*	try{
		arm = std::unique_ptr<KinDrv::JacoArm>(new KinDrv::JacoArm());		
	}
	catch (KinDrv::KinDrvException e){
		std::cout << e.what() << std::endl;

		std::cin.ignore();
		return 0;
	}
	std::cout << "Arm found!" << std::endl;
	
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();

	printf("position: %f , %f, %f", Pos1.position[0], Pos1.position[1], Pos1.position[2]);
	std::cin.ignore();

	arm->start_api_ctrl();
	arm->set_target_cart(Pos1.position, Pos1.finger_position);

	std::cin.ignore(); */
	return 0;
}
