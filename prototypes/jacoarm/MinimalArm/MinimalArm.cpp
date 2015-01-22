// MinimalArm.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//
#include <iostream>           // std::cout
#include <memory>             // std::unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm
#include <thread>			  // Sleep

//evaluation.cpp
void testImprecisions(KinDrv::JacoArm* arm);

KinDrv::JacoArm* initArm(){
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
	
	printf("Current position is:");
	printf("position: %f, %f, %f \n", Pos1.position[0], Pos1.position[1], Pos1.position[2]);
	printf("rotation: %f, %f, %f \n", Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2]);
	printf("Finger:   %f, %f, %f \n", Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);
	return arm;
}

int main(int argc, char* argv[])
{
	KinDrv::JacoArm* arm = initArm();

	//Moves arm around in every axis and evaluates deviations
	testImprecisions(arm);
	
	std::cin.ignore(); 
	return 0;
}

