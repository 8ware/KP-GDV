// MinimalArm.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//
#include <iostream>           // std::cout
#include <memory>             // std::unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm
#include <thread>

void testImprecisions(KinDrv::JacoArm* arm);

KinDrv::jaco_position_t ArmMoveTo(KinDrv::JacoArm* arm, float Position[3], float Rotation[3], float Finger[3]){
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();
	Position[0] = Position[0] * 0.01f;
	Position[1] = Position[1] * 0.01f;
	Position[2] = Position[2] * 0.01f;
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


KinDrv::jaco_position_t ArmMoveTo(KinDrv::JacoArm* arm, float Position[3]){
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();

	Position[0] = Position[0];// * 0.01f;
	Position[1] = Position[1];// * 0.01f;
	Position[2] = Position[2];// *0.01f;
	Pos1.position[0] = Position[0];
	Pos1.position[1] = Position[1];
	Pos1.position[2] = Position[2];

	KinDrv::jaco_position_t PosNow = arm->get_cart_pos();
	float Diff_Position[3] = { 5, 5, 5 };

	arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

	while (Diff_Position[0] > 0.01 ||
		Diff_Position[1] > 0.01 ||
		Diff_Position[2] > 0.01
		)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		PosNow = arm->get_cart_pos();
		Diff_Position[0] = sqrt(pow(Position[0] - PosNow.position[0], 2));
		Diff_Position[1] = sqrt(pow(Position[1] - PosNow.position[1], 2));
		Diff_Position[2] = sqrt(pow(Position[2] - PosNow.position[2], 2));

		arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

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

	testImprecisions(arm);
	
	std::cin.ignore();
	/*float Position[3] = { 0, -0.50, 0.50 };
	float Rotation[3] = { 3, 0, -3 };
	float Finger[3] = { 10, 10, 10 };*/
	
	//arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);
	
//	Pos1 = ArmMoveTo(arm, Position, Rotation, Finger);
	/*Pos1 = arm->get_cart_pos();
	printf("position: %f, %f, %f \n", Pos1.position[0], Pos1.position[1], Pos1.position[2]);
	printf("rotation: %f, %f, %f \n", Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2]);
	printf("Finger:   %f, %f, %f \n", Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);
	*/
	std::cin.ignore(); 
	return 0;
}



void Moveandprint(KinDrv::JacoArm* arm, float m_x, float m_y, float m_z){
	float p[3] = { m_x, m_y, m_z };
	KinDrv::jaco_position_t nachher = ArmMoveTo(arm, p);
	printf("nachher: x:%f y:%f z:%f \nSoll: x:%f y:%f z:%f", nachher.position[0], nachher.position[1], nachher.position[2], m_x, m_y, m_z);
}

void getDiff(KinDrv::jaco_position_t ref, KinDrv::jaco_position_t actual, float(&result)[3]){
	for (int i = 0; i < 3; i++){
		result[i] = abs(ref.position[i] - actual.position[i]);
	}
}

void testImprecisions(KinDrv::JacoArm* arm){
	//Test: Abweichungen in jede Achsenrichtung (hin und zurück)
	float diffs[3][2][3];//[Achse][Hin/Rück][x,y,z]
	float delta = 0.15f;//meter!

	printf("Achsen-Abweichungen: ");
	for (int achse = 0; achse < 3; achse++){
		//hin
		KinDrv::jaco_position_t p = arm->get_cart_pos();
		p.position[achse] += delta;
		Moveandprint(arm, p.position[0], p.position[1], p.position[2]);
		getDiff(p, arm->get_cart_pos(), diffs[achse][0]);
		printf("hin Abweichung: x=> %f, y=> %f, z=> %f", achse, diffs[achse][0], diffs[achse][1], diffs[achse][2]);
		//und zurück
		p = arm->get_cart_pos();
		p.position[achse] -= delta;
		Moveandprint(arm, p.position[0], p.position[1], p.position[2]);
		getDiff(p, arm->get_cart_pos(), diffs[achse][1]);
		printf("zurück Abweichung: x=> %f, y=> %f, z=> %f", achse, diffs[achse][0], diffs[achse][1], diffs[achse][2]);
	}
	float median[3] = { 0, 0, 0 };
	int total = 6;
	for (int achse = 0; achse < 3; achse++){
		for (int hz = 0; hz < 2; hz++){
			for (int i = 0; i < 3; i++){
				median[i] += diffs[achse][hz][i];
			}
		}
	}
	for (int i = 0; i < 3; i++){
		median[i] /= total;
	}

	printf("Mittlere Abweichung bei Achsen-Test:  x=> %f, y=> %f, z=> %f", median[0], median[1], median[2]);

	//Nun Figur abfahren und Gesamt-abweichung bestimmen
	const int pointCount = 4;

	//Muss evtl noch angepasst werden, da ich die Range des Arms nicht genau weiß
	float figur[pointCount][3] = {
		0.0f,0.3f,0.3f, // startpunkt
		0.2f, 0.4f, 0.4f,
		-0.2f, 0.5f, 0.5f,
		0.3f, 0.3f, 0.3f // endpunkt = startpunkt
	};

	float diffTotal[3] = { 0, 0, 0 };
	for (int i = 0; i < pointCount; i++){
		KinDrv::jaco_position_t p = arm->get_cart_pos();
		Moveandprint(arm, figur[i][0], figur[i][1], figur[i][2]);
		float diff[3];
		getDiff(p, arm->get_cart_pos(), diff);
		for (int k = 0; k < 3; k++) { diffTotal[k] += diff[k] / pointCount; }//gleich mittelwert bilden
		printf("Abweichung:  x=> %f, y=> %f, z=> %f",diff[0],diff[1],diff[2]);
	}

	printf("Mittlere Abweichung bei Figur: x=> %f, y=> %f, z=> %f", diffTotal[0], diffTotal[1], diffTotal[2]);
}
