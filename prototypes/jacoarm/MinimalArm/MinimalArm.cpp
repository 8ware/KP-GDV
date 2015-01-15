// MinimalArm.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
//
#include <iostream>           // std::cout
#include <memory>             // std::unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm
#include <thread>

const float pi = 3.1415f;
float handFromAbove[3] = { pi, 0, 0 };

void testImprecisions(KinDrv::JacoArm* arm);

KinDrv::jaco_position_t ArmMoveTo(KinDrv::JacoArm* arm, float Position[3], float Rotation[3], float Finger[3]){
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();
	Position[0] = Position[0];
	Position[1] = Position[1];
	Position[2] = Position[2];
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
	KinDrv::jaco_position_t posPrev = PosNow;

	float Diff_Position[3] = { 5, 5, 5 };
	float Diff_Rotation[3] = { 5, 5, 5 };
	float Diff_Finger[3] = { 5, 5, 5 };

	arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);
	bool hasmoved = true;
	while (hasmoved)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		PosNow = arm->get_cart_pos();
		hasmoved = false;
		for (int i = 0; i < 3; i++){
			if ((int)(PosNow.position[i] * 10000) != (int)(posPrev.position[i] * 10000) ||
				(int)(PosNow.rotation[i] * 100) != (int)(posPrev.rotation[i] * 100) ||
				(int)(PosNow.finger_position[i] * 100) != (int)(posPrev.finger_position[i] * 100)
				)
			{
				hasmoved = true;
				break;
			}
		}

		arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);
		posPrev = PosNow;
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
	KinDrv::jaco_position_t posPrev = PosNow;
	float Diff_Position[3] = { 5, 5, 5 };

	arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);
	bool hasmoved = true;

	while (hasmoved)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		PosNow = arm->get_cart_pos();
		hasmoved = false;
		for (int i = 0; i < 3; i++){
			if ((int)(PosNow.position[i] * 10000) != (int)(posPrev.position[i] * 10000))
				hasmoved = true;
		}
		arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

		posPrev = PosNow;
		//std::cin.ignore();
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	Pos1 = arm->get_cart_pos();
	ArmMoveTo(arm, Position, handFromAbove, Pos1.finger_position);
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

void getDiff(float (&ref)[3], float (&actual)[3], float(&result)[3]){
	actual[2] += 0.02f;
	for (int i = 0; i < 3; i++){
		result[i] = abs(ref[i] - actual[i]);
	}
}

///Muss ein mit null initialisiertes 
void calcMedian(float diffs[][3], int total, float (&result)[3]){
	for (int k = 0; k < total; k++){
		for (int x = 0; x < 3; x++){
			result[x] += diffs[k][x];
		}
	}
	for (int i = 0; i < 3; i++){
		result[i] /= (float)total;
	}
}

void testImprecisions(KinDrv::JacoArm* arm){
	//Test: Abweichungen in jede Achsenrichtung (hin und zurück)
	float deltaStart = 0.05f;//meter!
	float delta = 0.05f;

	//fahre zu ausgangsposition plus hand-rotation on oben
	float start_pos_position[3] = { 0.0f, -0.7f, 0.0f };
	KinDrv::jaco_position_t start_pos= arm->get_cart_pos();
	ArmMoveTo(arm, start_pos_position);

	const int countPoints = 6;
	float achsenFahrt[3][countPoints][3];
	float diffs[3][countPoints][3];//[Achse][i][x,y,z]
	printf("Achsen-Abweichungen: ");
	for (int achse = 0; achse < 3; achse++){
		for (int i = 0; i < countPoints; i++){
			for (int x = 0; x < 3; x++){
				float factor = ((i%2 ? 0.0f : 1.0f)) * (achse==1 ? 1 : 1);
				achsenFahrt[achse][i][x] = start_pos_position[x] + (deltaStart + delta*i) * factor * (achse == x ? 1 : 0);
			}
		}
	}

	for (int achse = 0; achse < 3; achse++){
		for (int i = 0; i < countPoints; i++){
			KinDrv::jaco_position_t result= ArmMoveTo(arm, achsenFahrt[achse][i]);
			getDiff(achsenFahrt[achse][i], arm->get_cart_pos().position, diffs[achse][i]);
			printf("Absolut: x=> %f, y=> %f, z=> %f\n", result.position[0], result.position[1], result.position[2]);
			printf("Abweichung: x=> %f, y=> %f, z=> %f\n", diffs[achse][i][0], diffs[achse][i][1], diffs[achse][i][2]);
		}
		//zurück zum start
		ArmMoveTo(arm, start_pos_position, handFromAbove, start_pos.finger_position);
		float medians[3] = { 0, 0, 0 };
		calcMedian(diffs[achse],countPoints,medians);
		printf("Mittlere Abweichung: x=> %f, y=> %f, z=> %f\n", medians[0], medians[1], medians[2]);
	}

	/*
	//Nun Figur abfahren und Gesamt-abweichung bestimmen
	const int pointCount = 4;

	//Muss evtl noch angepasst werden, da ich die Range des Arms nicht genau weiß
	float figur[pointCount][3] = {
		0.0f,0.3f,0.05f, // startpunkt
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

	printf("Mittlere Abweichung bei Figur: x=> %f, y=> %f, z=> %f", diffTotal[0], diffTotal[1], diffTotal[2]);*/
}
