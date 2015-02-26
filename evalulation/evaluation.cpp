/**
* This file provides the method testImprecisions() to 
* do a test run in every axis and print the results to
* csv file with the deviations of a test run.
*/
#include <iostream>           // cout
#include <fstream>
#include <string>
#include <memory>             // unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm
#include <thread>
#include <cmath>

using namespace std;

const float pi = 3.1415f;
float handFromAbove[3] = { pi, 0, 0 };

//Calculate the absolute difference for all coordinates
void getDiff(float(&ref)[3], float(&actual)[3], float(&result)[3]){
	actual[2] += 0.02f;
	for (int i = 0; i < 3; i++){
		result[i] = abs(ref[i] - actual[i]);
	}
}

//Calculates the average of all coordinates
void calcMedian(float diffs[][3], int total, float(&result)[3]){
	result[0] = 0; result[1] = 0; result[2] = 0;
	for (int k = 0; k < total; k++){
		for (int x = 0; x < 3; x++){
			result[x] += diffs[k][x];
		}
	}
	for (int i = 0; i < 3; i++){
		result[i] /= (float)total;
	}
}

//Moves the arm to the specified position, rotation and finger position
KinDrv::jaco_position_t ArmMoveToEval(KinDrv::JacoArm* arm, float Position[3], float Rotation[3], float Finger[3]){
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();

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

	arm->set_target_cart(Position[0], Position[1], Position[2], Rotation[0], Rotation[1], Rotation[2], Finger[0], Finger[1], Finger[2]);
	bool hasmoved = true;
	while (hasmoved)
	{
		this_thread::sleep_for(chrono::milliseconds(100));
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
	}
	Pos1 = arm->get_cart_pos();
	arm->stop_api_ctrl();

	return Pos1;
}


//moves the arm to the position, and fires a second
//movement instruction to reset the hand rotation
KinDrv::jaco_position_t ArmMoveToEval(KinDrv::JacoArm* arm, float Position[3]){
	KinDrv::jaco_position_t Pos1 = arm->get_cart_pos();

	Pos1.position[0] = Position[0];
	Pos1.position[1] = Position[1];
	Pos1.position[2] = Position[2];

	KinDrv::jaco_position_t PosNow = arm->get_cart_pos();
	KinDrv::jaco_position_t posPrev = PosNow;

	arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);
	bool hasmoved = true;

	while (hasmoved)
	{
		this_thread::sleep_for(chrono::milliseconds(100));
		PosNow = arm->get_cart_pos();
		hasmoved = false;
		for (int i = 0; i < 3; i++){
			if ((int)(PosNow.position[i] * 10000) != (int)(posPrev.position[i] * 10000))
				hasmoved = true;
		}
		arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

		posPrev = PosNow;
	}
	this_thread::sleep_for(chrono::milliseconds(100));
	
	//Move Hand to "From above", position remains the same or gets a little corrected with this movement
	Pos1 = arm->get_cart_pos();
	ArmMoveToEval(arm, Position, handFromAbove, Pos1.finger_position);

	Pos1 = arm->get_cart_pos();
	arm->stop_api_ctrl();

	return Pos1;
}



///Test: Deviation in every axis, moves arm back and forth and sums up the deviations to calculate a average deviation
///outputs the results in the specified filename as a csv
void testImprecisions(KinDrv::JacoArm* arm, std::string csvFileName){
	
	float deltaStart = 0.05f;//meter!
	float delta = 0.05f;

	//Move to start position, with hand-rotation "from above"
	float start_pos_position[3] = { 0.0f, -0.6f, 0.1f };
	KinDrv::jaco_position_t start_pos = arm->get_cart_pos();
	ArmMoveToEval(arm, start_pos_position);

	const int countPoints = 6;
	float evalPoints[3][countPoints][3];
	float evalPointsResult[3][countPoints][3];
	float evalPointsStarts[3][countPoints][3];
	float diffs[3][countPoints][3];//[axis][i][x,y,z]
	float avgs[3][3];
	
	//Prepare points to evaluate
	for (int axis = 0; axis < 3; axis++){
		for (int i = 0; i < countPoints; i++){
			for (int x = 0; x < 3; x++){
				float factor = ((i % 2 ? 0.0f : 1.0f));//back and forth, y-axis: opposite direction needed
				evalPoints[axis][i][x] = start_pos_position[x] + (deltaStart + delta*i) * factor * (axis == x ? 1 : 0);
			}
		}
	}
	//Move the arm to all the positions and save the results
	for (int axis = 0; axis < 3; axis++){
		for (int i = 0; i < countPoints; i++){
			
			KinDrv::jaco_position_t start = arm->get_cart_pos();

			evalPointsStarts[axis][i][0] = start.position[0];
			evalPointsStarts[axis][i][1] = start.position[1];
			evalPointsStarts[axis][i][2] = start.position[2];

			KinDrv::jaco_position_t result =	ArmMoveToEval(arm, evalPoints[axis][i]);
			getDiff(evalPoints[axis][i], result.position, diffs[axis][i]);

			evalPointsResult[axis][i][0] = result.position[0];
			evalPointsResult[axis][i][1] = result.position[1];
			evalPointsResult[axis][i][2] = result.position[2];

			printf("Position: x=> %f, y=> %f, z=> %f\n", result.position[0], result.position[1], result.position[2]);
			printf("Deviation: x=> %f, y=> %f, z=> %f\n", diffs[axis][i][0], diffs[axis][i][1], diffs[axis][i][2]);
		}
		//back to start
		ArmMoveToEval(arm, start_pos_position, handFromAbove, start_pos.finger_position);
		
		calcMedian(diffs[axis], countPoints, avgs[axis]);
		printf("Average Deviation: x=> %f, y=> %f, z=> %f\n", avgs[axis][0], avgs[axis][1], avgs[axis][2]);
		printf("\n");
	}

	//Print into csv file:
	//startposition, delta, destination position, actually reached position, deviation
	ofstream csvFile;
	string delimiter = ",";
	csvFile.open(csvFileName, ios::out);
	if (csvFile.is_open()){
		for (int a = 0; a < 3; a++){
			for (int i = 0; i < countPoints; i++){
				//start
				for (int c = 0; c < 3; c++){
					csvFile << evalPointsStarts[a][i][c] << delimiter;
				}
				//destination
				for (int c = 0; c < 3; c++){
					csvFile << evalPoints[a][i][c] << delimiter;
				}
				//delta Movement
				for (int c = 0; c < 3; c++){
					csvFile << abs(evalPointsStarts[a][i][c] - evalPoints[a][i][c]) << delimiter;
				}
				//reached position
				for (int c = 0; c < 3; c++){
					csvFile << evalPointsResult[a][i][c] << delimiter;
				}
				//diff to destination
				for (int c = 0; c < 3; c++){
					csvFile << diffs[a][i][c] << delimiter;
				}
				csvFile << endl;
				
			}
		}

		//Print averages per axis movements
		csvFile << endl;
		csvFile << "Averages";
		csvFile << endl;
		for (int a = 0; a < 3; a++){
			for (int i = 0; i < 3; i++){
				csvFile << avgs[a][i] << delimiter;
			}
		}
		csvFile.close();
	}

}
