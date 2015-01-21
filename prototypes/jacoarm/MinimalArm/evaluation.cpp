#include <iostream>           // std::cout
#include <memory>             // std::unique_ptr
#include "libkindrv/kindrv.h" // KinDrv::JacoArm
#include <thread>

const float pi = 3.1415f;
float handFromAbove[3] = { pi, 0, 0 };


void getDiff(float(&ref)[3], float(&actual)[3], float(&result)[3]){
	actual[2] += 0.02f;
	for (int i = 0; i < 3; i++){
		result[i] = abs(ref[i] - actual[i]);
	}
}

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
	}
	Pos1 = arm->get_cart_pos();
	arm->stop_api_ctrl();

	return Pos1;
}


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
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		PosNow = arm->get_cart_pos();
		hasmoved = false;
		for (int i = 0; i < 3; i++){
			if ((int)(PosNow.position[i] * 10000) != (int)(posPrev.position[i] * 10000))
				hasmoved = true;
		}
		arm->set_target_cart(Position[0], Position[1], Position[2], Pos1.rotation[0], Pos1.rotation[1], Pos1.rotation[2], Pos1.finger_position[0], Pos1.finger_position[1], Pos1.finger_position[2]);

		posPrev = PosNow;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	
	//Move Hand to "From above", position remains the same or gets a little corrected with this movement
	Pos1 = arm->get_cart_pos();
	ArmMoveToEval(arm, Position, handFromAbove, Pos1.finger_position);

	Pos1 = arm->get_cart_pos();
	arm->stop_api_ctrl();

	return Pos1;
}



//Test: Deviation in every axis, moves arm back and forth and sums up the deviations to calculate a average deviation
void testImprecisions(KinDrv::JacoArm* arm){
	float deltaStart = 0.05f;//meter!
	float delta = 0.05f;

	//Move to start position, with hand-rotation "from above"
	float start_pos_position[3] = { 0.0f, -0.7f, 0.0f };
	KinDrv::jaco_position_t start_pos = arm->get_cart_pos();
	ArmMoveToEval(arm, start_pos_position);

	const int countPoints = 6;
	float evalPoints[3][countPoints][3];
	float diffs[3][countPoints][3];//[axis][i][x,y,z]
	
	printf("Axis-Deviations: ");
	
	//Prepare points to evaluate
	for (int axis = 0; axis < 3; axis++){
		for (int i = 0; i < countPoints; i++){
			for (int x = 0; x < 3; x++){
				float factor = ((i % 2 ? 0.0f : 1.0f));//back and forth, y-axis: opposite direction needed
				evalPoints[axis][i][x] = start_pos_position[x] + (deltaStart + delta*i) * factor * (axis == x ? 1 : 0);
			}
		}
	}

	for (int axis = 0; axis < 3; axis++){
		for (int i = 0; i < countPoints; i++){
			KinDrv::jaco_position_t result = ArmMoveToEval(arm, evalPoints[axis][i]);
			getDiff(evalPoints[axis][i], arm->get_cart_pos().position, diffs[axis][i]);
			printf("Position: x=> %f, y=> %f, z=> %f\n", result.position[0], result.position[1], result.position[2]);
			printf("Deviation: x=> %f, y=> %f, z=> %f\n", diffs[axis][i][0], diffs[axis][i][1], diffs[axis][i][2]);
		}
		//zurück zum start
		ArmMoveToEval(arm, start_pos_position, handFromAbove, start_pos.finger_position);
		float avg[3] = { 0, 0, 0 };
		calcMedian(diffs[axis], countPoints, avg);
		printf("Average Deviation: x=> %f, y=> %f, z=> %f\n", avg[0], avg[1], avg[2]);
	}
}
