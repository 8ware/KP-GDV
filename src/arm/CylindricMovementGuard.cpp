#include <kinjo/arm/CylindricMovementGuard.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <easylogging++.h>

namespace kinjo {
namespace arm {

	CylindricMovementGuard::CylindricMovementGuard()
	{
	}

	void CylindricMovementGuard::Init_Deadzones() {
		//TODO: Read Values from config file
		InnerCircleRadius = 0.2f;
		OuterCircleRadius = 0.7f;
		maxHeight = 0.8f;
		tableHeight = -0.1f;
	}

	void CylindricMovementGuard::Handle_Deathzones(cv::Vec3f startPos, cv::Vec3f endPos, int *HandlingResult, cv::Vec3f *PosToTravelFirst){
		Init_Deadzones();
		if (startPos == endPos){
			*HandlingResult = 0;
			return;
		}
		if (!EndpointLegal(endPos)){
			*HandlingResult = 2;
			return;
		}
			

		if (!EndpointNotInTable(endPos)){
			//4 arm was send inside the table(be more careful!)
			*HandlingResult = 2;
			return;
		}
			
		if (!StartpointLegal(startPos)){
			*HandlingResult = 3;
			return;
		}

		if (LineCircleIntersection(startPos, endPos)){

			*PosToTravelFirst = CalculateDetour(startPos, endPos);

			//1 simple detour(no problem)
			*HandlingResult = 1;
			return;
		}

		//0 everything went fine
		*HandlingResult = 0;
		return;
	}

	bool CylindricMovementGuard::LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos) {

		//g: x-> = S + Lambda * (E - S) = S + Lambda * B
		//define short variable names
		//calc Vector B
		cv::Vec3f S = startPos,
			B = endPos - startPos,
			P; //P is Point on g closest to 0 0 0 or 0 0 0 in the worst case
		float Lambda;

		//in case StartPoint and EndPoint are the same
		if (B[0] == 0.0f && B[1] == 0.0f)
			return false;

		//since we ignore the z level:
		S[2] = 0.0f;
		B[2] = 0.0f;
		//calc closest point on g to 0,0,0
		Lambda = (0 - S.dot(B)) / (B.dot(B));
		P = S + Lambda * B;
		float distance = sqrt(P.dot(P)); //Distance between 0 0 0 and the closest point
		//if thats lesser than the Radius AND between Startpoint and Enpoint, we have an intersection
		if (distance < InnerCircleRadius && Lambda>0.0f && Lambda < 1.0f){
			return true;
		}
		return false;
	}

	cv::Vec3f CylindricMovementGuard::CalculateDetour(cv::Vec3f startPos, cv::Vec3f endPos){

		cv::Vec3f S = startPos,
			B = endPos - startPos,
			P; //P is Point on g closest to 0 0 0 or 0 0 0 in the worst case


		S[2] = 0.0f;
		B[2] = 0.0f;
		//calc closest point on g to 0,0,0
		float Lambda = (0 - S.dot(B)) / (B.dot(B));

		P = S + (Lambda * B);
		//Normalize
		if (sqrt(P.dot(P)) == 0){
			//in this case P is exactly on 0,0,0, so we need a Plan B
			//very unlikely but possible...
			//we rotate direction Vector B 90 degree
			P[0] = B[1];
			P[1] = B[0];
		}

		//Z level of P still is zero, we interpolate the Z between Start and End Point
		P = P / sqrt(P.dot(P));
		P = P * InnerCircleRadius * 1.5f;
		S = startPos;
		B = endPos - startPos;
		P[2] = S[2] + Lambda * B[2];
		return P;
	}

	bool CylindricMovementGuard::StartpointLegal(cv::Vec3f startPos) {
		if (sqrt(startPos[0] * startPos[0] + startPos[1] * startPos[1] + startPos[2] * startPos[2]) < InnerCircleRadius) {
			LOG(INFO) << "startpoint too close to socket, use physical joystick"; 
			return false;
		}
		if (sqrt(startPos[0] * startPos[0] + startPos[1] * startPos[1] + startPos[2] * startPos[2]) > OuterCircleRadius) {
			LOG(INFO) << "startpoint too far away, something is wrong"; 
			return false;
		}
		if (maxHeight > startPos[2]){
			LOG(INFO) << "startpoint too high, something is wrong";
			//return false; //TODO: test this first!
		}
		return true;
	}

	bool CylindricMovementGuard::EndpointLegal(cv::Vec3f endPos) {
		if (sqrt(endPos[0] * endPos[0] + endPos[1] * endPos[1] + endPos[2] * endPos[2]) < InnerCircleRadius){
			LOG(INFO) << "endpoint too close to socket";
			return false;
		}
		if (sqrt(endPos[0] * endPos[0] + endPos[1] * endPos[1] + endPos[2] * endPos[2]) > OuterCircleRadius){
			LOG(INFO) << "endpoint not reachable, too far away";
			return false;
		}
		if (maxHeight > endPos[2]){
			LOG(INFO) << "endpoint too high";
			//return false; // TODO: test this first!
		}
		return true;
	}
	bool CylindricMovementGuard::EndpointNotInTable(cv::Vec3f endPos){
		if (endPos[2] <= tableHeight){
			LOG(INFO) << "endpoint in table, Z = " << endPos[2];
			return false;
		}
		return true;
	}
}//arm
}//kinjo
