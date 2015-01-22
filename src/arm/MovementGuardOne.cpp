#include <kinjo/arm/MovementGuardOne.hpp>
#include <opencv2/imgproc/imgproc.hpp>



namespace kinjo {
	namespace arm {
		
		
		bool MovementGuardOne::LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos, float CircleRadius, JacoArm& arm){
			//check if start or endpoint are inside the circle
			if (sqrt(startPos[0] * startPos[0] + startPos[1] * startPos[1] + startPos[2] * startPos[2]) < CircleRadius) {
				printf("StartPos inside Deadzone!\n");
				return true;
			}
			if (sqrt(endPos[0] * endPos[0] + endPos[1] * endPos[1] + endPos[2] * endPos[2]) < CircleRadius){
				printf("EndPos inside Deadzone! \n");
				return true;
			}


			//now that we know both points are outside the circle, lets begin
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
			if (distance < CircleRadius && Lambda>0.0f && Lambda < 1.0f){

				//now we know we intersect the deadzone
				//So we tell the system to move to another position first

				//we need a new z coordinate
				S = startPos;
				B = endPos - startPos;
				Lambda = (0 - S.dot(B)) / (B.dot(B));
				P[2] = S[2] + Lambda * B[2];
				//Normalize
				if (sqrt(P.dot(P)) == 0){
					//in this case P is exactly on 0,0,0, so we need a Plan B
					//very unlikely but possible...
					//we rotate direction Vector B 90 degree
					P[0] = B[1];
					P[1] = B[0];
				}

				P = P / sqrt(P.dot(P));
				printf("Calculating Detour\n");
				arm.moveTo(P * CircleRadius * 2.3f * 1000);
				arm.moveTo(endPos * 1000);
				return true;
			}
			return false;
		}
	}//arm
}//kinjo
