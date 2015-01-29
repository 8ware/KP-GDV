#pragma once

#include <opencv2/core/affine.hpp>

namespace kinjo {
	namespace arm {

		class MovementGuard {
		public:
			virtual void Init_Deadzones() = 0;
			virtual void Handle_Deathzones(cv::Vec3f startPos, cv::Vec3f endPos, int *HandlingResult, cv::Vec3f *PosToTravelFirst) = 0;
			virtual bool LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos) = 0;
			virtual cv::Vec3f CalculateDetour(cv::Vec3f startPos, cv::Vec3f endPos) = 0;
			virtual bool StartpointLegal(cv::Vec3f startPos, bool *InInnerCircle) = 0;
			virtual bool EndpointLegal(cv::Vec3f endPos, bool *InInnerCircle) = 0;
			virtual bool EndpointNotInTable(cv::Vec3f endPos) = 0;
		};

	}//arm
}//kinjo