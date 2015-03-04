#pragma once

#include <opencv2/core/affine.hpp>

namespace kinjo {
namespace arm {

	class MovementGuard {
	public:
		virtual void Handle_Deathzones(cv::Vec3f startPos, cv::Vec3f endPos, int *HandlingResult, cv::Vec3f *PosToTravelFirst) = 0;
	};

}//namespace arm
}//namespace kinjo