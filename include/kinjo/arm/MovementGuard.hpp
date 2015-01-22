#include <opencv2/core/affine.hpp>
#include <kinjo/arm/JacoArm.hpp>

namespace kinjo {
	namespace arm {

		class MovementGuard {

			virtual bool LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos, float CircleRadius, JacoArm& arm) = 0;
		};

	}//arm
}//kinjo