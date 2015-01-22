#include <kinjo/arm/MovementGuard.hpp>


namespace kinjo {
	namespace arm {

		class MovementGuardOne final : public MovementGuard {

		public:
			MovementGuardOne();
			virtual ~MovementGuardOne() = default;
			bool LineCircleIntersection(cv::Vec3f startPos, cv::Vec3f endPos, float CircleRadius, JacoArm& arm) override;
				

		};

	}//arm
}//kinjo