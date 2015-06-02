#pragma once

#include <kinjo/grasp/PickStrategy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <kinjo/arm/Arm.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is the BoxDropStrategy, you click a box and the arm is supposed to drop the item into the box
	**/
	class SimpleTablePickStrategy final : public PickStrategy
	{
	public:
		SimpleTablePickStrategy(kinjo::arm::Arm* arm);
		~SimpleTablePickStrategy() = default;

		void pickItem(cv::Vec3f vector) override;

	public:
		kinjo::arm::Arm* arm;

	};
}
}