#pragma once

#include <kinjo/grasp/DropStrategy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <kinjo/arm/Arm.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is the BoxDropStrategy, you click a box and the arm is supposed to drop the item into the box
	**/
	class BoxDropStrategy final : public DropStrategy
	{
	public:
		BoxDropStrategy(kinjo::arm::Arm* arm);
		~BoxDropStrategy() = default;

		void dropItem(cv::Matx44f rigidBodymatrix, cv::Vec3f visionVector) override;

	public:
		kinjo::arm::Arm* arm;
	};
}
}