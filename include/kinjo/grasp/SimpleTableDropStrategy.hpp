#pragma once

#include <kinjo/grasp/DropStrategy.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <kinjo/arm/Arm.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is the SimpleTableDropStrategy, you click on a table and
	* the arm is supposed to drop the item on top of it
	**/
	class SimpleTableDropStrategy final : public DropStrategy
	{
	public:
		SimpleTableDropStrategy(kinjo::arm::Arm* arm);
		~SimpleTableDropStrategy() = default;

		void dropItem(cv::Matx44f rigidBodymatrix, cv::Vec3f visionVector) override;

	public:
		kinjo::arm::Arm* arm;

	};
}
}