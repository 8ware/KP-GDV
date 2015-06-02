#pragma once

#include <kinjo/grasp/Grasper.hpp>
#include <kinjo/grasp/PickStrategy.hpp>
#include <kinjo/grasp/DropStrategy.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is the Simple Pick And Drop Grasper, inherited from Grasper
	**/
	class SimplePickAndDropGrasper final : public Grasper
	{
	public:
		SimplePickAndDropGrasper(
			kinjo::grasp::PickStrategy* picker,
			kinjo::grasp::DropStrategy* dropper);
		~SimplePickAndDropGrasper() = default;

		void pickItem(cv::Vec3f vector) override;
		void dropItem(cv::Matx44f rigidBodymatrix, cv::Vec3f visionVector) override;
	

	public:
		kinjo::grasp::PickStrategy* picker;
		kinjo::grasp::DropStrategy* dropper;
	};
}
}