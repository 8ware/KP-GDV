#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is a DroppStrategy Interface
	**/
	class DropStrategy
	{
	public:
		virtual void dropItem(cv::Matx44f rigidBodyMatrix, cv::Vec3f visionVector) = 0;
	};
	}
}