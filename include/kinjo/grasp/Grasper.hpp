#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is a Grasper Interface
	**/
	class Grasper
	{
	public:
		virtual void pickItem(cv::Vec3f vector) = 0;
		virtual void dropItem(cv::Matx44f rigidBodymatrix, cv::Vec3f visionVector) = 0;
	};
}
}