#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
namespace grasp {

	/**
	* this is a PickStrategy Interface
	**/
	class PickStrategy
	{
	public:
		virtual void pickItem(cv::Vec3f vector) = 0;
	};
}
}