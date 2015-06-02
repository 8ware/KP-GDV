#include <kinjo/grasp/SimpleTableDropStrategy.hpp>

#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f

namespace kinjo {
namespace grasp {

	SimpleTableDropStrategy::SimpleTableDropStrategy(kinjo::arm::Arm* arm){
		this->arm = arm;
	}

	void SimpleTableDropStrategy::dropItem(cv::Matx44f rigidBodyMatrix, cv::Vec3f visionVector){
		this->arm->moveToStartPosition(true);
		
		cv::Vec3f vector(rigidBodyMatrix * visionVector);

		cv::Vec3f const zInvariant(vector[0], vector[1], vector[2] + 100);
		this->arm->moveTo(zInvariant);
		this->arm->openFingers();
		this->arm->moveToStartPosition(false);

		//LOG(INFO) << "Drop Item done!";
	}
}
}