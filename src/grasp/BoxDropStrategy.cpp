#include <kinjo/grasp/BoxDropStrategy.hpp>
#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f


namespace kinjo {
namespace grasp {

	BoxDropStrategy::BoxDropStrategy(kinjo::arm::Arm* arm){
		this->arm = arm;
	}

	void BoxDropStrategy::dropItem(cv::Matx44f rigidBodyMatrix, cv::Vec3f visionVector){
		this->arm->moveToStartPosition(true);

		visionVector[2] += 140;
		cv::Vec3f vector(rigidBodyMatrix * visionVector);
		
		cv::Vec3f const zVariant(vector[0], vector[1], vector[2] + 150);
		cv::Vec3f const zVariant2(vector[0], vector[1], vector[2] + 50);
		this->arm->moveTo(zVariant);
		this->arm->moveTo(zVariant2);
		this->arm->openFingers();
		this->arm->moveTo(zVariant);
		this->arm->moveToStartPosition(false);
	}
}
}