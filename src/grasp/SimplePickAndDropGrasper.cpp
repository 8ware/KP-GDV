#include <kinjo/grasp/SimplePickAndDropGrasper.hpp>


namespace kinjo {
namespace grasp {

	SimplePickAndDropGrasper::SimplePickAndDropGrasper(
		kinjo::grasp::PickStrategy* picker,
		kinjo::grasp::DropStrategy* dropper){
		this->picker = picker;
		this->dropper = dropper;
	}

	void SimplePickAndDropGrasper::pickItem(cv::Vec3f vector){
		picker->pickItem(vector);
	}

	void SimplePickAndDropGrasper::dropItem(cv::Matx44f rigidBodyMatrix, cv::Vec3f visionVector){
		dropper->dropItem(rigidBodyMatrix, visionVector);
	}
}
}