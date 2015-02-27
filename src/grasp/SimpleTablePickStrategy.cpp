#include <kinjo/grasp/SimpleTablePickStrategy.hpp>


namespace kinjo {
namespace grasp {

	SimpleTablePickStrategy::SimpleTablePickStrategy(kinjo::arm::Arm* arm){
		this->arm = arm;
	}

	void SimpleTablePickStrategy::pickItem(cv::Vec3f vector){

		this->arm->moveToStartPosition(false);
		this->arm->openFingers();

		cv::Vec3f const zInvariant(vector[0], vector[1], vector[2] + 200);
		this->arm->moveTo(zInvariant);
		this->arm->moveTo(vector);
		this->arm->closeFingers();
		
		// do we really want to detour to the start position?
		// Not going to the drop position immediately
		this->arm->moveToStartPosition(true);

		// LOG(INFO) << "Grab Item done!";

	}
}
}