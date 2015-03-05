#include <kinjo/app/GraspState.hpp>

#include <iostream>
#include <thread>

#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f
#include <opencv2/highgui/highgui.hpp>	// CV_EVENT_{L,R}BUTTONDBLCLK


namespace kinjo {
namespace app {

GraspState::GraspState(State** readyState,
		grasp::Grasper* grasper, vision::Vision* vision, calibration::Calibrator* calibrator,
		int* xOffset, int* yOffset, int* zOffset) {
	this->log = el::Loggers::getLogger("GraspState");

	this->mutex = new std::mutex();
	this->mutex->lock();

	this->designator = "GRASPING";
	this->infos.push_back("Press 'a' to abort grasping!");

	this->next = this;
	this->readyState = readyState;

	this->grasper = grasper;
	this->vision = vision;
	this->calibrator = calibrator;

	this->xOffset = xOffset;
	this->yOffset = yOffset;
	this->zOffset = zOffset;
}

void GraspState::process(int mouseEvent, cv::Point point) {
	if (mouseEvent == CV_EVENT_LBUTTONDBLCLK){

		cv::Matx44f const mat44fRigidBodyTransformation
			= calibrator->getRigidBodyTransformation();

		log->debug("Clicked image pixel position: %v", point);

		cv::Vec3f const v3fVisionPosition
			= vision->estimateVisionPositionFromImagePointPx(point);

		if (v3fVisionPosition[2] > 0.0f) {
			log->debug("Resulting vision position: %v", v3fVisionPosition);

			cv::Vec3f v3fArmPosition(mat44fRigidBodyTransformation * v3fVisionPosition);
			log->debug("Resulting arm position: %v", v3fArmPosition);

			v3fArmPosition[0] += static_cast<float>(*xOffset);
			v3fArmPosition[1] += static_cast<float>(*yOffset);
			v3fArmPosition[2] += static_cast<float>(*zOffset);
			log->debug("Resulting arm position with offset: %v", v3fArmPosition);

			pickqueue.push(v3fArmPosition);
			log->info("Enqueued target %v", v3fArmPosition);
			graspAsync();
		}
		else {
			log->warn("Clicked position with unknown depth: can not move there!");
			if (!this->activeThread)
				this->next = *this->readyState;
		}
	}
	else if (mouseEvent == CV_EVENT_RBUTTONDBLCLK){

		cv::Matx44f const mat44fRigidBodyTransformation
			= calibrator->getRigidBodyTransformation();

		log->debug("Clicked image pixel position: %v", point);

		cv::Vec3f const v3fVisionPosition
			= vision->estimateVisionPositionFromImagePointPx(point);

		if (v3fVisionPosition[2] > 0.0f) {
			log->debug("Resulting vision position: %v", v3fVisionPosition);
			/*
			cv::Vec3f v3fArmPosition(mat44fRigidBodyTransformation * v3fVisionPosition);
			log->debug("Resulting arm position: %v", v3fArmPosition);
			
			v3fArmPosition[0] += static_cast<float>(*xOffset);
			v3fArmPosition[1] += static_cast<float>(*yOffset);
			v3fArmPosition[2] += static_cast<float>(*zOffset);
			log->debug("Resulting arm position with offset: %v", v3fArmPosition);
			*/
			if (!this->dropposition[0])
				this->mutex->unlock();
			this->dropposition =v3fVisionPosition;
			log->info("Set target drop visionposition %v", v3fVisionPosition);
			graspAsync();
		}
		else {
			log->warn("Clicked position with unknown depth: can not move there!");
			if (!this->activeThread)
				this->next = *this->readyState;
		}
	}
}

void GraspState::graspAsync() {
	if (this->activeThread) {
		log->debug("Grasping thread already running...");
		return;
	}

	this->activeThread = true;

	log->debug("Starting grapsing thread...");
	std::thread thread(&GraspState::grasp, this);
	thread.detach();
}

void GraspState::grasp() {
	while (!this->pickqueue.empty()) {
		this->currentTargetPosition = pickqueue.front();
		pickqueue.pop();
		log->info("Dequeued target position %v", this->currentTargetPosition);

		std::stringstream stream;
		stream << "Target at position " << this->currentTargetPosition << " mm";
		this->details = stream.str();

		this->grasper->pickItem(currentTargetPosition);
		this->mutex->lock();
		this->grasper->dropItem(calibrator->getRigidBodyTransformation(), this->dropposition);
		this->mutex->unlock();
		//arm->GrabItem(currentTargetPosition);
	}

	this->activeThread = false;
	this->next = *this->readyState;
}

void GraspState::process(int key) {
	switch (key) {
		case 'a':
			log->trace("Key 'a' was pressed");
			// TODO abort grasping
			this->next = *this->readyState;
			break;
	}
}

} // end namespace app
} // end namespace kinjo

