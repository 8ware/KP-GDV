#include <kinjo/app/CalibrationState.hpp>

#include <iostream>
#include <thread>

#include <easylogging++.h>

#include <kinjo/RenderHelper.hpp>


static el::Logger* LOG = el::Loggers::getLogger("CalibState");

namespace kinjo {
namespace app {

CalibrationState::CalibrationState(State** initState, State** readyState,
		arm::Arm* arm, calibration::Calibrator* calibrator) {
	this->designator = "CALIBRATING";
	this->infos.push_back("Press 'a' to abort calibration!");

	this->next = this;
	this->initState = initState;
	this->readyState = readyState;

	this->arm = arm;
	this->calibrator = calibrator;
}

void CalibrationState::initialize() {
	calibrator->reset();
	std::thread(&CalibrationState::initializeAsync, this).detach();
}

void CalibrationState::initializeAsync() {
	arm->moveToStartPosition(true);
	calibrator->calibrateAsync();
}

void CalibrationState::process(cv::Mat& rgbImage, cv::Mat& depthImage) {
	if(calibrator->getIsValidTransformationAvailable()) {
		cv::Matx44f calibMatrix = calibrator->getRigidBodyTransformation();
		LOG->info("Rigid Body Transformation Matrix:\n%v", calibMatrix);

		this->next = *this->readyState;
	}
}

void CalibrationState::process(int key) {
	switch (key) {
		case 'a':
			LOG->trace("Key 'a' was pressed");
			// TODO abort calibration
			this->next = *this->initState;
			break;
	}
}

} // end namespace app
} // end namespace kinjo

