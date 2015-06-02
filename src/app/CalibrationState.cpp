#include <kinjo/app/CalibrationState.hpp>

#include <iostream>
#include <thread>


namespace kinjo {
namespace app {

CalibrationState::CalibrationState(State** initState, State** readyState,
		arm::Arm* arm, calibration::Calibrator* calibrator) {
	this->log = el::Loggers::getLogger("CalibState");

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
		log->info("Rigid Body Transformation Matrix:\n%v", calibMatrix);

		this->next = *this->readyState;
	}
}

void CalibrationState::process(int key) {
	switch (key) {
		case 'a':
			log->trace("Key 'a' was pressed");
			// TODO abort calibration
			this->next = *this->initState;
			break;
	}
}

} // end namespace app
} // end namespace kinjo

