#include <kinjo/app/ReadyState.hpp>

#include <iostream>

#include <opencv2/highgui/highgui.hpp>	// CV_EVENT_{L,R}BUTTONDBLCLK
#include <easylogging++.h>

#include <kinjo/util/Config.hpp>


static el::Logger* LOG = el::Loggers::getLogger("ReadyState");

namespace kinjo {
namespace app {

ReadyState::ReadyState(State** calibState, State** graspState,
		calibration::Calibrator* calibrator, std::string& matrixFilename) {
	this->designator = "IDLE";
	this->infos.push_back("Double click to grasp item!");
	this->infos.push_back("Press 'c' to re-calibrate!");
	this->infos.push_back("Press 's' to save calibration matrix!");

	this->next = this;
	this->calibState = calibState;
	this->graspState = graspState;

	this->calibrator = calibrator;
	this->matrixFilename = matrixFilename;
}

void ReadyState::process(int mouseEvent, cv::Point point) {
	switch (mouseEvent) {
		case CV_EVENT_LBUTTONDBLCLK:
			LOG->trace("Propagating mouse event to grasp state");
			// propagate mouse event to grasping state
			(*this->graspState)->process(mouseEvent, point);
			this->next = *this->graspState;
			break;
	}
}

void ReadyState::process(int key) {
	switch (key) {
		case 'c':
			LOG->trace("Key 'c' was pressed");
			this->next = *this->calibState;
			break;
		case 's':
			LOG->trace("Key 's' was pressed");
			LOG->info("Saved calibration matrix to '%v'", this->matrixFilename);
			cv::Matx44f calibMatrix = calibrator->getRigidBodyTransformation();
			util::Config::writeMatrixToFile(this->matrixFilename, calibMatrix);
			break;
	}
}

} // end namespace app
} // end namespace kinjo

