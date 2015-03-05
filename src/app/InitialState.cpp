#include <kinjo/app/InitialState.hpp>

#include <iostream>


namespace kinjo {
namespace app {

InitialState::InitialState(State** calibState) {
	this->log = el::Loggers::getLogger("InitialState");

	this->designator = "INITIALIZED";
	this->infos.push_back("Press 'c' to start calibration!");

	this->next = this;
	this->calibState = calibState;
}

void InitialState::process(int key) {
	switch (key) {
		case 'c':
			log->trace("Key 'c' was pressed");
			this->next = *this->calibState;
			break;
	}
}

} // end namespace app
} // end namespace kinjo

