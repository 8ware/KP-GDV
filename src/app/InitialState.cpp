#include <kinjo/app/InitialState.hpp>

#include <iostream>

#include <easylogging++.h>


static el::Logger* LOG = el::Loggers::getLogger("InitState");

namespace kinjo {
namespace app {

InitialState::InitialState(State** calibState) {
	this->designator = "INITIALIZED";
	this->infos.push_back("Press 'c' to start calibration!");

	this->next = this;
	this->calibState = calibState;
}

void InitialState::process(int key) {
	switch (key) {
		case 'c':
			LOG->trace("Key 'c' was pressed");
			this->next = *this->calibState;
			break;
	}
}

} // end namespace app
} // end namespace kinjo

