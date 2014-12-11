#include <kinjo/arm/ArmFactory.hpp>

int main(int argc, char* argv[]){

	auto const Arm = kinjo::arm::ArmFactory::getInstance(1);

	return 0;
}