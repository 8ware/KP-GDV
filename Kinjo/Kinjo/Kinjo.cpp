
#include <kinjo/ArmFactory.hpp>

int main(int argc, char* argv[]){

	kinjo::Arm* TheArm = kinjo::ArmFactory::getInstance(1);

	return 0;
}