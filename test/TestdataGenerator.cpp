
#include <iostream>

#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/vision/OpenNiVision.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, const char** argv) {
	if (argc < 4) {
		std::cerr << "Too less arguments: expected X, Y and Z\n";
		return 1;
	}

	int x, y, z;
	std::istringstream(argv[1]) >> x;
	std::istringstream(argv[2]) >> y;
	std::istringstream(argv[3]) >> z;

	std::cout << "Generating data for at JacoArm-position ("
		<< x << "," << y << "," << z << ")\n";
	
	return 0;
}

