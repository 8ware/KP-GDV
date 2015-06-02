
#include <iostream>
#include <cstdlib>
#include <ctime>

#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/vision/OpenNiVision.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


static const char* EXTENSION = "png";
static const int DEFAULT_COUNT = 10;
static const int BASE_OFFSET = 10;

static const int NEG_POS = 2;
static const int POS_ONLY = 1;


cv::Vec3f asVector(int x, int y, int z) {
	float x_f = static_cast<float>(x);
	float y_f = static_cast<float>(y);
	float z_f = static_cast<float>(z);

	cv::Vec3f vector(x_f, y_f, z_f);

	return vector;
}

std::string generateFilename(int run, const char* type) {
	char number[3];
	std::sprintf(number, "%03i", run);

	std::stringstream stream;
	stream << number << "_" << type << '.' << EXTENSION;

	return stream.str();
}

float randomNumber(int range, int flag = NEG_POS) {
	int number = rand() % (range - BASE_OFFSET) + BASE_OFFSET;
	int sign = rand() % flag == 0 ? +1 : -1;

	return static_cast<float>(sign * number);
}

cv::Vec3f generateRandomPosition() {
	float x = randomNumber(50);
	float y = randomNumber(40);
	float z = randomNumber(50, POS_ONLY);

	return cv::Vec3f(x, y, z);
}

int main(int argc, const char** argv) {
	int count = DEFAULT_COUNT;
	if (argc < 1)
		std::istringstream(argv[1]) >> count;

	std::shared_ptr<kinjo::arm::Arm> Arm
		= kinjo::arm::ArmFactory::getInstance();
	std::shared_ptr<kinjo::vision::Vision> vision
		= std::make_shared<kinjo::vision::OpenNiVision>();

	srand(time(NULL));
	for (int run = 1; run <= count; ++run) {
		cv::Vec3f position = generateRandomPosition();
		std::cout << "Generating data at position " << position << std::endl;
	}


	int x, y, z;
	std::istringstream(argv[1]) >> x;
	std::istringstream(argv[2]) >> y;
	std::istringstream(argv[3]) >> z;

	std::cerr << "Generating data for at JacoArm-position ("
		<< x << "," << y << "," << z << ")\n";

	std::shared_ptr<kinjo::arm::Arm> Arm
		= kinjo::arm::ArmFactory::getInstance();
	std::shared_ptr<kinjo::vision::Vision> vision
		= std::make_shared<kinjo::vision::OpenNiVision>();

//	cv::Vec3f initial(20.8491f, -27.63f, 48.6976f);
//	Arm->moveTo(initial);
	std::cerr << "Current position: " << Arm->getPosition() << std::endl;

	cv::Vec3f position = asVector(x, y, z);
	cv::Vec3f actual = position;
	if (!(x > 9000 && y > 9000 && z > 9000)) {
		Arm->moveTo(position);
		actual = Arm->getPosition();
	}

	// do not record if a fourth arbitrary parameter is given
	if (argc > 4)
		return 0;

	for (int i = 1; i <= 10; ++i) {
		std::string depthFn = generateFilename(i, "depth");
		std::string imageFn = generateFilename(i, "image");

		std::cout << depthFn << " | " << imageFn << " : " << position
			<< " | " << actual << std::endl;

		cv::Mat depth = vision->getDepth();
		cv::Mat image = vision->getRgb();

		cv::imwrite(depthFn, depth);
		cv::imwrite(imageFn, image);
	}
	
	return 0;
}

