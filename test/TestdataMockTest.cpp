
#include <iostream>
#include <memory>

#include <opencv2/highgui/highgui.hpp>

#include <kinjo/mock/DirectoryBasedDataProvider.hpp>
#include <kinjo/mock/TestdataMock.hpp>


using namespace cv;
using namespace std;
using namespace kinjo::arm;
using namespace kinjo::vision;
using namespace kinjo::calibration;
using namespace kinjo::mock;


static const int ESCAPE = 27;
static const int SPACE = 32;


int main(int argc, char* argv[]) {
	if (argc == 1)
		return 1;

	string directory = argv[1];
	shared_ptr<DataProvider> provider
		= make_shared<DirectoryBasedDataProvider>(directory);

	TestdataMock mock(provider);
	Arm *arm = &mock;
	Vision *vision = &mock;
	CalibrationPointGenerator *generator = &mock;

	uint16_t maxDepth = vision->getMaxDepthValue();
	float scaleFactor = numeric_limits<uint16_t>::max() / maxDepth;

	int imgLoopKey = -1;
	while (imgLoopKey != ESCAPE) {
		Vec3f position = generator->getNextCalibrationPoint();
		cout << "Moving arm to position " << position << endl;
		arm->moveTo(position);
		Vec3f actual = arm->getPosition();
		cout << "Reached actual position " << actual << endl;

		do
		{
			vision->updateImages(true);
			const Mat depth = vision->getDepth();
			const Mat rgb = vision->getRgb();
			imshow("DEPTH", depth * scaleFactor);
			imshow("RGB", rgb);

			imgLoopKey = waitKey(100);
		} while(imgLoopKey != SPACE && imgLoopKey != ESCAPE);
	}

	return 0;
}

