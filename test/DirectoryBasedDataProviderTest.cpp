
#include <iostream>
#include <algorithm>
#include <memory>

#include <opencv2/highgui/highgui.hpp>

#include <kinjo/mock/DirectoryBasedDataProvider.hpp>


using namespace cv;
using namespace std;
using namespace kinjo::mock;


static const float MAX_DEPTH = 5000.0f;
static const float SCALE_FACTOR = numeric_limits<uint16_t>::max() / MAX_DEPTH;


int main(int argc, char* argv[]) {
	if (argc == 1)
		return 1;

	string directory = argv[1];
	shared_ptr<DataProvider> provider
		= make_shared<DirectoryBasedDataProvider>(directory);
	vector<pair<Vec3f, Vec3f>> positions = provider->getPositions();

	Vec3f position = positions[1].first;
	cout << "Showing images for position " << position << endl;

	vector<Mat> depths = provider->getDepthImages(position);
	vector<Mat> rgbs = provider->getRgbImages(position);

	int idx = 0;
	int size = std::min(depths.size(), rgbs.size());
	do
	{
		imshow("DEPTH", depths[idx] * SCALE_FACTOR);
		imshow("RGB", rgbs[idx]);
		idx = (idx + 1) % size;
	} while(waitKey(100) != 27);

	return 0;
}

