
#include <kinjo/vision/Vision.hpp>
#include <kinjo/vision/OpenNiVision.hpp>

#include <sstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
using namespace cv;
using namespace kinjo::vision;


#define TITLE_DEPTH "Vision (Depth)"
#define TITLE_RGB "Vision (RGB)"
#define REFRESH_INTERVAL 100


void updateCoordinates(int event, int x, int y, int flags, void *param) {
	Point* point = static_cast<Point*>(param);

	switch (event) {
		case CV_EVENT_LBUTTONUP:
			*point = Point(x, y);
			break;
	}
}

void displayCoordinates(Mat& image, Point& point, Scalar& color) {
	circle(image, point, 3, color);
	circle(image, point, 7, color);
}

void displayDistance(Mat& image, Point& point, Scalar& color, ushort distance) {
	float dist_cm = distance / 10;
	stringstream stream;
	stream << "[ " << dist_cm << "cm ]";
	Point shifted = point + Point(15, -5);
	putText(image, stream.str(), shifted, FONT_HERSHEY_SIMPLEX, 0.3, color);
}

int main(void) {
	Vision* vision;
	vision = new OpenNiVision();

	Point point(-1, -1);
	namedWindow(TITLE_DEPTH);
	namedWindow(TITLE_RGB);
	setMouseCallback(TITLE_DEPTH, updateCoordinates, &point);
	setMouseCallback(TITLE_RGB, updateCoordinates, &point);

	uint16_t maxDepth = vision->getMaxDepthValue();
	uint16_t maxValue = numeric_limits<uint16_t>::max();
	float scaleFactor = maxValue / maxDepth;

	Scalar depthColor(maxValue);
	Scalar rgbColor(0, 255, 0);

	Mat depth, rgb;
	do {
		vision->updateImages(true);
		vision->getDepth().copyTo(depth);
		rgb = vision->getRgb();

		if (0 <= point.x && point.x < depth.cols
				&& 0 <= point.y && point.y < depth.rows) {
			displayCoordinates(depth, point, depthColor);
			displayCoordinates(rgb, point, rgbColor);

			ushort distance = depth.at<ushort>(point.y, point.x);
			if (distance > 0) {
				displayDistance(depth, point, depthColor, distance);
				displayDistance(rgb, point, rgbColor, distance);
			}

		}

		imshow(TITLE_DEPTH, depth * scaleFactor);
		imshow(TITLE_RGB, rgb);
	} while (waitKey(REFRESH_INTERVAL) != 27);

	return 0;
}

