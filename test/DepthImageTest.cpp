
#include <iostream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace cv;
using namespace std;


static const float MAX_DEPTH = 5000.0f;
static const float SCALE_FACTOR = numeric_limits<uint16_t>::max() / MAX_DEPTH;


void updateCoordinates(int event, int x, int y, int /*flags*/, void *param)
{
	cv::Point* point = static_cast<cv::Point*>(param);

	switch(event)
	{
	case CV_EVENT_LBUTTONUP:
		*point = cv::Point(x, y);
		break;
	}
}

void displayCoordinates(cv::Mat& image, cv::Point const & point, cv::Scalar const & color)
{
	circle(image, point, 3, color);
	circle(image, point, 7, color);
}

void displayDistance(cv::Mat& image, cv::Point const & point, cv::Scalar const & color, ushort distance)
{
	float dist_cm = static_cast<float>(distance) / 10.0f;
	std::stringstream stream;
	stream << "[ " << dist_cm << "cm ]";
	cv::Point shifted = point + cv::Point(15, -5);
	putText(image, stream.str(), shifted, cv::FONT_HERSHEY_SIMPLEX, 0.3, color);
}

int main(int argc, const char** argv) {
	Mat original = imread(argv[1], CV_LOAD_IMAGE_ANYDEPTH);

	Point point(-1, -1);
	namedWindow("DEPTH");
	setMouseCallback("DEPTH", updateCoordinates, &point);

	Mat depth;
	cv::Scalar const depthColor = cv::Scalar(255 << 8);
	do
	{
		original.copyTo(depth);
		if(0 <= point.x && point.x < depth.cols
			&& 0 <= point.y && point.y < depth.rows)
		{
			displayCoordinates(depth, point, depthColor);

			ushort distance = depth.at<ushort>(point.y, point.x);
			if(distance > 0)
			{
				displayDistance(depth, point, depthColor, distance);
			}
		}

		imshow("DEPTH", depth * SCALE_FACTOR);
	} while(waitKey(100) != 27);
}

