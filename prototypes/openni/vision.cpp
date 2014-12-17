
// standard libraries
#include <iostream>
#include <string>
#include <sstream>

// OpenNI
#include <XnCppWrapper.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace std;
/*
 * TODO maybe don't use namespaces to indicate
 *      affiliation of particular used classes
 */
using namespace cv;
using namespace xn;


#define TITLE "Vision"
#define TITLE_DEPTH "Vision (DEPTH)"
#define TITLE_RGB "Vision (RGB)"
/*
 * TODO get height and width from metadata:
 *
 *   DepthMetaData depth_metadata;
 *   depth_generator.GetMetaData(depth_metadata);
 *   int width = depth_metadata.YRes();
 *   int height = depth_metadata.XRes();
 */
#define ROWS 480
#define COLS 640
#define REFRESH_INTERVAL 40


void check_status(XnStatus& status, string what);
void translate(const XnDepthPixel* depth_pixels, Mat& depth_map);
void translate(const XnRGB24Pixel* rgb_pixels, Mat& rgb_map, Size size);
void update_coordinates(int event, int x, int y, int flags, void *param);


XnStatus status;
Context context;

DepthGenerator depth_generator;
ImageGenerator rgb_generator;

Point point = Point(-1, -1);

// http://denislantsman.com/?p=84
void convert_pixel_map(const XnRGB24Pixel* pImageMap, Mat& cv_image, int rows, int cols) {
	int sizes[2] = {rows, cols};
	cv_image = Mat(2, sizes, CV_8UC3, (void*) pImageMap);
}

Size getOutputSize(MapGenerator& generator) {
	XnMapOutputMode output;
	generator.GetMapOutputMode(output);
	return Size(output.nXRes, output.nYRes);
}

int main(void) {
	status = context.Init();
	check_status(status, "Initializing context");

	status = depth_generator.Create(context);
	check_status(status, "Creating depth generator");

	status = rgb_generator.Create(context);
	check_status(status, "Creating RGB generator");

	status = context.StartGeneratingAll();
	check_status(status, "Starting generation");

	Size depthSize = getOutputSize(depth_generator);
	Size rgbSize = getOutputSize(rgb_generator);

	namedWindow(TITLE_DEPTH);
	namedWindow(TITLE_RGB);
	setMouseCallback(TITLE_DEPTH, update_coordinates);
	setMouseCallback(TITLE_RGB, update_coordinates);

	Mat depth_mat;
	Mat rgb_mat;
	do {
		status = context.WaitAnyUpdateAll(); // And/Any/One/None
		check_status(status, "Updating depth/RGB data");

		const XnDepthPixel* depth_map = depth_generator.GetDepthMap();
		depth_mat = Mat(depthSize, CV_16UC1, (void*) depth_map);

		const XnRGB24Pixel* rgb_map = rgb_generator.GetRGB24ImageMap();
		rgb_mat = Mat(rgbSize, CV_8UC3, (void*) rgb_map);
//		translate(rgb_map, rgb_mat, rgbSize);

		Mat depth, rgb;
		depth_mat.copyTo(depth);
		rgb_mat.copyTo(rgb);

		if (0 <= point.x && point.x < depth_mat.cols
				&& 0 <= point.y && point.y < depth_mat.rows) {
			ushort distance = depth_mat.at<ushort>(point.y, point.x);
//			cout << "Distance: " << distance << "mm" << endl;
//			Scalar color = rgb_mat.at<Scalar>(point.y, point.x);
//			int color = rgb_mat.at<uchar>(point.y, point.x);
//			cout << "Color:    " << color << endl;

			Scalar d_color = Scalar(255 << 8);
			circle(depth, point, 3, d_color);
			circle(depth, point, 7, d_color);

			Scalar color = Scalar(0, 255, 0);
			circle(rgb, point, 3, color);
			circle(rgb, point, 7, color);

			if (distance > 0) {
				stringstream stream;
				stream << "[ " << distance << "mm ]";
				putText(depth, stream.str(), point + Point(15, -5), FONT_HERSHEY_SIMPLEX, 0.3, d_color);
				putText(rgb, stream.str(), point + Point(15, -5), FONT_HERSHEY_SIMPLEX, 0.3, color);
			}
//			point = Point(-1, -1);
		}

		imshow(TITLE_DEPTH, depth);
		imshow(TITLE_RGB, rgb);

		depth_mat.release();
		rgb_mat.release();
	} while (waitKey(REFRESH_INTERVAL) != 27);

	return 0;
}


void translate(const XnRGB24Pixel* rgb_pixels, Mat& rgb_map, Size size) {
	rgb_map = Mat(size, CV_8UC3);
	cout << "RGB=" << rgb_map.size() << endl;
	for (int y = 0; y < size.height; y++) {
		for (int x = 0; x < size.width; x++) {
			XnRGB24Pixel rgb = *rgb_pixels;
			Scalar bgr = Scalar(rgb.nBlue, rgb.nGreen, rgb.nRed);
			rgb_map.at<Scalar>(y, x) = bgr;
			rgb_pixels++;
		}
	}
}

void check_status(XnStatus& status, string what) {
	if (status != XN_STATUS_OK) {
		cout << what << " failed: " << xnGetStatusString(status) << endl;
		exit(-1);
	}
}

void update_coordinates(int event, int x, int y, int flags, void *param) {
	switch (event) {
		case CV_EVENT_LBUTTONUP:
			point = Point(x, y);
//			cout << point << endl;
			break;
	}
}

