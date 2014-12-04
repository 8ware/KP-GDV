
// standard libraries
#include <iostream>
#include <string>

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
void update_coordinates(int event, int x, int y, int flags, void *param);


XnStatus status;
Context context;

DepthGenerator depth_generator;

Point coords = Point(-1, -1);


int main(void) {
	status = context.Init();
	check_status(status, "Initializing context");

	status = depth_generator.Create(context);
	check_status(status, "Creating depth generator");

	status = context.StartGeneratingAll();
	check_status(status, "Starting generation");

	namedWindow(TITLE);
	setMouseCallback(TITLE, update_coordinates);

	Mat depth_mat(ROWS, COLS, CV_16UC1);
	do {
		status = context.WaitOneUpdateAll(depth_generator);
		check_status(status, "Updating depth data");

		const XnDepthPixel* depth_map = depth_generator.GetDepthMap();
		translate(depth_map, depth_mat);

		if (coords.x >= 0 && coords.y >= 0) {
			ushort distance = depth_mat.at<ushort>(coords.y, coords.x);
			cout << "Distance: " << distance << "mm" << endl;
			coords = Point(-1, -1);
		}

		imshow(TITLE, depth_mat);
	} while (waitKey(REFRESH_INTERVAL) != 27);

	return 0;
}


void translate(const XnDepthPixel* depth_pixels, Mat& depth_map) {
	for (int y = 0; y < ROWS; y++) {
		for (int x = 0; x < COLS; x++, depth_pixels++) {
			XnDepthPixel depth = *depth_pixels;
			depth_map.at<ushort>(y, x) = depth;
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
			coords = Point(x, y);
			break;
	}
}

