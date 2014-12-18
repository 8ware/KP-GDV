#include <kinjo/calibration/Calibration.hpp>

#include <vector>		// std::vector
#include <cstdint>		// std::uint16_t

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

namespace kinjo
{
    namespace recognition
    {
        /**
        * Algorithm adapted from: http://wiki.elphel.com/index.php?title=OpenCV_Tennis_balls_recognizing_tutorial
		*
		* Step 1: Convert RGB -> HSV for better masking in step 2.
		* Step 2: Mask objects with the expected color to get only the calibration object.
		* Step 3: Morphological operations on mask to remove some occluders.
		* Step 4: Blur for better circle detection.
		* Step 5: Circle detection via Hough transform.
		* Step 6: Look up the circles vision position.
        **/
		cv::Vec3f getCalibrationObjectVisionPosition(
			cv::Mat const & matRgb, 
			cv::Mat const & matDepth)
		{
			// 1: Make a copy of the image and convert RGB to HSV color space.
			cv::Mat matHsv(matRgb.rows, matRgb.cols, CV_8UC3);
			cv::cvtColor(matRgb, matHsv, CV_BGR2HSV);

			//cv::imshow("HSV", matHsv);

			// 2: Mask the calibration object.
			// TODO:	Get a good color.
			//			Maybe by picking it first and giving the position as input value to get the current color from hsv?
			cv::Mat matMask(matHsv.rows, matHsv.cols, CV_8UC1);
			cv::inRange(
				matHsv,
				cv::Scalar(0.11*256, 0.60*256, 0.20*256, 0),
				cv::Scalar(0.14*256, 1.00*256, 1.00*256, 0),
				matMask);
			matHsv.release();

			cv::imshow("Mask", matMask);

			// 3: Morphological operations to enhance the mask.
			// CV_SHAPE_RECT is faster but a circle could lead to beter results.
			// TODO: Set morph size adaptively depending on image size.
			// Close.
			/*int const iMorphSizeClose(10);
			cv::Mat matStructureElementClose(cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(2*iMorphSizeClose+1, 2*iMorphSizeClose+1), cv::Point(iMorphSizeClose, iMorphSizeClose)));
			morphologyEx(matMask, matMask, cv::MORPH_OPEN, matStructureElementClose);
			matStructureElementClose.release();
			// Open.
			int const iMorphSizeOpen(5);
			cv::Mat matStructureElementOpen(cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(2*iMorphSizeOpen+1, 2*iMorphSizeOpen+1), cv::Point(iMorphSizeOpen, iMorphSizeOpen)));
			morphologyEx(matMask, matMask, cv::MORPH_OPEN, matStructureElementOpen);
			matStructureElementOpen.release();*/

			//cv::imshow("MaskFilter", matMask);

			// 4: Smooth the mask, otherwise a lot of false circles would be detected.
			// TODO: Play with the blur sizes.
			cv::GaussianBlur(matMask, matMask, cv::Size(19, 19), 0.0, 0.0);

			cv::imshow("MaskFilterBlur", matMask);

			// 5: Run the Hough transform.
			std::vector<cv::Vec3f> vv3fCircles;		// (x, y, radius)
			cv::HoughCircles(
				matMask, 
				vv3fCircles,
				CV_HOUGH_GRADIENT,
				2,									// TODO: Play with this parameter. Inverse ratio of the accumulator size. 1=input image res, 2=half res, ...
				matMask.rows/10,					// TODO: Play with tis parameter. Minimum circle distance. OpenCV example uses /4 but we only want one circle. Is it better to use high value?
				200,								// Canny edge detector threshold.
				40,									// TODO: Play with this parameter. Accumulator threshold. Higher->Less circles.
				2,									// Minimum circle radius.
				0);									// Maximum circle radius.

			cv::Mat matRgbCopy(matRgb.rows, matRgb.cols, CV_8UC3);
			matRgb.copyTo(matRgbCopy);
			typedef std::vector<cv::Vec3f>::const_iterator TIterator;
			TIterator const itCirclesEnd(vv3fCircles.end());
			for(TIterator itCircles(vv3fCircles.begin()); itCircles != itCirclesEnd; ++itCircles)
			{
				cv::Vec3f const & v3fCircle(*itCircles);
				cv::Point const v2iCenter(cvRound(v3fCircle[0]), cvRound(v3fCircle[1]));
				int const iRadius(cvRound(v3fCircle[2]));
				// Draw the center point.
				cv::circle(matRgbCopy, v2iCenter, 3, cv::Scalar(0, 255, 0, 0), -1, CV_AA, 0);
				// Draw the whole circle.
				cv::circle(matRgbCopy, v2iCenter, iRadius, cv::Scalar(255, 0, 0, 0), 3, CV_AA, 0);
			}
			cv::imshow("Recognition", matRgbCopy);

			//std::cout << "Recognized calibration object count: " << vv3fCircles.size() << std::endl;

			if(vv3fCircles.size()>0)
			{
				// 6: Just take the first circle because they are sorted by confidence.
				cv::Vec3f const v3fCircle(vv3fCircles[0]);
				// Get its center and radius.
				cv::Point const v2iCenter(cvRound(v3fCircle[0]), cvRound(v3fCircle[1]));
				//int const iRadius(cvRound(v3fCircle[2]));

				// Look up th depth at this position.
				std::uint16_t const fDepth(matDepth.at<std::uint16_t>(v2iCenter.y, v2iCenter.x));
				// FIXME: How to get the real x,y coordinates in vision?.
				// FIXME: How to scale z?
				return cv::Vec3f(v3fCircle[0], v3fCircle[1], fDepth);
			}
			else
			{
				return cv::Vec3f(0.0f, 0.0f, 0.0f);
			}
        }
    }
}