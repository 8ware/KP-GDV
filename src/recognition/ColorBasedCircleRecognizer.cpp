#include <kinjo/recognition/ColorBasedCircleRecognizer.hpp>

#include <opencv2/imgproc/imgproc.hpp>

#include <vector>		// std::vector
#include <cstdint>		// std::uint16_t

#include <iostream>

//#define KINJO_RECOGNITION_DEBUG
//#define KINJO_RECOGNITION_TWEAK_WINDOW

#if defined(KINJO_RECOGNITION_DEBUG) || defined(KINJO_RECOGNITION_TWEAK_WINDOW)
	#include <opencv2/highgui/highgui.hpp>
#endif

namespace kinjo
{
    namespace recognition
	{

		/**
		 * 
		 **/
		cv::Point ColorBasedCircleRecognizer::estimateCalibrationObjectImagePointPx(
			cv::Mat const & matRgb) const
		{
			auto const calibrationObjectPositionPx(estimateCalibrationObjectImagePointPxAndRadius(matRgb));

			return calibrationObjectPositionPx.first;
		}
        /**
         * Algorithm adapted from: http://wiki.elphel.com/index.php?title=OpenCV_Tennis_balls_recognizing_tutorial
		 *
		 * Step 1: Convert RGB -> HSV for better masking in step 2.
		 * Step 2: Mask objects with the expected color to get only the calibration object.
		 * Step 3: Morphological operations on mask to remove some occluders.
		 * Step 4: Blur for better circle detection.
		 * Step 5: Circle detection via Hough transform.
		 * Step 6: Look up the circles vision position.
		 * 
		 * \return	The position of the calibration object in pixels and the circle radius.
         **/
		std::pair<cv::Point, float> ColorBasedCircleRecognizer::estimateCalibrationObjectImagePointPxAndRadius(
			cv::Mat const & matRgb) const
		{
			static int iMorphSizeDilatePx(4);
			static int iMorphSizeErodePx(8);
			static int iGaussianBlurFilterWidthHalf(8);
			static int iInvRatioAccuSize(2);
			static int iMinCircleDistImageHeightPercent(50);
			static int iCannyEdgeThreshold(100);
			static int iHoughAccuThreshold(15);
			static int iMinCircleRadiusImageHeightPercent(1);
			static int iMaxCircleRadiusImageHeightPercent(25);
			static int iMinHuePercent(11);
			static int iMaxHuePercent(15);
			static int iMinSatPercent(30);
			static int iMinValPercent(40);

#ifdef KINJO_RECOGNITION_TWEAK_WINDOW
			static bool bInitialized(false);
			if(!bInitialized)
			{
				std::string const sRecognitionTweakWindowTitle("Recognition Tweak Window");
				cv::namedWindow(sRecognitionTweakWindowTitle, CV_WINDOW_NORMAL|CV_GUI_EXPANDED);
				cv::createTrackbar("MorphSizeDilatePx", sRecognitionTweakWindowTitle, &iMorphSizeDilatePx, 100, nullptr);
				cv::createTrackbar("MorphSizeErodePx", sRecognitionTweakWindowTitle, &iMorphSizeErodePx, 100, nullptr);
				cv::createTrackbar("BlurWidthHalf", sRecognitionTweakWindowTitle, &iGaussianBlurFilterWidthHalf, 50, nullptr);
				cv::createTrackbar("InvRatioAccuSize", sRecognitionTweakWindowTitle, &iInvRatioAccuSize, 10, nullptr);
				cv::createTrackbar("MinCircleDist%", sRecognitionTweakWindowTitle, &iMinCircleDistImageHeightPercent, 100, nullptr);
				cv::createTrackbar("EdgeThreashold", sRecognitionTweakWindowTitle, &iCannyEdgeThreshold, 1000, nullptr);
				cv::createTrackbar("HoughThreashold", sRecognitionTweakWindowTitle, &iHoughAccuThreshold, 500, nullptr);
				cv::createTrackbar("MinCircleRad%", sRecognitionTweakWindowTitle, &iMinCircleRadiusImageHeightPercent, 100, nullptr);
				cv::createTrackbar("MaxCircleRad%", sRecognitionTweakWindowTitle, &iMaxCircleRadiusImageHeightPercent, 100, nullptr);
				cv::createTrackbar("MinHue%", sRecognitionTweakWindowTitle, &iMinHuePercent, 100, nullptr);
				cv::createTrackbar("MaxHue%", sRecognitionTweakWindowTitle, &iMaxHuePercent, 100, nullptr);
				cv::createTrackbar("MinSat%", sRecognitionTweakWindowTitle, &iMinSatPercent, 100, nullptr);
				cv::createTrackbar("MinVal%", sRecognitionTweakWindowTitle, &iMinValPercent, 100, nullptr);
				bInitialized = true;
			}
			// Fix some inputs not allowed to be zero.
			if(iMorphSizeDilatePx==0) { iMorphSizeDilatePx = 1; }
			if(iMorphSizeErodePx==0) { iMorphSizeErodePx = 1; }
			if(iInvRatioAccuSize==0) { iInvRatioAccuSize = 1; }
			if(iCannyEdgeThreshold==0) { iCannyEdgeThreshold = 1; }
			if(iHoughAccuThreshold==0) { iHoughAccuThreshold = 1; }
			if(iMinCircleDistImageHeightPercent==0) { iMinCircleDistImageHeightPercent = 1; }

#endif
			// 1: Make a copy of the image and convert RGB to HSV color space.
			cv::Mat matHsv(matRgb.rows, matRgb.cols, CV_8UC3);
			cv::cvtColor(matRgb, matHsv, CV_BGR2HSV);

#ifdef KINJO_RECOGNITION_DEBUG
			cv::imshow("HSV", matHsv);
#endif
			// 2: Mask the calibration object.
			// TODO: Maybe pick it first and give the position as input value to get the current color from hsv?
			cv::Mat matMask(matHsv.rows, matHsv.cols, CV_8UC1);
			double fMinHue(static_cast<double>(iMinHuePercent)/100.0);
			double fMaxHue(static_cast<double>(iMaxHuePercent)/100.0);
			double fMinSat(static_cast<double>(iMinSatPercent)/100.0);
			double fMinVal(static_cast<double>(iMinValPercent)/100.0);
			cv::inRange(
				matHsv,
				cv::Scalar(fMinHue*255, fMinSat*255, fMinVal*255, 0),
				cv::Scalar(fMaxHue*255, 1.00*255, 1.00*255, 0),
				matMask);
			matHsv.release();

#ifdef KINJO_RECOGNITION_DEBUG
			cv::imshow("Mask", matMask);
#endif

			// 3: Morphological operations to enhance the mask.
			// TODO: Set morph size adaptively depending on image size.

			// Dilation
			cv::Mat matStructureElementDilation(
				cv::getStructuringElement(
				cv::MORPH_ELLIPSE,
				cv::Size(2*iMorphSizeDilatePx+1, 2*iMorphSizeDilatePx+1),
				cv::Point(iMorphSizeDilatePx, iMorphSizeDilatePx)));
			cv::morphologyEx(matMask, matMask, cv::MORPH_DILATE, matStructureElementDilation);
			matStructureElementDilation.release();

			// Erosion.
			cv::Mat matStructureElementErosion(
				cv::getStructuringElement(
				cv::MORPH_ELLIPSE,
				cv::Size(2*iMorphSizeErodePx+1, 2*iMorphSizeErodePx+1),
				cv::Point(iMorphSizeErodePx, iMorphSizeErodePx)));
			cv::morphologyEx(matMask, matMask, cv::MORPH_ERODE, matStructureElementErosion);
			matStructureElementErosion.release();

#ifdef KINJO_RECOGNITION_DEBUG
			cv::imshow("MaskFilter", matMask);
#endif

			// 4: Smooth the mask, otherwise a lot of false circles would be detected.
			cv::GaussianBlur(
				matMask, 
				matMask, 
				cv::Size(
					2*iGaussianBlurFilterWidthHalf+1,
					2*iGaussianBlurFilterWidthHalf+1),
				0.0, 
				0.0);

#ifdef KINJO_RECOGNITION_DEBUG
			cv::imshow("MaskFilterBlur", matMask);
#endif

			// 5: Run the Hough transform.
			std::vector<cv::Vec3f> vv3fCircles;		// (x, y, radius)
			int const iMinCircleDistPx(static_cast<int>(
				static_cast<float>(matMask.rows)
				* (static_cast<float>(iMinCircleDistImageHeightPercent)/100.0f)));
			int const iMinCircleRadiusPx(static_cast<int>(
				static_cast<float>(matMask.rows)
				* (static_cast<float>(iMinCircleRadiusImageHeightPercent)/100.0f)));
			int const iMaxCircleRadiusPx(static_cast<int>(
				static_cast<float>(matMask.rows)
				* (static_cast<float>(iMaxCircleRadiusImageHeightPercent)/100.0f)));

			cv::HoughCircles(
				matMask,
				vv3fCircles,
				CV_HOUGH_GRADIENT,
				iInvRatioAccuSize,		// Inverse ratio of the accumulator size. 1=input image res, 2=half res, ...
				iMinCircleDistPx,		// Minimum circle distance. OpenCV example uses /4 but we only want one circle. Is it better to use high value?
				iCannyEdgeThreshold,	// Canny edge detector threshold.
				iHoughAccuThreshold,	// Accumulator threshold. Higher->Less circles.
				iMinCircleRadiusPx,		// Minimum circle radius.
				iMaxCircleRadiusPx);	// Maximum circle radius.

#ifdef KINJO_RECOGNITION_DEBUG
			cv::Mat matRgbCopy(matRgb.rows, matRgb.cols, CV_8UC3);
			matRgb.copyTo(matRgbCopy);
			for(auto && v3fCircle : vv3fCircles)
			{
				cv::Point const v2iCenter(cvRound(v3fCircle[0]), cvRound(v3fCircle[1]));
				int const iRadius(cvRound(v3fCircle[2]));
				// Draw the center point.
				cv::circle(matRgbCopy, v2iCenter, 3, cv::Scalar(0, 255, 0, 0), -1, CV_AA, 0);
				// Draw the whole circle.
				cv::circle(matRgbCopy, v2iCenter, iRadius, cv::Scalar(255, 0, 0, 0), 3, CV_AA, 0);
			}
			cv::imshow("Recognition Result", matRgbCopy);
#endif

			if(vv3fCircles.size()>0u)
			{
				// 6: Just take the first circle because they are sorted by confidence.
				cv::Vec3f const v3fCircle(vv3fCircles[0u]);

				cv::Point const v2iCenter(
					cvRound(v3fCircle[0]),
					cvRound(v3fCircle[1]));

				return std::make_pair(v2iCenter, v3fCircle[2]);
			}
			else
			{
				return std::make_pair(cv::Point(0, 0), 0.0f);
			}
        }
    }
}