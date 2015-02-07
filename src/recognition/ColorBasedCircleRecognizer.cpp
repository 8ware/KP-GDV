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
		ColorBasedCircleRecognizer::ColorBasedCircleRecognizer(
			int iMorphSizeDilatePx,
			int iMorphSizeErodePx,
			int iGaussianBlurFilterWidthHalf,
			int iInvRatioAccuSize,
			int iMinCircleDistImageHeightPercent,
			int iCannyEdgeThreshold,
			int iHoughAccuThreshold,
			int iMinCircleRadiusImageHeightPercent,
			int iMaxCircleRadiusImageHeightPercent,
			int iMinHuePercent,
			int iMaxHuePercent,
			int iMinSatPercent,
			int iMinValPercent) :
				m_iMorphSizeDilatePx(iMorphSizeDilatePx),
				m_iMorphSizeErodePx(iMorphSizeErodePx),
				m_iGaussianBlurFilterWidthHalf(iGaussianBlurFilterWidthHalf),
				m_iInvRatioAccuSize(iInvRatioAccuSize),
				m_iMinCircleDistImageHeightPercent(iMinCircleDistImageHeightPercent),
				m_iCannyEdgeThreshold(iCannyEdgeThreshold),
				m_iHoughAccuThreshold(iHoughAccuThreshold),
				m_iMinCircleRadiusImageHeightPercent(iMinCircleRadiusImageHeightPercent),
				m_iMaxCircleRadiusImageHeightPercent(iMaxCircleRadiusImageHeightPercent),
				m_iMinHuePercent(iMinHuePercent),
				m_iMaxHuePercent(iMaxHuePercent),
				m_iMinSatPercent(iMinSatPercent),
				m_iMinValPercent(iMinValPercent)
		{}

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
#ifdef KINJO_RECOGNITION_TWEAK_WINDOW
			static bool bInitialized(false);
			if(!bInitialized)
			{
				std::string const sRecognitionTweakWindowTitle("Recognition Tweak Window");
				cv::namedWindow(sRecognitionTweakWindowTitle, CV_WINDOW_NORMAL|CV_GUI_EXPANDED);
				cv::createTrackbar("MorphSizeDilatePx", sRecognitionTweakWindowTitle, &m_iMorphSizeDilatePx, 100, nullptr);
				cv::createTrackbar("MorphSizeErodePx", sRecognitionTweakWindowTitle, &m_iMorphSizeErodePx, 100, nullptr);
				cv::createTrackbar("BlurWidthHalf", sRecognitionTweakWindowTitle, &m_iGaussianBlurFilterWidthHalf, 50, nullptr);
				cv::createTrackbar("InvRatioAccuSize", sRecognitionTweakWindowTitle, &m_iInvRatioAccuSize, 10, nullptr);
				cv::createTrackbar("MinCircleDist%", sRecognitionTweakWindowTitle, &m_iMinCircleDistImageHeightPercent, 100, nullptr);
				cv::createTrackbar("EdgeThreashold", sRecognitionTweakWindowTitle, &m_iCannyEdgeThreshold, 1000, nullptr);
				cv::createTrackbar("HoughThreashold", sRecognitionTweakWindowTitle, &m_iHoughAccuThreshold, 500, nullptr);
				cv::createTrackbar("MinCircleRad%", sRecognitionTweakWindowTitle, &m_iMinCircleRadiusImageHeightPercent, 100, nullptr);
				cv::createTrackbar("MaxCircleRad%", sRecognitionTweakWindowTitle, &m_iMaxCircleRadiusImageHeightPercent, 100, nullptr);
				cv::createTrackbar("MinHue%", sRecognitionTweakWindowTitle, &m_iMinHuePercent, 100, nullptr);
				cv::createTrackbar("MaxHue%", sRecognitionTweakWindowTitle, &m_iMaxHuePercent, 100, nullptr);
				cv::createTrackbar("MinSat%", sRecognitionTweakWindowTitle, &m_iMinSatPercent, 100, nullptr);
				cv::createTrackbar("MinVal%", sRecognitionTweakWindowTitle, &m_iMinValPercent, 100, nullptr);
				bInitialized = true;
			}
			// Fix some inputs not allowed to be zero.
			if(m_iMorphSizeDilatePx==0) { m_iMorphSizeDilatePx = 1; }
			if(m_iMorphSizeErodePx==0) { m_iMorphSizeErodePx = 1; }
			if(m_iInvRatioAccuSize==0) { m_iInvRatioAccuSize = 1; }
			if(m_iCannyEdgeThreshold==0) { m_iCannyEdgeThreshold = 1; }
			if(m_iHoughAccuThreshold==0) { m_iHoughAccuThreshold = 1; }
			if(m_iMinCircleDistImageHeightPercent==0) { m_iMinCircleDistImageHeightPercent = 1; }

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
			double fMinHue(static_cast<double>(m_iMinHuePercent)/100.0);
			double fMaxHue(static_cast<double>(m_iMaxHuePercent)/100.0);
			double fMinSat(static_cast<double>(m_iMinSatPercent)/100.0);
			double fMinVal(static_cast<double>(m_iMinValPercent)/100.0);
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
				cv::Size(2*m_iMorphSizeDilatePx+1, 2*m_iMorphSizeDilatePx+1),
				cv::Point(m_iMorphSizeDilatePx, m_iMorphSizeDilatePx)));
			cv::morphologyEx(matMask, matMask, cv::MORPH_DILATE, matStructureElementDilation);
			matStructureElementDilation.release();

			// Erosion.
			cv::Mat matStructureElementErosion(
				cv::getStructuringElement(
				cv::MORPH_ELLIPSE,
				cv::Size(2*m_iMorphSizeErodePx+1, 2*m_iMorphSizeErodePx+1),
				cv::Point(m_iMorphSizeErodePx, m_iMorphSizeErodePx)));
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
					2*m_iGaussianBlurFilterWidthHalf+1,
					2*m_iGaussianBlurFilterWidthHalf+1),
				0.0, 
				0.0);

#ifdef KINJO_RECOGNITION_DEBUG
			cv::imshow("MaskFilterBlur", matMask);
#endif

			// 5: Run the Hough transform.
			std::vector<cv::Vec3f> vv3fCircles;		// (x, y, radius)
			int const iMinCircleDistPx(static_cast<int>(
				static_cast<float>(matMask.rows)
				* (static_cast<float>(m_iMinCircleDistImageHeightPercent)/100.0f)));
			int const iMinCircleRadiusPx(static_cast<int>(
				static_cast<float>(matMask.rows)
				* (static_cast<float>(m_iMinCircleRadiusImageHeightPercent)/100.0f)));
			int const iMaxCircleRadiusPx(static_cast<int>(
				static_cast<float>(matMask.rows)
				* (static_cast<float>(m_iMaxCircleRadiusImageHeightPercent)/100.0f)));

			cv::HoughCircles(
				matMask,
				vv3fCircles,
				CV_HOUGH_GRADIENT,
				m_iInvRatioAccuSize,	// Inverse ratio of the accumulator size. 1=input image res, 2=half res, ...
				iMinCircleDistPx,		// Minimum circle distance. OpenCV example uses /4 but we only want one circle. Is it better to use high value?
				m_iCannyEdgeThreshold,	// Canny edge detector threshold.
				m_iHoughAccuThreshold,	// Accumulator threshold. Higher->Less circles.
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
		/**
		 * 
		 **/
		std::size_t ColorBasedCircleRecognizer::getRecommendedRecognitionAttempCount() const
		{
			return 3;
		}
    }
}