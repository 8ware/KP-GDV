#pragma once

#include <kinjo/recognition/Recognizer.hpp>

namespace kinjo {
namespace recognition {

	class ColorBasedCircleRecognizer : public Recognizer
	{
	public:
		/**
		 * Constructor
		 **/
		ColorBasedCircleRecognizer(
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
			int iMinValPercent);

		/**
		 * \return The position of the calibration object in the image.
		 **/
		virtual cv::Point estimateCalibrationObjectImagePointPx(
			cv::Mat const & matRgb) const override;

		/**
		 * \return The position of the calibration object in the image and its radius.
		 **/
		std::pair<cv::Point, float> estimateCalibrationObjectImagePointPxAndRadius(
			cv::Mat const & matRgb) const;
			
		/**
	     * \return The number of recognition attempts that should be made.
		 **/
		virtual std::size_t getRecommendedRecognitionAttempCount() const override;

	private:
		int m_iMorphSizeDilatePx;
		int m_iMorphSizeErodePx;
		int m_iGaussianBlurFilterWidthHalf;
		int m_iInvRatioAccuSize;
		int m_iMinCircleDistImageHeightPercent;
		int m_iCannyEdgeThreshold;
		int m_iHoughAccuThreshold;
		int m_iMinCircleRadiusImageHeightPercent;
		int m_iMaxCircleRadiusImageHeightPercent;
		int m_iMinHuePercent;
		int m_iMaxHuePercent;
		int m_iMinSatPercent;
		int m_iMinValPercent;
	};
}
}