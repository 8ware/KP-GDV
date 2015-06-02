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
			std::size_t const & uiRecognitionAttemptCount,
			int const & iMorphSizeDilatePx,
			int const & iMorphSizeErodePx,
			int const & iGaussianBlurFilterWidthHalf,
			int const & iInvRatioAccuSize,
			int const & iMinCircleDistImageHeightPercent,
			int const & iCannyEdgeThreshold,
			int const & iHoughAccuThreshold,
			int const & iMinCircleRadiusImageHeightPercent,
			int const & iMaxCircleRadiusImageHeightPercent,
			int const & iMinHuePercent,
			int const & iMaxHuePercent,
			int const & iMinSatPercent,
			int const & iMinValPercent);

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
		std::size_t m_uiRecognitionAttemptCount;

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