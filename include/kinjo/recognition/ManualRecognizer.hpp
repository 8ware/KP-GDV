#pragma once

#include <kinjo/recognition/Recognizer.hpp>

namespace kinjo
{
	namespace recognition
	{
		class ManualRecognizer : public Recognizer
		{
		public:
			/**
			 * \return Constructor.
			 **/
			ManualRecognizer();

			/**
			 * \return The position of the calibration object in the image and its radius.
			 **/
			virtual cv::Point estimateCalibrationObjectImagePointPx(
				cv::Mat const & matRgb) const override;

		private:
			/**
			 * \return The mouse callback.
			 **/
			static void mouseCallback(int event, int x, int y, int /*flags*/, void * param);

		private:
			cv::Point mutable m_v2iClickPosPx;
			bool mutable m_bClicked;
		};
	}
}