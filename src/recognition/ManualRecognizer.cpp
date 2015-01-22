#include <kinjo/recognition/ManualRecognizer.hpp>

#include <opencv2/highgui/highgui.hpp>

#include <thread>

namespace kinjo
{
	namespace recognition
	{
		/**
		 *
		 **/
		ManualRecognizer::ManualRecognizer() :
			m_bClicked(false)
		{}
		/**
		 *
		 **/
		cv::Point ManualRecognizer::estimateCalibrationObjectImagePointPx(
			cv::Mat const & matRgb) const
		{
			m_bClicked = false;

			cv::imshow("ManualRecognizer", matRgb);
			cv::setMouseCallback("ManualRecognizer", mouseCallback, const_cast<ManualRecognizer *>(this));

			do
			{
				cv::waitKey(10); ;
			}
			while(!m_bClicked);

			return m_v2iClickPosPx;
		}
		/**
		 * 
		 **/
		std::size_t ManualRecognizer::getRecommendedRecognitionAttempCount() const
		{
			return 1;
		}
		/**
		 *
		 **/
		void ManualRecognizer::mouseCallback(int event, int x, int y, int , void * param)
		{
			ManualRecognizer * this_ = static_cast<ManualRecognizer*>(param);

			switch(event)
			{
			case CV_EVENT_LBUTTONUP:
				this_->m_v2iClickPosPx = cv::Point(x, y);
				this_->m_bClicked = true;
				break;
			}
		}
	}
}