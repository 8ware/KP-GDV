#pragma once

#include <kinjo/Vision.hpp>

namespace kinjo {

	class OpenNiVision : public Vision {

	public:
		/**
		 * \return the depth values normalized to centimetres.
		 */
		virtual cv::Mat getDepth() const override
		virtual cv::Mat getImage() const override;

	};

}

