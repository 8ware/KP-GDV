#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {

	class Vision {

	public:
		/**
		 * \return the depth values normalized to centimetres.
		 */
		virtual cv::Mat getDepth() const = 0;
		virtual cv::Mat getImage() const = 0;

	};

}

