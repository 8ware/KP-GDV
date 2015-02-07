#pragma once

#include <opencv2/imgproc/imgproc.hpp>

#include <cstdint>

namespace kinjo {
namespace vision {

    class Vision
    {
	public:
		/**
		 * Updates the images.
		 **/
		virtual void updateImages(bool bRequireUpdates) = 0;

        /**
         * \return The depth image in millimeters.
         */
		virtual cv::Mat getDepth() const = 0;
		/**
		 * \return The color image.
		 */
		virtual cv::Mat getRgb() const = 0;

		/**
		 * \return The maximum depth value.
		 */
		virtual std::uint16_t getMaxDepthValue() const = 0;

		/**
		 * \return The 3d position in the vision.
		 **/
		virtual cv::Vec3f getPositionFromImagePointPx(
			cv::Point const & v2iPointPx) const = 0;

		/**
	 	 * Estimates the vision position from the 8 surrounding pixels and the given pixel only incorporating valid values.
		 *
		 * \return The 3d position in the vision.
		 **/
		cv::Vec3f estimateVisionPositionFromImagePointPx(
			cv::Point const & v2iPointPx) const;
    };

}
}

