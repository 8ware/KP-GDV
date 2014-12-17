#pragma once

#include <kinjo/vision/Vision.hpp>

#include <string>

// OpenNI
#include <XnCppWrapper.h>


namespace kinjo {
    namespace vision {

        class OpenNiVision : public Vision
        {

        public:
			/**
			 * The default constructor.
			 */
			OpenNiVision();

			/**
			 * \return the depth values normalized to centimeters.
			 */
			virtual cv::Mat getDepth();
			/**
			 * \return the RGB image already translated to OpenCV's internal
			 * BGR representation.
			 */
			// TODO maybe rename to getBgr?
			virtual cv::Mat getRgb();

		private:
			/**
			 * The status which is set by OpenNI.
			 */
			XnStatus status;
			/**
			 * The context which keeps the depth- and RGB-production nodes.
			 */
			xn::Context context;
			/**
			 * The actual depth image generator (production node).
			 */
			xn::DepthGenerator depthGenerator;
			/**
			 * The size of the depth-generator's images.
			 */
			cv::Size depthImageSize;
			/**
			 * The actual RGB image generator (production node).
			 */
			xn::ImageGenerator rgbGenerator;
			/**
			 * The size of the RGB-generator's images.
			 */
			cv::Size rgbImageSize;

			/**
			 * Helper function to check the OpenNI status.
			 * \param action a message describing the action during which the
			 * error occurred
			 */
			void checkStatus(std::string action) const;
			/**
			 * Helper function to get the generator's output image size.
			 * \param generator the generator which produces image maps
			 * \return the (OpenCV) size of the generated image.
			 */
			cv::Size getImageSize(xn::MapGenerator& generator) const;
        };

    }
}

