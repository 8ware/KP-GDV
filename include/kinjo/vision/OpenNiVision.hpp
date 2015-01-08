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
			 * Updates the images.
			 * 
			 * \param bRequireUpdates	
			 *		If the thread should wait for new data from all inputs.
			 *		If this is false, the images are only updated if there is already new data.
			 **/
			virtual void updateImages(bool bRequireUpdates) override;

			/**
			 * \return the depth values normalized to centimeters.
			 */
			virtual cv::Mat const & getDepth() override;
			/**
			 * \return the RGB image already translated to OpenCV's internal
			 * BGR representation.
			 */
			virtual cv::Mat const & getRgb() override;

			/**
			 * \return The 3d position in the vision.
			 **/
			virtual cv::Vec3f getPositionFromImagePointPx(
				cv::Point const & v2iPointPx) override;

		private:
			/**
			 * Helper function to check the OpenNI status.
			 * \param action a message describing the action during which the
			 * error occurred
			 */
			static void checkStatus(XnStatus const & status, std::string const & action);
			/**
			 * Helper function to get the generator's output image size.
			 * \param generator the generator which produces image maps
			 * \return the (OpenCV) size of the generated image.
			 */
			cv::Size getImageSize(xn::MapGenerator const & generator) const;

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
			 * The RGB image.
			 */
			cv::Mat matRgb;
			/**
			 * The depth image.
			 */
			cv::Mat matDepth;
        };

    }
}

