
#include <kinjo/vision/OpenNiVision.hpp>

#include <iostream>
#include <cstdint>		// std::uint16_t

namespace kinjo {
    namespace vision {

		OpenNiVision::OpenNiVision()
		{
			checkStatus(context.Init(), "Initializing context");

			checkStatus(depthGenerator.Create(context), "Creating depth generator");
			depthGenerator.SetIntProperty("nearMode", 1u);

			checkStatus(rgbGenerator.Create(context), "Creating RGB generator");

			checkStatus(context.StartGeneratingAll(), "Starting generation");

			depthImageSize = getImageSize(depthGenerator);
			rgbImageSize = getImageSize(rgbGenerator);

			// Align the depth image center to the color image center.
			checkStatus(depthGenerator.GetAlternativeViewPointCap().SetViewPoint(rgbGenerator), "GetAlternativeViewPointCap().SetViewPoint");
		}

		void OpenNiVision::updateImages(bool bRequireUpdates)
		{
			// Update the internal generator buffers if there is new data.
			if(bRequireUpdates)
			{
				checkStatus(context.WaitAndUpdateAll(), "WaitAndUpdateAll");
			}
			else
			{
				checkStatus(context.WaitNoneUpdateAll(), "WaitNoneUpdateAll");
			}

			// Update the depth data.
			const XnDepthPixel* depthPixels = depthGenerator.GetDepthMap();
			matDepth = cv::Mat(depthImageSize, CV_16UC1, const_cast<void*>(reinterpret_cast<void const *>(depthPixels)));

			// Update the color data.
			const XnRGB24Pixel* rgbPixels = rgbGenerator.GetRGB24ImageMap();
			cv::Mat bgr(rgbImageSize, CV_8UC3, const_cast<void*>(reinterpret_cast<void const *>(rgbPixels)));
			cv::cvtColor(bgr, matRgb, CV_RGB2BGR);
		}

		cv::Mat const & OpenNiVision::getDepth() const
		{
			return matDepth;
		}

		cv::Mat const & OpenNiVision::getRgb() const
		{
			return matRgb;
		}

		std::uint16_t OpenNiVision::getMaxDepthValue() const
		{
			return depthGenerator.GetDeviceMaxDepth();
		}

		cv::Vec3f OpenNiVision::getPositionFromImagePointPx(
			cv::Point const & v2iPointPx) const
		{
			// Look up the depth at this position.
			std::uint16_t const uiDepth(getDepth().at<std::uint16_t>(v2iPointPx.y, v2iPointPx.x));
			XnPoint3D const v3fProjectivePoint{
				static_cast<XnFloat>(v2iPointPx.x), 
				static_cast<XnFloat>(v2iPointPx.y), 
				static_cast<XnFloat>(uiDepth)};
			XnPoint3D v3fRealPoint;
			depthGenerator.ConvertProjectiveToRealWorld(1, &v3fProjectivePoint, &v3fRealPoint);

			return cv::Vec3f(
				static_cast<float>(v3fRealPoint.X),
				static_cast<float>(v3fRealPoint.Y),
				(v3fRealPoint.Z==0.0f)
					? 0.0f 
					: static_cast<float>(v3fRealPoint.Z));
		}

		void OpenNiVision::checkStatus(
			XnStatus const & status, 
			std::string const & action)
		{
			if (status != XN_STATUS_OK) {
				std::string cause = xnGetStatusString(status);
				std::stringstream ss;
				ss << action << " failed: " << cause << std::endl;
				throw std::runtime_error(ss.str().c_str());
			}
		}

		cv::Size OpenNiVision::getImageSize(
			xn::MapGenerator const & generator) const
		{
			XnMapOutputMode output;
			generator.GetMapOutputMode(output);
			return cv::Size(output.nXRes, output.nYRes);
		}

    }
}

