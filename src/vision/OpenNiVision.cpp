
#include <kinjo/vision/OpenNiVision.hpp>

#include <iostream>


namespace kinjo {
    namespace vision {

		OpenNiVision::OpenNiVision()
		{
			status = context.Init();
			checkStatus("Initializing context");

			status = depthGenerator.Create(context);
			checkStatus("Creating depth generator");

			status = rgbGenerator.Create(context);
			checkStatus("Creating RGB generator");

			status = context.StartGeneratingAll();
			checkStatus("Starting generation");

			depthImageSize = getImageSize(depthGenerator);
			rgbImageSize = getImageSize(rgbGenerator);
		}


		cv::Mat OpenNiVision::getDepth()
		{
			status = context.WaitOneUpdateAll(depthGenerator);
			checkStatus("Updating depth data");

			const XnDepthPixel* depthPixels = depthGenerator.GetDepthMap();
			cv::Mat d(depthImageSize, CV_16UC1, (void*) depthPixels);

			cv::Mat depth;
			d.copyTo(depth);

			// TODO maybe use float-matrix since mm resolution is desirable
//			depth /= 10; // mm -> cm

			d.release();

			return depth;
		}

		cv::Mat OpenNiVision::getRgb()
		{
			status = context.WaitOneUpdateAll(rgbGenerator);
			checkStatus("Updating RGB data");

			const XnRGB24Pixel* rgbPixels = rgbGenerator.GetRGB24ImageMap();
			cv::Mat bgr(rgbImageSize, CV_8UC3, (void*) rgbPixels);

			cv::Mat rgb;
			cvtColor(bgr, rgb, CV_RGB2BGR);

			bgr.release();

			return rgb;
		}


		void OpenNiVision::checkStatus(std::string action) const
		{
			if (status != XN_STATUS_OK) {
				std::string cause = xnGetStatusString(status);
				std::cout << action << " failed: " << cause << std::endl;
				exit(-1);
			}
		}

		cv::Size OpenNiVision::getImageSize(xn::MapGenerator& generator) const
		{
			XnMapOutputMode output;
			generator.GetMapOutputMode(output);
			return cv::Size(output.nXRes, output.nYRes);
		}

    }
}

