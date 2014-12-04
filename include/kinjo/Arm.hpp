#pragma once

#include <opencv2/imgproc/imgproc.hpp>


namespace kinjo {

	class Arm {

		public:
			/**
			 * \param vector the absolute position in centimeters.
			 */
			virtual void moveTo(cv::Vec3f vector) = 0;
			/**
			 * \param vector the relative position in centimeters.
			 */
			virtual void moveBy(cv::Vec3f vector) = 0;
			/**
			 * \return the absolute position in centimeters.
			 */
			virtual cv::Vec3f getPosition() const = 0;
			virtual void openFingers() = 0;
			virtual void closeFingers() = 0;

	};

}

