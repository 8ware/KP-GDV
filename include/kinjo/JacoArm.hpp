#pragma once

#include <kinjo/Arm.hpp>

namespace kinjo {

	class JacoArm : public Arm {

	public:
		/**
		 * \param vector the absolute position in centimetres.
		 */
		virtual void moveTo(cv::Vec3f vector) override;
		/**
		 * \param vector the relative position in centimetres.
		 */
		virtual void moveBy(cv::Vec3f vector) override;
		/**
		 * \return the absolute position in centimetres.
		 */
		virtual cv::Vec3f getPosition() const override;
		virtual void openFingers() override;
		virtual void closeFingers() override;

	};

}

