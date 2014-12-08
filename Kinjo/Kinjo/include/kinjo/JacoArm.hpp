#pragma once

#include <kinjo/Arm.hpp>

#include "libkindrv/kindrv.h" // KinDrv::JacoArm

namespace kinjo {

	class JacoArm : public Arm {


	public:
		JacoArm()
		{
		TheJacoArm = new KinDrv::JacoArm();
		}

		~JacoArm(){
			delete TheJacoArm;
		}
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
	private:
		KinDrv::JacoArm* TheJacoArm;
	};//class
} //namespace

