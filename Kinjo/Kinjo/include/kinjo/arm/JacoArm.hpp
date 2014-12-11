#pragma once

#include <kinjo/arm/Arm.hpp>

#include <memory>

namespace kinjo {
    namespace arm {

        class JacoArm final : public Arm
        {

        public:
            JacoArm();

            /**
            * \param vector the absolute position in centimetres.
            */
            void moveTo(cv::Vec3f vector) override;

            /**
            * \param vector the absolute grab rotation in Degree
            */
            void rotateTo(cv::Vec3f vector) override;

            /**
            * \param vector the relative position in centimetres.
            */
            void moveBy(cv::Vec3f vector) override;

            /**
            * \param vector the relative Rotation in Degree.
            */
            void rotateBy(cv::Vec3f vector) override;

            /**
             * \return the absolute position in centimetres.
             */
            cv::Vec3f getPosition() const override;
            /**
            * \return the absolute rotation in degree
            **/
            cv::Vec3f getRotation() const override;
            void openFingers() override;
            void closeFingers() override;

        private:
            std::shared_ptr<KinDrv::JacoArm> TheJacoArm;
            bool isArmMoving();
        };//class
    
    }
} //namespace

