#pragma once

#include <kinjo/vision/Vision.hpp>

namespace kinjo {
    namespace vision {

        class OpenNiVision : public Vision
        {

        public:
            /**
             * \return the depth values normalized to centimetres.
             */
            virtual cv::Mat getDepth() const override;
            virtual cv::Mat getRgb() const override;

        };

    }
}

