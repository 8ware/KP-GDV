#pragma once

#include <opencv2/imgproc/imgproc.hpp>

namespace kinjo {
    namespace vision {

        class Vision
        {

        public:
            /**
             * \return the depth values normalized to centimetres.
             */
            virtual cv::Mat getDepth() = 0;
            virtual cv::Mat getRgb() = 0;

        };

    }
}

