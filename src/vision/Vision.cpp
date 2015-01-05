#include <kinjo/vision/Vision.hpp>

namespace kinjo
{
	namespace vision
	{
		/**
		 * 
		 **/
		cv::Vec3f Vision::estimatePositionFromImagePointPx(
			cv::Point const & v2iPointPx)
		{
			cv::Vec3f v3fVisionPosition(0.0f, 0.0f, 0.0f);
			std::size_t uiValidPositions(0);

			for(int y(-1); y<2; ++y)
			{
				for(int x(-1); x<2; ++x)
				{
					cv::Point const v2iOffsetPointPx(v2iPointPx.x + x, v2iPointPx.y + y);

					// The point has to be within the image.
					if(v2iOffsetPointPx.x>=0 && v2iOffsetPointPx.x<getDepth().cols
						&& v2iOffsetPointPx.y>=0 && v2iOffsetPointPx.y<getDepth().rows)
					{
						cv::Vec3f const p(getPositionFromImagePointPx(v2iOffsetPointPx));

						// If the position is valid, add it up
						if(p[2u] > 0.0f)
						{
							v3fVisionPosition += p;
							++uiValidPositions;
						}
					}
				}
			}

			return (v3fVisionPosition / static_cast<float>(uiValidPositions));
		}
	}
}