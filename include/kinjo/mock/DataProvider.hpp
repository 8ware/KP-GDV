#pragma once

#include <string>
#include <vector>
#include <utility>

#include <opencv2/imgproc/imgproc.hpp>


namespace kinjo {
namespace mock {

/**
 * This interface defines the methods required to provide the sampled test data
 * to the arm's and vision's mock implementation. Test data is given as a set
 * of sampled positions with their associated depth and RGB images. A position
 * is sampled with at least one depth and RGB image, respectively.
 */
class DataProvider {

public:

	/**
	 * Delivers a list of points which are associated to depth and RGB images.
	 * Each element consists actually of a pair of points where the first point
	 * denotes the requested position while the second one is the real position
	 * after moving the arm.
	 *
	 * \return the list of coordinate pairs available for the set of test data.
	 */
	virtual std::vector<std::pair<cv::Vec3f, cv::Vec3f>> getPositions() const = 0;

	/**
	 * Delivers the depth image(s) associated to the given position. At least
	 * one image is provided unless there are no images sampled for the given
	 * position.
	 *
	 * \param position the position which is associated with at least one depth
	 *        image
	 *
	 * \return a vector containing all depth image data associated to the
	 *         position.
	 */
	virtual std::vector<cv::Mat> getDepthImages(cv::Vec3f position) const = 0;

	/**
	 * Delivers the RGB image(s) associated to the given position. At least
	 * one image is provided unless there are no images sampled for the given
	 * position.
	 *
	 * \param position the position which is associated with at least one RGB
	 *        image
	 *
	 * \return a vector containing all RGB image data associated to the
	 *         position.
	 */
	virtual std::vector<cv::Mat> getRgbImages(cv::Vec3f position) const = 0;

};

} // end namespace mock
} // end namespace kinjo

