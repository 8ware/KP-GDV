#pragma once
#ifndef _MSC_VER
#include <kinjo/mock/DataProvider.hpp>


namespace kinjo {
namespace mock {

/**
 * This implementation of a DataProvider uses a certain directory structure to
 * load and provide the requested data. In fact the directory structure must
 * fit into the following pattern:
 *
 *     <directory>─┬─<x1>,<y1>,<z1>_<x1'>,<y1'>,<z1'>─┬─001_depth.png
 *                 │                                  ├─001_image.png
 *                 │                                  └─...
 *                 └─...
 *
 * Thereby the first tuple of x-y-z coordinates denote the requested position
 * while the second tuple represents the real position after the arm reached
 * its final state. Both tuples are delimited through an underscore and can
 * contain signed floating point numbers separated by commas. The directory
 * which they represent contains the actual depth and RGB image <em>pairs</em>
 * which are sequentially numbered.
 */
class DirectoryBasedDataProvider : public DataProvider {

public:

	/**
	 * Creates a new data provider using the given directory as data base.
	 */
	DirectoryBasedDataProvider(std::string directory);

	virtual std::vector<std::pair<cv::Vec3f, cv::Vec3f>> getPositions() const;
	virtual std::vector<cv::Mat> getDepthImages(cv::Vec3f position) const;
	virtual std::vector<cv::Mat> getRgbImages(cv::Vec3f position) const;

private:

	/**
	 * The directory used as data base.
	 */
	std::string directory;

	/**
	 * The list of position pairs provided by the data set.
	 */
	std::vector<std::pair<cv::Vec3f, cv::Vec3f>> positions;

	/**
	 * Loads all available positions from the base directory, in fact
	 * initializes the #positions vector.
	 */
	virtual void loadPositions();

	/**
	 * Loads all images of a certain type for the specific position.
	 *
	 * \param type the required image type
	 * \param position the position where the image was sampled
	 * \param flags the flags which indicate the loaded image data type
	 *
	 * \return the list of loaded images.
	 */
	virtual std::vector<cv::Mat> loadImages(std::string type,
			cv::Vec3f position, int flags) const;

	/**
	 * Checks whether the given position is available. If true, delivers the
	 * counter part which is the actual position of the sampled arm.
	 *
	 * \param position the position to be validated
	 *
	 * \return the arm's actual position when it was sampled.
	 */
	virtual cv::Vec3f checkPosition(cv::Vec3f position) const;

};

} // end namespace mock
} // end namespace kinjo
#endif
