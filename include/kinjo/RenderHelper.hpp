#pragma once


#include <opencv2/highgui/highgui.hpp>

namespace kinjo {

	/**
	* Renders a double circle.
	* This is used to indicate the mouse click position.
	**/
	void renderDoubleCircle(cv::Mat& image, cv::Point const & point, cv::Scalar const & color);

	/**
	* Render the text at the center.
	**/
	void renderTextCenter(cv::Mat & image, cv::Scalar const & color, std::string const & sText, double fFontScale, int iThickness);

	/**
	* Render the given 3d-vector at the given point.
	**/
	void renderPosition(cv::Mat& image, cv::Point const & point, cv::Scalar const & color, cv::Vec3f const & v3fVisionPosition, std::string const & what = "");

	/**
	 * Renders a raster of crosshairs into the given image using the specified
	 * color.
	 *
	 * \param image the image into which the raster is rendered
	 * \param color the color of the raster to be used
	 */
	void renderRaster(cv::Mat& image, cv::Scalar const & color);

	/**
	 * Renders the given information (lines) at the lower left corner of the
	 * given image.
	 *
	 * \param image the image into which the information are rendered
	 * \param lines the lines of information to be rendered
	 */
	void renderInfos(cv::Mat& image, std::vector<std::string> lines);

}
