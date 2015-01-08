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
	void renderPosition(cv::Mat& image, cv::Point const & point, cv::Scalar const & color, cv::Vec3f const & v3fVisionPosition);
}