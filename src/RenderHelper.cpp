
#include <kinjo/RenderHelper.hpp>


namespace kinjo{

/**
* Renders a double circle.
* This is used to indicate the mouse click position.
**/
void renderDoubleCircle(cv::Mat& image, cv::Point const & point, cv::Scalar const & color)
{
	cv::circle(image, point, 3, color);
	cv::circle(image, point, 7, color);
}

/**
* Render the text at the center.
**/
void renderTextCenter(
	cv::Mat & image, 
	cv::Scalar const & color, 
	std::string const & sText, 
	double fFontScale,
	int iThickness)
{
	int const fontFace(
		cv::FONT_HERSHEY_SIMPLEX);

	int iBaseline(0);
	cv::Size textSize(
		cv::getTextSize(
			sText,
			fontFace,
			fFontScale, iThickness,
			&iBaseline));
	iBaseline += iThickness;

	cv::Point const textOrg(
		(image.cols - textSize.width)/2,
		(image.rows + textSize.height)/2);

	cv::putText(
		image,
		sText, textOrg,
		fontFace, fFontScale,
		color, iThickness);
}

/**
* Render the given 3d-vector at the given point.
**/
void renderPosition(cv::Mat& image, cv::Point const & point, cv::Scalar const & color, cv::Vec3f const & v3fVisionPosition)
{
	std::stringstream stream;
	stream << "[" << v3fVisionPosition[0u] << ", " << v3fVisionPosition[1u] << ", " << v3fVisionPosition[2u] << "] cm";
	cv::Point const shifted = point + cv::Point(15, -5);
	cv::putText(image, stream.str(), shifted, cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1, CV_AA);
}
}