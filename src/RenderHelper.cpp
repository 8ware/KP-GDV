
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
	stream << "[" << v3fVisionPosition[0u] << ", " << v3fVisionPosition[1u] << ", " << v3fVisionPosition[2u] << "] mm";
	cv::Point const shifted = point + cv::Point(15, -5);
	cv::putText(image, stream.str(), shifted, cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1, CV_AA);
}

void renderRaster(cv::Mat& image, cv::Scalar const & color)
{
	int length = 30;
	cv::Point offset_x(length, 0);
	cv::Point offset_y(0, length);

	int cols = image.cols;
	int rows = image.rows;

	std::vector<cv::Point> points;
	points.push_back(cv::Point(cols * 1/6, rows * 1/6)); // top-left
	points.push_back(cv::Point(cols * 1/2, rows * 1/6)); // top-mid
	points.push_back(cv::Point(cols * 5/6, rows * 1/6)); // top-right
	points.push_back(cv::Point(cols * 1/6, rows * 1/2)); // mid-left
	points.push_back(cv::Point(cols * 1/2, rows * 1/2)); // center
	points.push_back(cv::Point(cols * 5/6, rows * 1/2)); // mid-right
	points.push_back(cv::Point(cols * 1/6, rows * 5/6)); // bot-left
	points.push_back(cv::Point(cols * 1/2, rows * 5/6)); // bot-mid
	points.push_back(cv::Point(cols * 5/6, rows * 5/6)); // bot-right

	for (auto point : points) {
		cv::line(image, point-offset_x, point+offset_x, color);
		cv::line(image, point-offset_y, point+offset_y, color);
	}
}

}
