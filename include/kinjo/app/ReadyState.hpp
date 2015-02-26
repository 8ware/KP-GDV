#pragma once

#include <opencv2/imgproc/imgproc.hpp>

#include <kinjo/app/State.hpp>
#include <kinjo/calibration/Calibrator.hpp>


namespace kinjo {
namespace app {

/**
 * This class represents the ready state of the application.
 */
class ReadyState : public State {

public:

	/**
	 * Constructs the ready state with the calibration and grasp state as
	 * transitions as well as the given calibrator and matrix filename.
	 *
	 * \param calibState the pointer to the calibration state pointer
	 * \param graspState the pointer to the grasp state pointer
	 * \param calibrator the calibrator to be used
	 * \param matrixFilename the filename to store the calibration matrix in
	 */
	ReadyState(app::State** calibState, app::State** graspState,
			calibration::Calibrator* calibrator, std::string& matrixFilename);

	/**
	 * Actually does nothing.
	 */
	void initialize() override {}
	/**
	 * Actually does nothing.
	 */
	void process(cv::Mat& rgbImage, cv::Mat& depthImage) override {}
	void process(int mouseEvent, cv::Point point) override;
	void process(int key) override;

private:

	/**
	 * The pointer to the calibration state pointer.
	 */
	app::State** calibState;
	/**
	 * The pointer to the grasp state pointer.
	 */
	app::State** graspState;

	/**
	 * The calibrator to be used.
	 */
	calibration::Calibrator* calibrator;
	/**
	 * The name of the file to store the calibration matrix in.
	 */
	std::string matrixFilename;
	
};

} // end namespace app
} // end namespace kinjo

