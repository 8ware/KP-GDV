#pragma once

#include <opencv2/imgproc/imgproc.hpp>

#include <kinjo/app/State.hpp>
#include <kinjo/arm/Arm.hpp>
#include <kinjo/calibration/Calibrator.hpp>


namespace kinjo {
namespace app {

/**
 * This class represents the calibration state of the application.
 */
class CalibrationState : public State {

public:

	/**
	 * Constructs the calibration state with the initial and ready state as
	 * transitions as well as the given arm and calibrator.
	 *
	 * \param initState the pointer to the initial state pointer
	 * \param readyState the pointer to the ready state pointer
	 * \param arm the arm to be used
	 * \param calibrator the calibrator to be used
	 */
	CalibrationState(app::State** initState, app::State** readyState,
			arm::Arm* arm, calibration::Calibrator* calibrator);

	void initialize() override;
	void process(cv::Mat& rgbImage, cv::Mat& depthImage) override;
	/**
	 * Actually does nothing.
	 */
	void process(int mouseEvent, cv::Point point) override {}
	void process(int key) override;

protected:
	
	/**
	 * The arm to be used.
	 */
	arm::Arm* arm;
	/**
	 * The calibrator to be used.
	 */
	calibration::Calibrator* calibrator;

private:

	/**
	 * The pointer to the initial state pointer.
	 */
	app::State** initState;
	/**
	 * The pointer to the ready state pointer.
	 */
	app::State** readyState;

	/**
	 * Initializes the calibration state asynchronous.
	 */
	void initializeAsync();

};

} // end namespace app
} // end namespace kinjo

