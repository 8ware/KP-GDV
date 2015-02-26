#pragma once

#include <memory>

#include <opencv2/imgproc/imgproc.hpp>

#include <kinjo/app/State.hpp>


namespace kinjo {
namespace app {

/**
 * This class represents the initial state of the application.
 */
class InitialState : public State {

public:

	/**
	 * Constructs the initial state with the given calibration state as only
	 * transition.
	 *
	 * \param calibState the pointer to the calibration state pointer
	 */
	InitialState(app::State** calibState);

	/**
	 * Actually does nothing.
	 */
	void initialize() override {}
	/**
	 * Actually does nothing.
	 */
	void process(cv::Mat& rgbImage, cv::Mat& depthImage) override {}
	/**
	 * Actually does nothing.
	 */
	void process(int mouseEvent, cv::Point point) override {}
	void process(int key) override;

private:

	/**
	 * The pointer to the calibration state pointer.
	 */
	app::State** calibState;
	
};

} // end namespace app
} // end namespace kinjo

