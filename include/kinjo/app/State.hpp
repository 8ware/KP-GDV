#pragma once

#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>


namespace kinjo {
namespace app {

/**
 * This interface defines the representation of an application state.
 */
class State {

public:

	/**
	 * Initializes the state.
	 */
	virtual void initialize() = 0;

	/**
	 * Processes images for display.
	 *
	 * \param rgbImage the RGB image matrix
	 * \param depthImage the depth image matrix
	 */
	virtual void process(cv::Mat& rgbImage, cv::Mat& depthImage) = 0;
	/**
	 * Processes mouse events. This can be used to trigger actions or change
	 * the state.
	 *
	 * \param mouseEvent the event triggered by the mouse
	 * \param point the x-y coordinate of the mouse at the moment of the event
	 */
	virtual void process(int mouseEvent, cv::Point point) = 0;
	/**
	 * Processes key events. This can be used to trigger actions or change the
	 * state.
	 *
	 * \param key the key which was pressed
	 */
	virtual void process(int key) = 0;

	/**
	 * \return the state's designator, for example its name.
	 */
	std::string getDesignator() const {
		return this->designator;
	}

	/**
	 * \return the state's details, for example the current target position.
	 */
	std::string getDetails() const {
		return this->details;
	}

	/**
	 * \return the state's information, for example possible transitions.
	 */
	std::vector<std::string> getInfos() const {
		return this->infos;
	}

	/**
	 * Returns the next state and resets the next state to itself.
	 *
	 * \return the next state (mostly itself).
	 */
	State* getNext() {
		State* next = this->next;
		this->next = this;
		return next;
	}

protected:

	/**
	 * The state's designator, for example its name but can be also its current
	 * active sub-state.
	 */
	std::string designator;

	/**
	 * The state's details, for example the current target position.
	 */
	std::string details = "";

	/**
	 * The state's information, for example possible transitions.
	 */
	std::vector<std::string> infos;

	/**
	 * The next state (mostly set to itself).
	 */
	State* next;

};

} // end namespace app
} // end namespace kinjo

