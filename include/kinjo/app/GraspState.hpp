#pragma once

#include <queue>
#include <mutex>

#include <opencv2/imgproc/imgproc.hpp>

#include <kinjo/app/State.hpp>
#include <kinjo/vision/Vision.hpp>
#include <kinjo/calibration/Calibrator.hpp>
#include <kinjo/grasp/Grasper.hpp>


namespace kinjo {
namespace app {

/**
 * This class represents the grasp state of the application.
 */
class GraspState : public State {

public:

	/**
	 * Constructs the grasp state with the ready state as only transition as
	 * well as the given arm, vision and calibrator. The three offset values
	 * are used to correct the grasp target position due to inaccuracies of
	 * the calibration.
	 *
	 * \param readyState the pointer to the ready state pointer
	 * \param arm the arm to be used
	 * \param vision the vision to be used
	 * \param calibrator the calibrator to be used
	 * \param xOffset offset in x-direction
	 * \param yOffset offset in y-direction
	 * \param zOffset offset in z-direction
	 */
	GraspState(app::State** readyState,
			grasp::Grasper* grasper, vision::Vision* vision, calibration::Calibrator* calibrator,
			int* xOffset, int* yOffset, int* zOffset);

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

	std::mutex* mutex;

	/**
	 * The pointer to the ready state pointer.
	 */
	app::State** readyState;
	
	/**
	 * The arm to be used.
	 */
	grasp::Grasper* grasper;
	/**
	 * The vision to be used.
	 */
	vision::Vision* vision;
	/**
	 * The calibrator to be used.
	 */
	calibration::Calibrator* calibrator;

	/**
	 * The pointer to the offset in x-direction.
	 */
	int* xOffset;
	/**
	 * The pointer to the offset in y-direction.
	 */
	int* yOffset;
	/**
	 * The pointer to the offset in z-direction.
	 */
	int* zOffset;

	/**
	 * The queue of target positions.
	 */
	std::queue<cv::Vec3f> pickqueue;	
	/**
	* The target positions to drop items
	*/
	cv::Vec3f dropposition;
	/**
	 * The target position which is currently processed.
	 */
	cv::Vec3f currentTargetPosition;
	/**
	 * The boolean flag which indicates whether a thread is already active.
	 */
	bool activeThread = false;

	/**
	 * Grasps an item asynchronously.
	 */
	void graspAsync();
	/**
	 * Grasps an item.
	 */
	void grasp();

};

} // end namespace app
} // end namespace kinjo

