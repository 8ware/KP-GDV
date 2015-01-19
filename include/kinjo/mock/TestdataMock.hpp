#pragma once

#include <kinjo/arm/Arm.hpp>
#include <kinjo/vision/Vision.hpp>
#include <kinjo/calibration/CalibrationPointGenerator.hpp>
#include <kinjo/mock/DataProvider.hpp>

#include <memory>


namespace kinjo {
namespace mock {

/**
 * This class provides mock implementations for Arm, CalibrationPointGenerator
 * and Vision. When the state of the Arm is changed, e.g. by calling moveTo(),
 * the state of the Vision is adjusted, such that the position corresponds to
 * the appropriate depth and RGB images. Available positions can requested
 * through the CalibrationPointGenerator interface.
 */
class TestdataMock :
	public arm::Arm,
	public vision::Vision,
	public calibration::CalibrationPointGenerator {

public:

	/**
	 * Creates a new instance of the mock implementation which uses the given
	 * DataProvider to deliver the sample data.
	 *
	 * \param provider an implementation of a data provider which delivers the
	 *        sample data
	 */
	TestdataMock(std::shared_ptr<DataProvider> provider);

	virtual void moveTo(cv::Vec3f vector);
	virtual cv::Vec3f getPosition() const;
	virtual cv::Vec3f getRotation() const;

	// The following methods are not implemented
	virtual void moveToStartPosition(bool hasFingersClosed);
	virtual void rotateTo(cv::Vec3f vector);
	virtual void moveBy(cv::Vec3f vector);
	virtual void rotateBy(cv::Vec3f vector);
	virtual void rotateHandBy(float MultiplesOfPI);
	virtual void openFingers();
	virtual void closeFingers();
	// ---

	virtual void updateImages(bool bRequireUpdates);
	virtual cv::Mat const & getDepth() const;
	virtual cv::Mat const & getRgb() const;
	virtual std::uint16_t getMaxDepthValue() const;
	virtual cv::Vec3f getPositionFromImagePointPx(cv::Point const & v2iPointPx) const;

	virtual cv::Vec3f getNextCalibrationPoint() const;

private:

	/**
	 * The data provider used to deliver sample data.
	 */
	std::shared_ptr<DataProvider> provider;

	/**
	 * The list of available positions.
	 */
	std::vector<std::pair<cv::Vec3f, cv::Vec3f>> positions;
	/**
	 * The count of available positions.
	 */
	int availPosCount;
	/**
	 * The index which denotes the current position.
	 */
	mutable int posIndex;
	/**
	 * The current position pair where the first position denotes the expected
	 * while the second position denotes the actual reached sample position.
	 */
	std::pair<cv::Vec3f, cv::Vec3f> currPosition;

	/**
	 * The index which denotes the current pair of depth and RGB images.
	 */
	int imgIndex;
	/**
	 * The current list of depth images corresponding to the current active
	 * position.
	 */
	std::vector<cv::Mat> currDepthImages;
	/**
	 * The current list of RGB images corresponding to the current active
	 * position.
	 */
	std::vector<cv::Mat> currRgbImages;

	/**
	 * Checks whether the mock implementation was initialized, in fact, the
	 * moveTo() method must be called before any position related method is
	 * invoked, e.g. updateImages(). A runtime_error will be thrown if not.
	 */
	void checkInitialized() const;
};

} // end namespace mock
} // end namespace kinjo

