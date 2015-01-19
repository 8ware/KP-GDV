
#include <kinjo/mock/TestdataMock.hpp>

#include <algorithm>


namespace kinjo {
namespace mock {
	
TestdataMock::TestdataMock(DataProvider* provider) {
	this->provider = provider;

	this->positions = provider->getPositions();
	this->availPosCount = this->positions.size();

	this->posIndex = -1;
	this->imgIndex = -1;
}

void TestdataMock::moveTo(cv::Vec3f position) {
	currDepthImages = provider->getDepthImages(position);
	currRgbImages = provider->getRgbImages(position);

	for (auto expActPos : positions) {
		if (expActPos.first == position) {
			currPosition = expActPos;
			return;
		}
	}

	throw std::runtime_error("Invalid position!");
}

cv::Vec3f TestdataMock::getPosition() const {
	checkInitialized();
	return currPosition.second;
}

cv::Vec3f TestdataMock::getRotation() const {
	return cv::Vec3f(-1.0f, -1.0f, -1.0f);
}


void TestdataMock::updateImages(bool bRequireUpdates) {
	checkInitialized();
	int imgCount = std::min(currDepthImages.size(), currRgbImages.size());
	imgIndex = (imgIndex + 1) % imgCount;
}

cv::Mat const & TestdataMock::getDepth() const {
	return currDepthImages[imgIndex];
}

cv::Mat const & TestdataMock::getRgb() const {
	return currRgbImages[imgIndex];
}

std::uint16_t TestdataMock::getMaxDepthValue() const {
	return 10000;
}

cv::Vec3f TestdataMock::getPositionFromImagePointPx(cv::Point const & point) const {
	// FIXME this is NO correct implemention
	float depth = static_cast<float>(currDepthImages[imgIndex].at<ushort>(point));
	return cv::Vec3f(point.x, point.y, depth);
}


cv::Vec3f TestdataMock::getNextCalibrationPoint() const {
	posIndex = (posIndex+1) % positions.size();
	return positions[posIndex].first;
}

void TestdataMock::checkInitialized() const {
	if (posIndex == -1)
		throw std::runtime_error("Method moveTo() has to be called first!");
}


void TestdataMock::moveToStartPosition(bool hasFingersClosed) {}
void TestdataMock::rotateTo(cv::Vec3f vector) {}
void TestdataMock::moveBy(cv::Vec3f vector) {}
void TestdataMock::rotateBy(cv::Vec3f vector) {}
void TestdataMock::rotateHandBy(float MultiplesOfPI) {}
void TestdataMock::openFingers() {}
void TestdataMock::closeFingers() {}


} // end namespace mock
} // end namespace kinjo

