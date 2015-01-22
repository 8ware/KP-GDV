#ifndef _MSC_VER
#include <kinjo/mock/DirectoryBasedDataProvider.hpp>

#include <iostream>
#include <algorithm>
#include <dirent.h>

#include <opencv2/highgui/highgui.hpp>


namespace kinjo {
namespace mock {

DirectoryBasedDataProvider::DirectoryBasedDataProvider(std::string directory) {
	this->directory = directory;

	loadPositions();
}

std::vector<std::pair<cv::Vec3f, cv::Vec3f>> DirectoryBasedDataProvider::getPositions() const {
	return this->positions;
}

std::vector<cv::Mat> DirectoryBasedDataProvider::getDepthImages(cv::Vec3f position) const {
	return loadImages("depth", position, CV_LOAD_IMAGE_ANYDEPTH);
}

std::vector<cv::Mat> DirectoryBasedDataProvider::getRgbImages(cv::Vec3f position) const {
	return loadImages("image", position, CV_LOAD_IMAGE_COLOR);
}

void DirectoryBasedDataProvider::loadPositions() {
	DIR *dh = opendir(directory.c_str());
	if (!dh)
		throw std::runtime_error("Cannot open directory!");

	struct dirent* entry;
	while ((entry = readdir(dh)) != NULL) {
		std::string name = entry->d_name;

		if (entry->d_type != DT_DIR)
			continue;

		if (name == "." || name == "..")
			continue;

		std::size_t delim = name.find("_");
		std::string xyz_ = name.substr(0, delim);
		std::string _xyz = name.substr(delim+1);

		std::size_t s1_ = xyz_.find(",");
		std::size_t s2_ = xyz_.rfind(",");
		float x_ = std::stof(xyz_.substr(0, s1_) + ".0");
		float y_ = std::stof(xyz_.substr(s1_+1, s2_-s1_-1) + ".0");
		float z_ = std::stof(xyz_.substr(s2_+1) + ".0");

		std::size_t _s1 = _xyz.find(",");
		std::size_t _s2 = _xyz.rfind(",");
		float _x = std::stof(_xyz.substr(0, _s1));
		float _y = std::stof(_xyz.substr(_s1+1, _s2-_s1-1));
		float _z = std::stof(_xyz.substr(_s2+1));

		cv::Vec3f expected(x_, y_, z_);
		cv::Vec3f actual(_x, _y, _z);
		std::pair<cv::Vec3f, cv::Vec3f> pair(expected, actual);

		this->positions.push_back(pair);
	}

	if (closedir(dh) == -1)
		throw std::runtime_error("Cannot close directory!");
}

std::vector<cv::Mat> DirectoryBasedDataProvider::loadImages(std::string type,
		cv::Vec3f position, int flags) const {
	cv::Vec3f actual = checkPosition(position);

	std::stringstream sstream;
	sstream << position[0] << ',' << position[1] << "," << position[2] << "_"
			<< actual[0]   << ',' << actual[1]   << "," << actual[2];
	std::string directory = this->directory + "/" + sstream.str();

	DIR *dh = opendir(directory.c_str());
	if (!dh)
		throw std::runtime_error("Cannot open directory!");

	std::vector<std::string> files;
	struct dirent* entry;
	while ((entry = readdir(dh)) != NULL) {
		std::string name = entry->d_name;

		if (entry->d_type != DT_REG)
			continue;

		if (name == "." || name == "..")
			continue;

		if (name.find(type) == -1)
			continue;

		files.push_back(directory + "/" + name);
	}

	if (closedir(dh) == -1)
		throw std::runtime_error("Cannot close directory!");

	std::sort(files.begin(), files.end());

	std::vector<cv::Mat> images;
	for (auto file : files) {
		cv::Mat image = cv::imread(file, flags);
		images.push_back(image);
	}

	return images;
}

cv::Vec3f DirectoryBasedDataProvider::checkPosition(cv::Vec3f expected) const {
	for (auto position : this->positions) {
		if (position.first == expected)
			return position.second;
	}

	throw std::runtime_error("Invalid position!");
}

} // end namespace mock
} // end namespace kinjo
#endif
