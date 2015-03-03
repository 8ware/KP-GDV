#include <string>
#include <vector>

#include <rapidjson/document.h>

#include <opencv/cv.h>
//#include <opencv2/core/core.hpp>

namespace kinjo {
namespace util {

class Config final {

public:
	Config(std::string const & json);
	
	/*
	returns value of a specific attribute
	*/		
	float getFloat(std::string const & section, std::string const & attribute);
	int getInt(std::string const & section, std::string const & attribute);
	std::string getString(std::string const & section, std::string const & attribute);
	bool getBool(std::string const & section, std::string const & attribute);

	static cv::Matx44f readMatrixFromFile(std::string const & fileName);
	static bool writeMatrixToFile(std::string const & fileName, cv::Matx44f &matrix);

private:
	rapidjson::Document doc;
	rapidjson::Value::ConstValueIterator getValue(std::string const & section, std::string const & attribute);
};

} //end of namespace util
} //end of namespace kinjo
