#include <string>
#include <vector>

#include <rapidjson/document.h>

#include <opencv/cv.h>

namespace kinjo {
namespace util {

class Config final {

public:
	Config(std::string const & json);
	
	/**
	* \return value of a specific attribute
	*/		
	float getFloat(std::string const & section, std::string const & attribute);
	int getInt(std::string const & section, std::string const & attribute);
	std::string getString(std::string const & section, std::string const & attribute);
	bool getBool(std::string const & section, std::string const & attribute);

	/**
	* \return the values of the given file in matrix format
	**/
	static cv::Matx44f readMatrixFromFile(std::string const & fileName);
	
	/**
	* writes given matrix to the given file
	* \return true if succeeded
	**/
	static bool writeMatrixToFile(std::string const & fileName, cv::Matx44f &matrix);

private:
	/**
	* representation of json config file
	**/
	rapidjson::Document doc;

	/**
	* \return iterator pointing to specific value
	**/
	rapidjson::Value::ConstValueIterator getValue(std::string const & section, std::string const & attribute);
};

} //end of namespace util
} //end of namespace kinjo
