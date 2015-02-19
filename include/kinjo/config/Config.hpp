#include <string>
#include <vector>

#include <rapidjson/document.h>

namespace kinjo {
namespace config {

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

	static bool readMatrixFromFile(std::string const & fileName, std::vector<std::vector< float>> &matrix);
	static bool writeMatrixToFile(std::string const & fileName, std::vector<std::vector< float>> &matrix);

private:
	rapidjson::Document doc;
	rapidjson::Value::ConstValueIterator getValue(std::string const & section, std::string const & attribute);
};

} //end of namespace config
} //end of namespace kinjo