#include <string>

#include <rapidjson/document.h>

	class Config final
	{

	private:
		rapidjson::Document doc;
		rapidjson::Value::ConstValueIterator Config::getValue(std::string const & section, std::string const & attribute);


	public:
		Config(std::string const & json);
	
		/*
		returns value of a specific attribute
		*/
		
		float getFloat(std::string const & section, std::string const & attribute);
		int Config::getInt(std::string const & section, std::string const & attribute);
		std::string Config::getString(std::string const & section, std::string const & attribute);

		};