#include <string>

#include <rapidjson/document.h>

	class Config final
	{

	private:
		rapidjson::Document doc;

	public:
		Config(const std::string& json);
	
		/*
		returns value of a specific attribute
		*/
		float getAttribute(std::string const section, std::string const attribute);
	
		};