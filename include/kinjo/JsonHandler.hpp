#include <string>
#include <vector>

#include <rapidjson/document.h>

	class JsonHandler final
	{

	private:
		rapidjson::Document doc_;

	public:
		JsonHandler(const std::string& json);
	
		/*
		returns value of a specific attribute
		*/
		float getAttribute(std::string& const section, std::string& const attribute);
	
		};