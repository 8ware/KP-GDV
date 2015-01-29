#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <rapidjson/rapidjson.h>

#include <kinjo/config/Config.hpp>


Config::Config(std::string const & filename)
	{
		std::stringstream ss;
		std::ifstream ifs;
		ifs.open(filename.c_str(), std::ios::binary);
		ss << ifs.rdbuf();
		ifs.close();

		if (doc.Parse<0>(ss.str().c_str()).HasParseError())
			throw std::invalid_argument("json parse error");
	}


//returns -1 if attribute not found
float Config::getAttribute(std::string const & section, std::string const & attribute){
	rapidjson::Value::ConstMemberIterator itr = doc.FindMember(section.c_str());
	if (itr != doc.MemberEnd()){
		rapidjson::Value::ConstMemberIterator it = itr->value.FindMember(attribute.c_str());
		if (it != itr->value.MemberEnd()){
			std::cout << it->name.GetString();
			return static_cast<float> (it->value.GetDouble());
		}
	}

	else return -1;
}