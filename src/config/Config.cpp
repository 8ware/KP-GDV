#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <kinjo/config/Config.hpp>
#include <rapidjson/rapidjson.h>

Config::Config(const std::string& filename)
	{
		std::stringstream ss;
		std::ifstream ifs;
		ifs.open(filename.c_str(), std::ios::binary);
		ss << ifs.rdbuf();
		ifs.close();

		if (doc.Parse<0>(ss.str().c_str()).HasParseError())
			throw std::invalid_argument("json parse error");
	}

float Config::getAttribute(std::string section, std::string attribute){
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