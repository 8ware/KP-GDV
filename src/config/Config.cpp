#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <rapidjson/rapidjson.h>

#include <kinjo/config/Config.hpp>

namespace kinjo {
namespace config {

Config::Config(std::string const & filename) {
		std::stringstream ss;
		std::ifstream ifs;
		ifs.open(filename.c_str(), std::ios::binary);
		ss << ifs.rdbuf();
		ifs.close();

		if (doc.Parse<0>(ss.str().c_str()).HasParseError())
			throw std::invalid_argument("json parse error");
	}

rapidjson::Value::ConstValueIterator Config::getValue(std::string const & section, std::string const & attribute) {
	rapidjson::Value::ConstMemberIterator itr = doc.FindMember(section.c_str());
	if (itr != doc.MemberEnd()){
		rapidjson::Value::ConstMemberIterator it = itr->value.FindMember(attribute.c_str());
		if (it != itr->value.MemberEnd()){

			return &it->value;
		}
	}
	else return NULL;
}

std::string Config::getString(std::string const & section, std::string const & attribute) {
	rapidjson::Value::ConstValueIterator iterator = getValue(section, attribute);
	return iterator->GetString();
}

float Config::getFloat(std::string const & section, std::string const & attribute) {
	rapidjson::Value::ConstValueIterator iterator = getValue(section, attribute);
	return static_cast<float> (iterator->GetDouble());
}

int Config::getInt(std::string const & section, std::string const & attribute) {
	rapidjson::Value::ConstValueIterator iterator = getValue(section, attribute);
	return iterator->GetInt();
}

bool Config::getBool(std::string const & section, std::string const & attribute) {
	rapidjson::Value::ConstValueIterator iterator = getValue(section, attribute);
	return iterator->GetBool();
}

} //end of namespace config
} //end of namespace kinjo
