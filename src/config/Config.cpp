#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>
#include <vector>

#include "kinjo/config/Config.hpp"

#include "rapidjson/prettywriter.h"	// for stringify JSON

#include <opencv2/core/affine.hpp>		// cv::Matx44f * cv::Vec3f

namespace kinjo {
namespace config {

Config::Config(std::string const & filename) {
		std::stringstream ss;
		std::ifstream ifs;
		ifs.open(filename.c_str(), std::ios::binary);
		if (!ifs)
			throw std::invalid_argument("Config File could not be opened!");
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
	return NULL;
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

bool Config::readMatrixFromFile(std::string const & fileName, cv::Matx44f &matrix){
	//read File
	std::stringstream ss;
	std::ifstream ifs;
	ifs.open(fileName.c_str(), std::ios::binary);
	ss << ifs.rdbuf();
	ifs.close();

	//Parse Array
	rapidjson::Document d;
	if (d.Parse<0>(ss.str().c_str()).HasParseError())
		throw std::invalid_argument("json parse error");

	//Read values in result vector
	if (d.IsArray()){
		matrix.zeros();
		size_t rowNr = 0;
		for (rapidjson::Value::ConstValueIterator irow = d.Begin(); irow != d.End(); irow++){
			if (irow->IsArray()){
				size_t colNr = 0;
				for (rapidjson::Value::ConstValueIterator icol = irow->Begin(); icol != irow->End(); icol++){
					matrix(rowNr, colNr) = (static_cast<float> (icol->GetDouble()));
					colNr++;
				}
			}
			else return false;
			rowNr++;
		}
		return true;
	}
	else return false;
}

bool Config::writeMatrixToFile(std::string const & fileName, cv::Matx44f &matrix){
	//write matrix to document
	rapidjson::Document d;
	d.SetArray();
	rapidjson::Document::AllocatorType& allocator = d.GetAllocator();
	for (int row = 0; row < 4; row++){
		rapidjson::Value cols(rapidjson::kArrayType);
		for (int col = 0; col < 4; col++){
			cols.PushBack(matrix.row(row).col(col).val[0], allocator);
		}
		d.PushBack(cols, allocator);
	}

	//Write json to stringbuffer
	rapidjson::StringBuffer sb;
	rapidjson::Writer<rapidjson::StringBuffer> writer(sb);
	d.Accept(writer);
	//write to file
	std::ofstream ofs;
	ofs.open(fileName,std::ios::binary);
	if (!ofs) return false;
	ofs.write((const char*)sb.GetString(),sb.GetSize());
	ofs.close();
	return true;
}

} //end of namespace config
} //end of namespace kinjo
