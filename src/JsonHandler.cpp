#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <stdexcept>

#include <rapidjson/rapidjson.h>

#include <kinjo/JsonHandler.hpp>

JsonHandler::JsonHandler(std::string const & filename)
	{
		std::stringstream ss;
		std::ifstream ifs;
		ifs.open(filename.c_str(), std::ios::binary);
		ss << ifs.rdbuf();
		ifs.close();

		if (doc_.Parse<0>(ss.str().c_str()).HasParseError())
			throw std::invalid_argument("json parse error");
	}

float JsonHandler::getAttribute(std::string const & section, std::string const & attribute){
	std::cout << "section: " << section <<", attribute: " << attribute << "\n";
	std::string tempA = "";
	std::string tempB = "";
	double ret = 0;

	//iterate over sections
	for (rapidjson::Document::ConstValueIterator sec_itr = this->doc_.Begin(); sec_itr != doc_.End(); ++sec_itr){
		
		//switch between titles and properties
		for (rapidjson::Value::ConstMemberIterator i = sec_itr->MemberBegin() ; i != sec_itr->MemberEnd();i++){
			
			tempA = i->name.GetString();
			if (tempA.compare("title") == 0){

				tempB = i->value.GetString();
				//if title is the section we look for
				if (tempB.compare(section) == 0){
					std::cout << "name: " << i->name.GetString() << "\n";
					std::cout << "value:" << i->value.GetString() << "\n";
				
					//look for matching property
					i++;
					tempA = i->name.GetString();
					if (tempA.compare("properties") == 0){
						//iterate over attributes
						for (rapidjson::Value::ConstMemberIterator attr_iter = i->value.MemberBegin(); attr_iter != i->value.MemberEnd(); attr_iter++)
						{

							//compare attribute name with given attribute name
							tempA = attr_iter->name.GetString();
							if (tempA.compare(attribute) == 0){
								if (!attr_iter->value.IsArray()){
									ret =  attr_iter->value.GetDouble();
									return static_cast<float>(ret);
								}
							}
						}
					}
				}
			}			
		}
	}
	
	return static_cast<float>(ret);
}
