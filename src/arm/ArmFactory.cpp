#pragma once
#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/arm/JacoArm.hpp>
#include <libkindrv/kindrv.h>       // KinDrv::JacoArm

#include <memory>                   // std::shared_ptr
#include <iostream>

namespace kinjo {
	namespace arm {
		std::shared_ptr<kinjo::arm::Arm> ArmFactory::getInstance()
		{
			std::shared_ptr<kinjo::arm::Arm> Product;
			try{
				Product = std::make_shared<kinjo::arm::JacoArm>();
			}
			catch (std::exception const e)
			{
				std::cerr << e.what() << std::endl;
			}

			// TODO: find out, if the Product contains an actual arm.
			// if not go on, if it does use that arm
			//if (Product == nullptr) std::cout << "Trying next arm." << std::endl;

			return Product;
		} //getInstance
	}//namespace arm
} //namespace kinjo