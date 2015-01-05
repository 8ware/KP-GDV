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

			// Add search for more Arms here if the Jaco arm is not found!

			if(!Product)
			{
				throw std::runtime_error("No arm found!");
			}

			return Product;
		} //getInstance
	}//namespace arm
} //namespace kinjo