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

			if (Product->initialized)
				return Product;
			// if not go on, if it does use that arm

			//add more Arms here
			return Product;
		} //getInstance
	}//namespace arm
} //namespace kinjo