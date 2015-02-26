#pragma once
#include <kinjo/arm/ArmFactory.hpp>
#include <kinjo/arm/JacoArm.hpp>
#include <libkindrv/kindrv.h>       // KinDrv::JacoArm

#include <kinjo/arm/MovementGuardOne.hpp>

#include <easylogging++.h>

#include <memory>                   // std::shared_ptr
#include <iostream>

namespace kinjo {
namespace arm {
	std::shared_ptr<kinjo::arm::Arm> ArmFactory::getInstance()
	{
		std::shared_ptr<kinjo::arm::Arm> Product;
		try{
			std::list<std::shared_ptr<MovementGuard>> MovGuardList;
			std::shared_ptr<MovementGuard> MVGuardOne = std::make_shared<MovementGuardOne>();
			MovGuardList.push_front(MVGuardOne);
			Product = std::make_shared<kinjo::arm::JacoArm>(MovGuardList);
		}
		catch (std::exception const e)
		{
			LOG(FATAL) << e.what();
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