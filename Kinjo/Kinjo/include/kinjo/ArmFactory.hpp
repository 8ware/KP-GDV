#pragma once
#include <kinjo\Arm.hpp>
#include <kinjo\JacoArm.hpp>
#include "libkindrv/kindrv.h" // KinDrv::JacoArm

namespace kinjo{

	//In Case other Arms are provided later we use a factory to get the arm.
	//So we can simply Implement the other arm and dont change any code in the
	//main program except the ArmKey at the beginning
	class ArmFactory
	{
	public:
		ArmFactory();
		~ArmFactory();

		static kinjo::Arm* getInstance(int ArmKey)
		{
			kinjo::Arm* Product;

			switch (ArmKey)
			{
			case 1:
				Product = new kinjo::JacoArm();
				break;
				// How to add other Arm types:
				//case 2:
				//	Product = new kinjo::phantasieArm();
				//	break;
			default:
				Product = new kinjo::JacoArm();
				break;
			}

			return Product;
		} //getInstance
	}; //class
} //namespace