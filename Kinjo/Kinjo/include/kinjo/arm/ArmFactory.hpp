#pragma once
#include <kinjo/arm/Arm.hpp>
#include <kinjo/arm/JacoArm.hpp>
#include <libkindrv/kindrv.h>       // KinDrv::JacoArm

#include <memory>                   // std::shared_ptr
#include <iostream>					// std::cout

namespace kinjo {
    namespace arm {

        //In Case other Arms are provided later we use a factory to get the arm.
        //So we can simply Implement the other arm and dont change any code in the
        //main program except the ArmKey at the beginning
        class ArmFactory
        {
        public:
            ArmFactory() = delete;
            ~ArmFactory() = default;

            static std::shared_ptr<kinjo::arm::Arm> getInstance()
            {
                std::shared_ptr<kinjo::arm::Arm> Product;
				try {
					Product = std::make_shared<kinjo::arm::JacoArm>();
					std::cout << "JacoArm found, using Jaco Arm." << std::endl;
				}
				catch (KinDrv::KinDrvException e)
				{
					std::cout << e.what() << std::endl;
					throw e;
				}
                

                return Product;
            } //getInstance
        }; //class
    
    }
} //namespace