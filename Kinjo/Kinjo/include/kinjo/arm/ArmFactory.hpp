#pragma once
#include <kinjo/arm/Arm.hpp>
#include <kinjo/arm/JacoArm.hpp>
#include <libkindrv/kindrv.h>       // KinDrv::JacoArm

#include <memory>                   // std::shared_ptr

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

            static std::shared_ptr<kinjo::arm::Arm> getInstance(int ArmKey)
            {
                std::shared_ptr<kinjo::arm::Arm> Product;

                switch(ArmKey)
                {
                case 1:
                    Product = std::make_shared<kinjo::arm::JacoArm>();
                    break;
                    // How to add other Arm types:
                    //case 2:
                    //	Product = std::make_shared<kinjo::phantasieArm>();
                    //	break;
                default:
                    Product = std::make_shared<kinjo::arm::JacoArm>();
                    break;
                }

                return Product;
            } //getInstance
        }; //class
    
    }
} //namespace