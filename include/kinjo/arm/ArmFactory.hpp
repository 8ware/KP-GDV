#pragma once
#include <kinjo/arm/Arm.hpp>
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

			/**
			* \returns a Instance of kinjo::arm::Arm in our case thats a Jaco Arm
			**/
			static std::shared_ptr<kinjo::arm::Arm> getInstance();
            
                
        }; //class
    
    }
} //namespace