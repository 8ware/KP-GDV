#include <kinjo/arm/ArmFactory.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main(/*int argc, char* argv[]*/){

	try
	{
		std::shared_ptr<kinjo::arm::Arm> const Arm = kinjo::arm::ArmFactory::getInstance();
		
		return 0;
    }
    catch(std::exception const & e)
    {
        std::cerr << e.what() << std::endl;
        return 1;
    }
    catch(...)
    {
        std::cerr << "Unknown exception!" << std::endl;
        return 1;
    }
}