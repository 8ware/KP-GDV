
SRC = ../src
INC = ../include
EXT = ../external

GCC   = g++
FLAGS = `pkg-config --cflags libopenni opencv libusb-1.0` -I$(INC) -I$(EXT)
LIBS  = `pkg-config --libs libopenni opencv libusb-1.0`

OpenNiVisionTest:
	@$(GCC) -std=c++11 $(FLAGS) $(LIBS) -o OpenNiVisionTest \
		OpenNiVisionTest.cpp $(SRC)/vision/OpenNiVision.cpp

TestdataGenerator:
	@$(GCC) -std=c++11 $(FLAGS) $(LIBS) -o TestdataGenerator \
		TestdataGenerator.cpp $(SRC)/vision/OpenNiVision.cpp \
		$(SRC)/arm/ArmFactory.cpp $(SRC)/arm/JacoArm.cpp \
		$(EXT)/libkindrv/exception.cpp $(EXT)/libkindrv/kindrv.cpp

DepthImageTest:
	@$(GCC) -std=c++11 $(FLAGS) $(LIBS) -o DepthImageTest DepthImageTest.cpp

DirectoryBasedDataProviderTest:
	@$(GCC) -std=c++11 $(FLAGS) $(LIBS) -o DirectoryBasedDataProviderTest \
		DirectoryBasedDataProviderTest.cpp $(SRC)/mock/DirectoryBasedDataProvider.cpp

TestdataMockTest:
	@$(GCC) -std=c++11 $(FLAGS) $(LIBS) -o TestdataMockTest \
		TestdataMockTest.cpp $(SRC)/mock/TestdataMock.cpp \
		$(SRC)/mock/DirectoryBasedDataProvider.cpp

