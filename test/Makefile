
SRC = ../src
INC = ../include

GCC   = g++
FLAGS = `pkg-config --cflags libopenni opencv` -I$(INC)
LIBS  = `pkg-config --libs libopenni opencv`

OpenNiVisionTest:
	@$(GCC) $(FLAGS) $(LIBS) -o OpenNiVisionTest OpenNiVisionTest.cpp $(SRC)/vision/OpenNiVision.cpp

