TARGET = MinimalArm

QMAKE_CXXFLAGS += -std=c++0x

INCLUDEPATH += /usr/include/libusb-1.0

LIBS			+= /usr/lib/x86_64-linux-gnu/libusb-1.0.so

HEADERS = libkindrv/types.h \
			 libkindrv/kindrv.h \
			 libkindrv/exception.h 

SOURCES = libkindrv/kindrv.cpp \
			 libkindrv/exception.cpp \
			 MinimalArm.cpp

