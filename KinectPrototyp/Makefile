ll: test

CFLAGS=-fPIC -g -Wall `pkg-config --cflags opencv` -lpthread
LIBS = `pkg-config --libs opencv`
INCLUDE = -I/usr/include/libfreenect
FREE_LIBS = -L/usr/lib -lfreenect

test:  main.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test
