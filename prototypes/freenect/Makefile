ll: test

CFLAGS=-fPIC -g -Wall -pthread `pkg-config --cflags opencv` 
LIBS = `pkg-config --libs opencv`
INCLUDE = -I/usr/include/libfreenect
FREE_LIBS = -L/usr/lib -lfreenect

test:  main.cpp
	$(CXX) $(INCLUDE) $(CFLAGS) $? -o $@  $(LIBS) $(FREE_LIBS)

%.o: %.cpp
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o test
