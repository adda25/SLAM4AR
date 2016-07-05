CC=g++ -std=c++11 
CXXFLAGS=-I.
OPENCV=`pkg-config --libs --cflags opencv`
OPT=-O2
DEPS=slam.hpp asvs.hpp robust_matcher.hpp marker.hpp camera_param_reader.hpp
OBJ=main.o slam.o asvs.o robust_matcher.o marker.o camera_param_reader.o

%.o: %.cpp $(DEPS)
	$(CC) $(OPT)  -c -o $@ $< $(CFLAGS) `pkg-config --cflags opencv`

eyes: $(OBJ)
	$(CC) $(OPT)  -o $@ $^ $(CFLAGS) $(OPENCV)

.PHONY: clean

clean:
	rm -f *.o 