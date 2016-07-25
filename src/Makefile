CC=g++ -std=c++11 
CXXFLAGS=-I.
OPENCV=`pkg-config --libs --cflags opencv` 
OPT=-O3
DEPS=slam.hpp asvs.hpp marker.hpp camera_param_reader.hpp map.hpp 
OBJ=main.o slam.o asvs.o  marker.o camera_param_reader.o map.o 

%.o: %.cpp $(DEPS)
	$(CC) $(OPT)  -c -o $@ $< $(CFLAGS) `pkg-config --cflags opencv`

eyes: $(OBJ)
	$(CC) $(OPT)  -o $@ $^ $(CFLAGS) $(OPENCV) -L/usr/local/lib/

.PHONY: clean

clean:
	rm -f *.o 