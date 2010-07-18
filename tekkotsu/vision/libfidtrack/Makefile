
all:
	g++ -c -o fidtrackX.o fidtrackX.cpp 
	g++ -c -o segment.o segment.cpp
	g++ -c -o treeidmap.o treeidmap.cpp
	g++ -c -o threshold.o tiled_bernsen_threshold.c 
	ar rc libfidtrack.a fidtrackX.o segment.o treeidmap.o threshold.o
	rm -f *.o

clean:
	rm -f *.o
	rm -f *.a





