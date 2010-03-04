
all:
	g++ -c -o Fiducial.o FiducialFinder.cpp -I/usr/include/opencv
	g++ -c -o vision.o vision.cpp -I/usr/include/opencv
	g++ -o vServer vision.o Fiducial.o -Llibv4l2cam -lcam -Llibfidtrack -lfidtrack -L/usr/local/lib -lcv -lcxcore -lcvaux -lhighgui 
	rm -f *.o

clean:
	rm -f *.o
	rm -f *.a





