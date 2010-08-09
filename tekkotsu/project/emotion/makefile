all:	t t poses.o
	make t; make poses.o
t: 	main.o emotion.o emotion.h poses.o
	g++ -o t main.o emotion.o poses.o
main.o:	main.cpp emotion.h
	g++ -c main.cpp
emotion.o:	emotion.cpp emotion.h
	g++ -c emotion.cpp
poses.o:	poses.cpp poses.h
	g++ -c poses.cpp
clean:
	rm -f *.o t core
