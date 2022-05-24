output: main.o
	g++ main.o -o output

main.o: main.cpp
	g++ -c -std=c++17 main.cpp

clean: 
	rm *.o output
