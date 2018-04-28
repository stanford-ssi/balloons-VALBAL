CC=g++
CFLAGS=-I ../../src/ -I ../ -std=c++17 -D JOHNSIM
_SRCF = Utils LasagnaController SpaghettiController2
SRCF = $(addprefix ../../src/, $(addsuffix .cpp, $(_SRCF)))


all: build run

build:
	$(CC) $(CFLAGS) -o lasagnaSim spag2Sim.cpp PastaSim.cpp $(SRCF)

run:
	./lasagnaSim #> output.csv

plot:
	python plot.py

clean:
	rm spag-ut
	rm data.csv
