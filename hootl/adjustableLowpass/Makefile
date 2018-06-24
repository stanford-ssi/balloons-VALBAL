CC=g++
CFLAGS=-I ../../src/ -I ../ -std=c++17 -D JOHNSIM
_SRCF = Utils
SRCF = $(addprefix ../../src/, $(addsuffix .cpp, $(_SRCF)))


all: build run

build:
	$(CC) $(CFLAGS) -o ut ut.cpp $(SRCF)

run:
	./ut #> output.csv

plot:
	python plot.py

clean:
	rm spag-ut
	rm data.csv
