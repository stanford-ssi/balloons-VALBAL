CC=g++
CFLAGS=-I ../../src/ -I ../../lib/spa -I ../ -std=c++17 -D JOHNSIM
_SRCF = Utils
SRCF = $(addprefix ../../src/, $(addsuffix .cpp, $(_SRCF)))
LIBF = ../../lib/spa/spa.c 


all: build run

build:
	$(CC) $(CFLAGS) -o ut ut.cpp $(SRCF) $(LIBF)

run:
	./ut #> output.csv

plot:
	python plot.py

clean:
	rm spag-ut
	rm data.csv
