CC=g++
CFLAGS=-I ../../src/ -I ../../lib/spa -I ../ -std=c++17 -D JOHNSIM
_SRCF = Utils LasagnaController
SRCF = $(addprefix ../../src/, $(addsuffix .cpp, $(_SRCF)))
LIBF = ../../lib/spa/spa.cpp

all: build run

build:
	$(CC) $(CFLAGS) -o lasagnaSim lasagnaSim.cpp PastaSim.cpp $(SRCF) $(LIBF)

run:
	./lasagnaSim #> output.csv

plot:
	python plot.py

clean:
	rm lasagnaSim
	rm data.csv
