CC=g++
CFLAGS=-I ../../src/ -I ../ -std=c++17
_SRCF = Utils SpaghettiController2
SRCF = $(addprefix ../../src/, $(addsuffix .cpp, $(_SRCF)))


all: build run

build: 
	$(CC) $(CFLAGS) -o spag-ut spag-ut.cpp $(SRCF) 

run: 
	./spag-ut #> output.csv

plot: 
	python plot.py

clean: 
	rm spag-ut
	rm data.csv