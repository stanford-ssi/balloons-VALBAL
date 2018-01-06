CC=g++
CFLAGS=-I ../src/ ../ -std=c++17 
_SRCF = Utils SpaghettiController2
SRCF = $(addprefix ../src/, $(addsuffix .cpp, $(_SRCF)))


all: build run

build: 
	$(CC) $(CFLAGS) -o sim sim.cpp $(SRCF) 

run: 
	./sim

clean: 
	rm sim 
