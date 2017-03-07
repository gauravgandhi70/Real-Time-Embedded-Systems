#Makefile#

CC=gcc
cflag=-lpthread -lrt


all: process1 process2


process1: process1.o
	$(CC) -o $@ $< -pthread


process2: process2.o
	$(CC) -o process2 process2.o -pthread

.o:.c
	$(cc) -c $< -o $@

	
clean:
	rm -f process1 process2
	rm -f *.o
