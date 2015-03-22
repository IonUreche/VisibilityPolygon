CC=g++
LIBS = -lGL -lglut
SOURCE = main.cpp
EXECUTABLE = main

all:
	$(CC) $(SOURCE) -o $(EXECUTABLE) $(LIBS)