CC=g++
CFLAGS=-lraylib
SRC=src
OBJ=obj
SRCS=$(wildcard $(SRC)/*.cpp)
OBJS=$(patsubst $(SRC)/%.cpp, $(OBJ)/%.o, $(SRCS))
BINDIR=bin
BIN=$(BINDIR)/main

all: $(BIN)

$(BIN): $(OBJS)
		$(CC) $(OBJS) -o $@ $(CFLAGS)

$(OBJ)/%.o: $(SRC)/%.cpp
		$(CC) -c $< -o $@ 

run: $(BIN)
		./$(BIN)
clean:
		$(RM) $(BINDIR)/* $(OBJ)/*

debug: $(OBJS)
		$(CC) $(OBJS) -g -o $(BINDIR)/$@ $(CFLAGS)
