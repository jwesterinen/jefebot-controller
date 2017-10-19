TARGET = jefebot

SRC = ./src
INC = ./include
OBJ = ./obj
BIN = ./bin

INCLUDES = -I./include -I../dp-framework/include
LIBS = -lm -ldp-framework

CPPFLAGS = $(INCLUDES) -O0 -g -Wall -c
LFLAGS = -L../dp-framework/lib

HEADERS = $(INC)/peripherals.h $(INC)/controller.h $(INC)/adc.h
OBJECTS = $(OBJ)/peripherals.o $(OBJ)/controller.o $(OBJ)/adc.o $(OBJ)/jefebot.o

.PHONY: all
all: $(TARGET)

$(TARGET) : $(OBJECTS)
	g++ $(LFLAGS) -o $(BIN)/$@ $(OBJECTS) $(LIBS)

$(OBJ)/%.o: $(SRC)/%.cpp $(HEADERS)
	g++ -c $(CPPFLAGS) -o $@ $<


.PHONY: clean
clean:
	rm -f $(BIN)/* $(OBJ)/*


