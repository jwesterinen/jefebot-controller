TARGET = jefebot

SRC = ./src
INC = ./include
OBJ = ./obj
BIN = ./bin

INCLUDES = -I./include
LIBS = -lm

CFLAGS = $(INCLUDES) -O0 -g -Wall -c
LFLAGS = 

HEADERS = $(INC)/framework.h $(INC)/dp_peripherals.h $(INC)/peripherals.h $(INC)/controller.h $(INC)/adc.h
OBJECTS = $(OBJ)/framework.o $(OBJ)/dp_peripherals.o $(OBJ)/peripherals.o $(OBJ)/controller.o $(OBJ)/adc.o $(OBJ)/jefebot.o

.PHONY: all
all: $(TARGET)

$(TARGET) : $(OBJECTS)
	g++ $(LFLAGS) -o $(BIN)/$@ $(OBJECTS) $(LIBS)

$(OBJ)/%.o: $(SRC)/%.cpp $(HEADERS)
#	g++ -c $(CFLAGS) -o $@ $^
	g++ -c $(CFLAGS) -o $@ $<


.PHONY: clean
clean:
	rm -f $(BIN)/* $(OBJ)/*


