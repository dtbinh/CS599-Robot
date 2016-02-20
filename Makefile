CC = g++
CFLAGS = -c -std=c++11 $(shell pkg-config --cflags playerc++)
LDFLAGS = $(shell pkg-config --libs playerc++)

SRC = source/
BIN = build/
PROG = args behavior InputControl RobotMessage RobotSocketConnection RobotTimer safewalk

SRCS = $(addprefix $(SRC), $(PROG))
BINS = $(addsuffix .o, $(addprefix $(BIN), $(basename $(PROG))))

all: target

target: $(BINS)
	$(CC) $(BINS) -o $(BIN)safewalk $(LDFLAGS)

$(BINS): $(BIN)%.o: $(SRC)%.cpp
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -f $(BINS)
	rm -f $(BIN)safewalk