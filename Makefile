CC = g++
CFLAGS = -c -std=c++11 $(shell pkg-config --cflags playerc++)
LDFLAGS = $(shell pkg-config --libs playerc++)
AR = ar rvs

SRC = source/
BIN = build/

LIBS = RobotCommunication SocketConnection RobotBehavior ArgumentsParser RobotFormation
LIBOBJ = $(addsuffix .o, $(addprefix $(BIN), $(LIBS)))

all: robot taskManager test

# Robot
robot: buildenv $(BIN)robot	
$(BIN)robot: $(LIBOBJ) $(BIN)args.o $(BIN)robot.o
	$(CC) $(LIBOBJ) $(BIN)args.o $(BIN)robot.o -o $(BIN)robot $(LDFLAGS)

# TaskManager
taskManager: buildenv $(BIN)taskManager	
$(BIN)taskManager: $(LIBOBJ) $(BIN)taskManager.o
	$(CC) $(LIBOBJ) $(BIN)taskManager.o -o $(BIN)taskManager $(LDFLAGS)

# build test codes
test: testCmd randPos networkMonitor

randPos: buildenv $(BIN)randPos
$(BIN)randPos: $(LIBOBJ) $(BIN)randPos.o
	$(CC) $(LIBOBJ) $(BIN)randPos.o -o $(BIN)randPos $(LDFLAGS)

testCmd: buildenv $(BIN)testCmd
$(BIN)testCmd: $(LIBOBJ) $(BIN)testCmd.o
	$(CC) $(LIBOBJ) $(BIN)testCmd.o -o $(BIN)testCmd $(LDFLAGS)

networkMonitor: buildenv $(BIN)networkMonitor
$(BIN)networkMonitor: $(LIBOBJ) $(BIN)networkMonitor.o
	$(CC) $(LIBOBJ) $(BIN)networkMonitor.o -o $(BIN)networkMonitor $(LDFLAGS)

# Compile any .o files
$(BIN)%.o: $(SRC)%.cpp
	$(CC) $(CFLAGS) $< -o $@

# The build directory
buildenv: $(BIN)
$(BIN):
	mkdir -p $(BIN)

# Clean
clean:
	rm -f $(BIN)*