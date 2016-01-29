CC = g++
CFLAGS = -c -std=c++11 $(shell pkg-config --cflags playerc++)
LDFLAGS = $(shell pkg-config --libs playerc++)
OUTDIR = build

all: safewalk

safewalk: safewalk.o behavior.o RobotMessage.o RobotSocketConnection.o args.o RobotTimer.o
	$(CC) $(OUTDIR)/safewalk.o $(OUTDIR)/behavior.o $(OUTDIR)/RobotMessage.o $(OUTDIR)/RobotSocketConnection.o $(OUTDIR)/args.o $(OUTDIR)/RobotTimer.o -o $(OUTDIR)/safewalk $(LDFLAGS)

behavior.o: behavior.cc behavior.h RobotMessage.h RobotSocketConnection.h
	$(CC) $(CFLAGS) behavior.cc -o $(OUTDIR)/behavior.o

safewalk.o: safewalk.cc behavior.h args.h RobotMessage.h RobotSocketConnection.h RobotTimer.h
	$(CC) $(CFLAGS) safewalk.cc -o $(OUTDIR)/safewalk.o

args.o: args.cc args.h
	$(CC) $(CFLAGS) args.cc -o $(OUTDIR)/args.o

RobotMessage.o: RobotMessage.cpp RobotMessage.h
	$(CC) $(CFLAGS) RobotMessage.cpp -o $(OUTDIR)/RobotMessage.o

RobotSocketConnection.o: RobotSocketConnection.cpp RobotSocketConnection.h
	$(CC) $(CFLAGS) RobotSocketConnection.cpp -o $(OUTDIR)/RobotSocketConnection.o

RobotTimer.o: RobotTimer.cpp RobotTimer.h
	$(CC) $(CFLAGS) RobotTimer.cpp -o $(OUTDIR)/RobotTimer.o

testSocket: testMessageSend.cpp testMessageReceive.cpp RobotSocketConnection.h RobotSocketConnection.o
	$(CC) $(CFLAGS) testMessageSend.cpp -o $(OUTDIR)/testMessageSend.o
	$(CC) $(CFLAGS) testMessageReceive.cpp -o $(OUTDIR)/testMessageReceive.o
	$(CC) $(OUTDIR)/testMessageSend.o $(OUTDIR)/RobotSocketConnection.o -o $(OUTDIR)/testMessageSend
	$(CC) $(OUTDIR)/testMessageReceive.o $(OUTDIR)/RobotSocketConnection.o -o $(OUTDIR)/testMessageReceive

clean:
	rm -f $(OUTDIR)/*.o
	rm -f $(OUTDIR)/safewalk