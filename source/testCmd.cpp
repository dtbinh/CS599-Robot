#include <iostream>
#include <stdlib.h>

#include "RobotCommunication.h"
#include "SocketConnection.h"

int main(int argc, char ** argv) {
  std::string destAddress = argv[1];             // First arg:  destination address
  unsigned short destPort = atoi(argv[2]);  // Second arg: destination port
  int cmd = atoi(argv[3]);               // Third arg:  commandID

  try {
    Robot::Communication robotCommunication(destAddress, destPort);
    robotCommunication.sendCommand(1, cmd);
  } catch (Robot::SocketException &e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  return 0;
}