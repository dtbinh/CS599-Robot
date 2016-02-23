#include <iostream>
#include <stdlib.h>

#include "behavior.h"
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

int main(int argc, char ** argv) {
  std::string destAddress = argv[1];             // First arg:  destination address
  unsigned short destPort = atoi(argv[2]);  // Second arg: destination port
  int cmd = atoi(argv[3]);               // Third arg:  commandID

  try {
    RobotSocket sendSocket;
    sendSocket.createSend();
    sendCommand(sendSocket, destAddress, destPort, 1, cmd);
  } catch (RobotSocketException &e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  return 0;
}