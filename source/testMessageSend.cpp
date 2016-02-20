#include "RobotSocketConnection.h"
#include <iostream>
#include <stdlib.h>

int main(int argc, char ** argv) {
  std::string destAddress = argv[1];             // First arg:  destination address
  unsigned short destPort = atoi(argv[2]);  // Second arg: destination port
  char* sendString = argv[3];               // Third arg:  string to broadcast

  try {
    RobotSocket socket;
    socket.createSend();
    socket.send(std::string(sendString), destAddress, destPort);
  } catch (RobotSocketException &e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  return 0;
}