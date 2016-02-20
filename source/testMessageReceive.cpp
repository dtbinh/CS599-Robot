#include "RobotSocketConnection.h"
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <termios.h>
#include <string.h>
#include <sys/select.h>

int main(int argc, char ** argv) {
  int localPort = atoi(argv[1]);

  try {
    RobotSocket socket;
    socket.createListen(localPort);

    char recvBuf[MAX_SOCKET_BUF];
    bool exitLoop = false;
    while (!exitLoop) {
      int nBytes = socket.listen(recvBuf, MAX_SOCKET_BUF);
      if (nBytes > 0) {
        rewind(stdout);
        std::cout << "Received: " << recvBuf << std::endl;
        rewind(stdout);
        std::cout << "Size: " << nBytes << std::endl;
      }
      //sleep(1);
    }

  } catch (RobotSocketException e) {
    std::cerr << e.what() << std::endl;
    exit(1);
  }

  return 0;
}