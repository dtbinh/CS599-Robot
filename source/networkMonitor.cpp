#include <iostream>
#include <unistd.h>
#include "SocketConnection.h"

// Global variables
#ifdef __APPLE__
	#define DEFAULT_BC_ADDRESS "192.168.0.255"
#else
	#define DEFAULT_BC_ADDRESS "127.255.255.255"
#endif
#define DEFAULT_LISTEN_PORT 9090

int main(int argc, char** argv) {
	Robot::Socket listenSocket;
	listenSocket.createListen(DEFAULT_LISTEN_PORT);

  char recvBuffer[Robot::MAX_SOCKET_BUF];
	while (true) {
		while (listenSocket.listen(recvBuffer, Robot::MAX_SOCKET_BUF) > 0) {
			std::cout << recvBuffer << std::endl;
		}
		sleep(1);
	}
}