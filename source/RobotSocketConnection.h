#ifndef __ROBOTSOCKETCONNECTION_H__
#define __ROBOTSOCKETCONNECTION_H__

#include <string>
#include <exception>
#include <netinet/in.h>      // For sockaddr_in

#define MAX_SOCKET_BUF 1024

class RobotSocketException: public std::exception {
public:
  RobotSocketException(const std::string &message) throw();
  const char * what() const throw();
  ~RobotSocketException() throw();

private:
  std::string mUserMessage;
};

class RobotSocket {
public:
  RobotSocket();
  ~RobotSocket();

  void close();
  // Receiver
  void createListen(int listenPort) throw(RobotSocketException);
  int listen(char * buffer, int bufferLen) throw(RobotSocketException);
  // Sender
  void createSend() throw(RobotSocketException);
  int send(std::string message, std::string remoteAddress, int remotePort) throw(RobotSocketException);

protected:
  int mFileDescriptor;

  void fillAddressAndPort(std::string ipAddress, int portNumber, sockaddr_in &addr);
  void setLocalPort(int localPort) throw(RobotSocketException);
  void setBroadcast() throw(RobotSocketException);
  void create() throw(RobotSocketException);
};

#endif