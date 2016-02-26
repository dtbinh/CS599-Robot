#ifndef __ROBOTSOCKETCONNECTION_H__
#define __ROBOTSOCKETCONNECTION_H__

#include <string>
#include <exception>
#include <netinet/in.h>      // For sockaddr_in

namespace RobotNetwork {

  const int MAX_SOCKET_BUF = 1024;

  class SocketException: public std::exception {
  public:
    SocketException(const std::string &message) throw();
    const char * what() const throw();
    ~SocketException() throw();

  private:
    std::string mUserMessage;
  };

  class Socket {
  public:
    Socket();
    ~Socket();

    void close();
    // Receiver
    void createListen(int listenPort) throw(SocketException);
    int listen(char * buffer, int bufferLen) throw(SocketException);
    // Sender
    void createSend() throw(SocketException);
    int send(std::string message, std::string remoteAddress, int remotePort) throw(SocketException);

  protected:
    int mFileDescriptor;

    void fillAddressAndPort(std::string ipAddress, int portNumber, sockaddr_in &addr);
    void setLocalPort(int localPort) throw(SocketException);
    void setBroadcast() throw(SocketException);
    void create() throw(SocketException);
  };

}

#endif