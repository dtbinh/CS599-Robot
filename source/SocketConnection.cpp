#include "SocketConnection.h"
#include <errno.h>
#include <sys/types.h>       // For data types
#include <sys/socket.h>      // For socket(), connect(), send(), and recv()
//#include <netdb.h>           // For gethostbyname()
#include <arpa/inet.h>       // For inet_addr()
#include <unistd.h>          // For close()
#include <string.h>            // for strerror()
#include <fcntl.h>            // For fcntl()

namespace Robot {

  // ----------------------------------------
  // Implementation of SocketException
  // ----------------------------------------
  SocketException::SocketException(const std::string &message) throw(): mUserMessage(message) {
    mUserMessage.append(": ");
    mUserMessage.append(strerror(errno));
  }

  const char * SocketException::what() const throw() {
    return mUserMessage.c_str();
  }

  SocketException::~SocketException() throw() {}

  // ----------------------------------------
  // Implementation of Socket
  // ----------------------------------------
  Socket::Socket(): mFileDescriptor(-1) { }

  Socket::~Socket() {
    this->close();
  }

  void Socket::close() {
    if (mFileDescriptor > 0) {
      ::close(mFileDescriptor);
    }
    mFileDescriptor = -1;
  }

  void Socket::create() throw(SocketException) {
    this->close();

    if ((mFileDescriptor = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
      throw SocketException("Create socket failed");
    }
    
    // to avoid the "address already in use" error
    const int yes = 1;
    socklen_t optlen = sizeof(yes);
    if (setsockopt(mFileDescriptor, SOL_SOCKET, SO_REUSEADDR, &yes, optlen) < 0) {
      throw SocketException("setsockopt SO_REUSEADDR failed");
    }

    #ifdef __APPLE__   // MacOS/X requires an additional call also
    if (setsockopt(mFileDescriptor, SOL_SOCKET, SO_REUSEPORT, &yes, optlen) < 0) {
      throw SocketException("setsockopt SO_REUSEPORT failed");
    }
    #endif
  }

  void Socket::createListen(int listenPort) throw(SocketException) {
    this->create();
    this->setLocalPort(listenPort);
  }

  void Socket::createSend() throw(SocketException) {
    this->create();
    this->setBroadcast();
  }

  void Socket::setLocalPort(int localPort) throw(SocketException) {
    // non-block
    fcntl(mFileDescriptor, F_SETFL, O_NONBLOCK);

    // setup local echo address
    sockaddr_in localAddress;
    localAddress.sin_family = AF_INET;
    localAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddress.sin_port = htons(localPort);
    memset(&(localAddress.sin_zero), '\0', 8);

    socklen_t optlen = sizeof(localAddress);
    if (bind(mFileDescriptor, (sockaddr *) &localAddress, optlen) < 0) {
      throw SocketException("Bind failed");
    }
  }

  void Socket::setBroadcast() throw(SocketException) {
    int yes = 1;
    socklen_t optlen = sizeof(yes);
    if (setsockopt(mFileDescriptor, SOL_SOCKET, SO_BROADCAST, &yes, optlen) < 0) {
        throw SocketException("setsockopt BROADCAST failed");
    }
  }

  int Socket::listen(char * buffer, int bufferLen) throw(SocketException) {
      sockaddr_in fromAddr; // connector's address information
      socklen_t addrLength = sizeof(fromAddr); 
      int nBytes; // number of bytes

      nBytes = recvfrom(mFileDescriptor, buffer, bufferLen-1, 0, (sockaddr *) &fromAddr, &addrLength);
      if (nBytes < 0) {
        nBytes = 0;
      }
      buffer[nBytes] = '\0';

      return nBytes;
  }

  int Socket::send(std::string message, std::string remoteAddress, int remotePort) throw(SocketException) {
    int nBytes = 0;

    sockaddr_in addr;
    socklen_t optlen = sizeof(addr);
    fillAddressAndPort(remoteAddress, remotePort, addr);
    if ((nBytes = sendto(mFileDescriptor, message.c_str(), message.length(), 0, (sockaddr *)&addr, optlen)) == -1) {
      throw SocketException("sendto failed");
      return -1;
    }

    return nBytes;
  }

  void Socket::fillAddressAndPort(std::string ipAddress, int portNumber, sockaddr_in &addr) {
    addr.sin_addr.s_addr = inet_addr(ipAddress.c_str());
    addr.sin_family = AF_INET;
    addr.sin_port = htons(portNumber);
    memset(&(addr.sin_zero), '\0', 8);
  }

}
