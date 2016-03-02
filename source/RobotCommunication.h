#ifndef __ROBOT_COMMUNICATION_H__
#define __ROBOT_COMMUNICATION_H__

#include <string>
#include <list>
#include "SocketConnection.h"

namespace RobotCommunication {

  const char MESSAGE_SEPERATOR = '!';
  const char FIELD_SEPERATOR = '$';
  const int FIELD_NUMBER = 10;

  const int CMD_START = 1;
  const int CMD_STOP = 0;
  const int CMD_RESUME = 2;
  const int CMD_EXIT = -1;

  const char MSG_TYPE_COMMAND = 'C';
  const char MSG_TYPE_POSITION = 'P';
  const char MSG_TYPE_REGISTER = 'R';
  const char MSG_TYPE_REQUESTREGISTER = 'B';
  const char MSG_TYPE_UNKNOWN = 'U';

  class MessageField {
  public:
    MessageField();
    bool parse(char * msgBuffer);
    int size();
    std::string &operator[](int index);

  private:
    int mSize;
    std::string mFieldList[FIELD_NUMBER];
    void init();
  };

  class Message {
  public:
    Message();
    Message(char messageType, int senderId);
    Message(int senderId, int command);
    Message(int senderId, double x, double y);
    std::string toString();
    bool parse(char * message);
    char getType();
    int getCommand();
    int getSenderID();
    double getX();
    double getY();

  private:
    char mType;
    int mSenderID;
    int mCommand;
    double mX;
    double mY;
  };

  class Communication {
  public:
    void sendCommand(RobotNetwork::Socket &socket, std::string remoteAddress, int listenPort, int robotID, int command);
    void sendMessagePosition(RobotNetwork::Socket &socket, std::string remoteAddress, int listenPort, int robotID, double x, double y);
    bool listenMessage(RobotNetwork::Socket &socket, RobotCommunication::Message& message);
    bool waitForCommand(Message &message, char expectedCommand);
    bool waitForMessage(Message &message, char expectedType);

  };
}

#endif