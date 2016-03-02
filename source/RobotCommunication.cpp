#include "RobotCommunication.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

namespace RobotCommunication {

  // --------------------
  // MessageField
  // --------------------
  MessageField::MessageField() {
    this->init();
  }

  void MessageField::init() {
    for (int i = 0; i < FIELD_NUMBER; i ++) {
      mFieldList[i] = "";
    }
    mSize = 0;
  }

  int MessageField::size() {
    return mSize;
  }

  std::string & MessageField::operator[](int index) {
    return mFieldList[index];
  }

  bool MessageField::parse(char * msgBuffer) {
    char * ptr = strchr(msgBuffer, MESSAGE_SEPERATOR);
    if (ptr != NULL)
    {
      int messageLength = ptr - msgBuffer + 1;
      char message[messageLength];
      memcpy(message, msgBuffer, messageLength);
      message[messageLength - 1] = '\0';

      ptr = message;
      this->init();
      while (ptr != NULL)
      {
        char* ptrNext = strchr(ptr, FIELD_SEPERATOR);
        if (ptrNext == NULL)
          ptrNext = message + messageLength - 1;
        
        int fieldSize = ptrNext - ptr;
        std::string field;
        field.append(ptr, fieldSize);
        mFieldList[mSize ++] = field;

        ptr = (ptrNext[0] == FIELD_SEPERATOR)? ptrNext + 1: NULL;
      }
      return true;
    }

    return false;
  }

  // --------------------
  // Message
  // --------------------
  Message::Message(): mType(MSG_TYPE_UNKNOWN) { }
  Message::Message(char messageType, int senderId): mType(messageType), mSenderID(senderId) { }
  Message::Message(int senderId, int command): mType(MSG_TYPE_COMMAND), mSenderID(senderId), mCommand(command) { }
  Message::Message(int senderId, double x, double y): mType(MSG_TYPE_POSITION), mSenderID(senderId), mX(x), mY(y) { }

  std::string Message::toString() {
    char msgBuffer[60];
    switch (mType) {
      case MSG_TYPE_COMMAND:
        sprintf(msgBuffer, "%c$%d$%d!", MSG_TYPE_COMMAND, mSenderID, mCommand);
        break;

      case MSG_TYPE_POSITION:
        sprintf(msgBuffer, "%c$%d$%.2f$%.2f!", MSG_TYPE_POSITION, mSenderID, mX, mY);
        break;

      case MSG_TYPE_REGISTER:
        sprintf(msgBuffer, "%c$%d!", MSG_TYPE_REGISTER, mSenderID);
        break;

      case MSG_TYPE_REQUESTREGISTER:
        sprintf(msgBuffer, "%c$%d!", MSG_TYPE_REQUESTREGISTER, mSenderID);
        break;

      default:
        msgBuffer[0] = '\0';
        break;
    }

    return std::string(msgBuffer);
  }

  bool Message::parse(char * message) {
    MessageField msgField;
    if (!msgField.parse(message)) {
      return false;
    }

    if (msgField.size() < 2) {
      return false;
    }

    char msgType = msgField[0][0];
    switch (msgType) {
      case MSG_TYPE_COMMAND:
        mType = MSG_TYPE_COMMAND;
        mSenderID = atoi(msgField[1].c_str());
        mCommand = atoi(msgField[2].c_str());
        break;

      case MSG_TYPE_POSITION:
        mType = MSG_TYPE_POSITION;
        mSenderID = atoi(msgField[1].c_str());
        mX = atof(msgField[2].c_str());
        mY = atof(msgField[3].c_str());      
        break;

      case MSG_TYPE_REGISTER:
      case MSG_TYPE_REQUESTREGISTER:
        mType = msgType;
        mSenderID = atoi(msgField[1].c_str());
        break;

      default:
        return false;
        break;
    }

    return true;
  }

  char Message::getType() {
    return mType;
  }

  int Message::getCommand() {
    return mCommand;
  }

  int Message::getSenderID() {
    return mSenderID;
  }

  double Message::getX() {
    return mX;
  }

  double Message::getY() {
    return mY;
  }

  /*
  ** Implementation of Communication
  */
  bool Communication::listenMessage(RobotNetwork::Socket &socket, RobotCommunication::Message& message)
  {
    char recvBuffer[RobotNetwork::MAX_SOCKET_BUF];

    int nBytes = socket.listen(recvBuffer, RobotNetwork::MAX_SOCKET_BUF);
    if (nBytes <= 0) {
      return false;
    }

    if (!message.parse(recvBuffer)) {
      return false;
    }

    return true;
  }

  void Communication::sendCommand(RobotNetwork::Socket &socket, std::string remoteAddress, int listenPort, int robotID, int command)
  {
    Message message(robotID, command);
    socket.send(message.toString(), remoteAddress, listenPort);
  }

  void Communication::sendMessagePosition(RobotNetwork::Socket &socket, std::string remoteAddress, int listenPort, int robotID, double x, double y)
  {
    Message message(robotID, x, y);
    socket.send(message.toString(), remoteAddress, listenPort);
  }

  bool Communication::waitForMessage(Message &message, char expectedType) {
    return (message.getType() == expectedType);
  }

  bool Communication::waitForCommand(Message &message, char expectedCommand)
  {
    return ((message.getType() == MSG_TYPE_COMMAND) && (message.getCommand() == expectedCommand));
  }

}
