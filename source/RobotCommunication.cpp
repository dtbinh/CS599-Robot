#include "RobotCommunication.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

namespace RobotCommunication {

  // --------------------
  // Constants definition
  // --------------------
  // const char MESSAGE_SEPERATOR = '!';
  // const char FIELD_SEPERATOR = '$';
  // const int FIELD_NUMBER = 10;

  // const int CMD_START = 1;
  // const int CMD_STOP = 0;
  // const int CMD_RESUME = 2;
  // const int CMD_EXIT = -1;

  // const char MSG_TYPE_TASK = 'T';
  // const char MSG_TYPE_COMMAND = 'C';
  // const char MSG_TYPE_POSITION = 'P';
  // const char MSG_TYPE_REGISTER = 'R';
  // const char MSG_TYPE_REQUESTREGISTER = 'B';
  // const char MSG_TYPE_UNKNOWN = 'U';

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
  Message::Message(int senderId, char formationType, double targetX, double targetY):
    mType(MSG_TYPE_TASK), mSenderID(senderId), mFormationType(formationType), mX(targetX), mY(targetY) { }


  std::string Message::toString() {
    char msgBuffer[60];
    switch (mType) {
      case MSG_TYPE_COMMAND:
        sprintf(msgBuffer, "%c$%d$%d!", MSG_TYPE_COMMAND, mSenderID, mCommand);
        break;

      case MSG_TYPE_POSITION:
        sprintf(msgBuffer, "%c$%d$%.2f$%.2f!", MSG_TYPE_POSITION, mSenderID, mX, mY);
        break;

      case MSG_TYPE_TASK:
        sprintf(msgBuffer, "%c$%d$%c$%.2f$%.2f!", MSG_TYPE_TASK, mSenderID, mFormationType, mX, mY);
        break;

      case MSG_TYPE_TASK_DONE:
        sprintf(msgBuffer, "%c$%d!", MSG_TYPE_TASK_DONE, mSenderID);
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

      case MSG_TYPE_TASK:
        mType = MSG_TYPE_TASK;
        mSenderID = atoi(msgField[1].c_str());
        mFormationType = msgField[2][0];
        mX = atof(msgField[3].c_str());
        mY = atof(msgField[4].c_str());      
        break;

      case MSG_TYPE_TASK_DONE:
        mType = MSG_TYPE_TASK_DONE;
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

  double Message::getFormationType() {
    return mFormationType;
  }
  /*
  ** Implementation of Communication
  */
  Communication::Communication(std::string address, int port): mAddress(address), mPort(port) {
    mSocketListen.createListen(mPort);
    mSocketSend.createSend();
  }

  bool Communication::listenMessage(Message& message)
  {
    char recvBuffer[RobotNetwork::MAX_SOCKET_BUF];

    int nBytes = mSocketListen.listen(recvBuffer, RobotNetwork::MAX_SOCKET_BUF);
    if (nBytes <= 0) {
      return false;
    }

    if (!message.parse(recvBuffer)) {
      return false;
    }

    return true;
  }

  void Communication::sendCommand(int robotID, int command)
  {
    Message message(robotID, command);
    mSocketSend.send(message.toString(), mAddress, mPort);
  }

  void Communication::sendMessagePosition(int robotID, double x, double y)
  {
    Message message(robotID, x, y);
    mSocketSend.send(message.toString(), mAddress, mPort);
  }

  void Communication::sendMessageTask(int robotID, char formationType, double targetX, double targetY) {
    Message message(robotID, formationType, targetX, targetY);
    mSocketSend.send(message.toString(), mAddress, mPort);
  }

  void Communication::sendMessageTaskDone(int robotID) {
    Message message(MSG_TYPE_TASK_DONE, robotID);
    mSocketSend.send(message.toString(), mAddress, mPort);
  }

  bool Communication::waitForMessage(Message &message, char expectedType) {
    return (message.getType() == expectedType);
  }

  bool Communication::waitForCommand(Message &message, char expectedCommand)
  {
    return ((message.getType() == MSG_TYPE_COMMAND) && (message.getCommand() == expectedCommand));
  }

}
