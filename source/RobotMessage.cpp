#include "RobotMessage.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// --------------------
// RobotMessageField
// --------------------
RobotMessageField::RobotMessageField() {
  this->init();
}

void RobotMessageField::init() {
  for (int i = 0; i < FIELD_NUMBER; i ++) {
    mFieldList[i] = "";
  }
  mSize = 0;
}

int RobotMessageField::size() {
  return mSize;
}

std::string & RobotMessageField::operator[](int index) {
  return mFieldList[index];
}

bool RobotMessageField::parse(char * msgBuffer) {
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
// RobotMessage
// --------------------
RobotMessage::RobotMessage() {
  this->mType = MSG_TYPE_UNKNOWN; // Init to unknown
}

RobotMessage::RobotMessage(char messageType, int senderId) {
  this->mType = messageType;
  this->mSenderID = senderId;
}

RobotMessage::RobotMessage(int senderId, int command) {
  this->mType = MSG_TYPE_COMMAND;
  this->mSenderID = senderId;
  this->mCommand = command;
}

RobotMessage::RobotMessage(int senderId, double x, double y) {
  this->mType = MSG_TYPE_POSITION;
  this->mSenderID = senderId;
  this->mX = x;
  this->mY = y;
}

std::string RobotMessage::toString() {
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

bool RobotMessage::parse(char * message) {
  RobotMessageField msgField;
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

char RobotMessage::getType() {
  return mType;
}

int RobotMessage::getCommand() {
  return mCommand;
}

int RobotMessage::getSenderID() {
  return mSenderID;
}

double RobotMessage::getX() {
  return mX;
}

double RobotMessage::getY() {
  return mY;
}