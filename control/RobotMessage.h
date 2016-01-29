#ifndef __ROBOTMESSAGE_H__
#define __ROBOTMESSAGE_H__

#include <string>
#include <list>

#define MESSAGE_SEPERATOR '!'
#define FIELD_SEPERATOR '$'
#define FIELD_NUMBER 10

#define CMD_START 1
#define CMD_STOP 0
#define CMD_RESUME 2
#define CMD_EXIT -1

#define MSG_TYPE_COMMAND 'C'
#define MSG_TYPE_POSITION 'P'
#define MSG_TYPE_REGISTER 'R'
#define MSG_TYPE_REQUESTREGISTER 'B'
#define MSG_TYPE_UNKNOWN 'U'

class RobotMessageField {
public:
  RobotMessageField();
  bool parse(char * msgBuffer);
  int size();
  std::string &operator[](int index);

private:
  int mSize;
  std::string mFieldList[FIELD_NUMBER];
  void init();
};

class RobotMessage {
public:
  RobotMessage();
  RobotMessage(char messageType, int senderId);
  RobotMessage(int senderId, int command);
  RobotMessage(int senderId, double x, double y);
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

#endif