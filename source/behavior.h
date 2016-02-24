#ifndef __BEHAVIOR_H__
#define __BEHAVIOR_H__

#include <libplayerc++/playerc++.h>
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

#define PI 3.1415926535
#define MAX_ROBOT_NUMBER 10

class RobotInfo
{
public:
  double x = 0;
  double y = 0;
  int robotID = -1;
};

class RobotList
{
public:
  RobotList();
  int getSize();
  RobotInfo* getInfo(int index);
  void updateInfo(const RobotInfo &info);
  void deleteInfo(const int robotID);

private:
  RobotInfo mList[MAX_ROBOT_NUMBER];
  int mSize;
};


void sendCommand(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID, int command);
void sendMessagePosition(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID, double x, double y);
bool listenMessage(RobotSocket &socket, RobotMessage& message);
bool waitForCommand(RobotMessage &message, char expectedCommand);
bool waitForMessage(RobotMessage &message, char expectedType);

void doDisperse(double &forwardSpeed, double &turnSpeed, double myX, double myY, double myYaw, RobotList &robotList, double rangeLimit);
void doAggregate(double &forwardSpeed, double &turnSpeed, double myX, double myY, double myYaw, RobotList &robotList, double rangeLimit);

#endif