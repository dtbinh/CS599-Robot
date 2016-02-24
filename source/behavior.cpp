#include <libplayerc++/playerc++.h>
#include <math.h>
#include <iostream>
#include "behavior.h"


// ************************
// communication functions
// ************************
bool listenMessage(RobotSocket &socket, RobotMessage& message)
{
  char recvBuffer[MAX_SOCKET_BUF];

  int nBytes = socket.listen(recvBuffer, MAX_SOCKET_BUF);
  if (nBytes <= 0) {
    return false;
  }

  if (!message.parse(recvBuffer)) {
    return false;
  }

  return true;
}

void sendCommand(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID, int command)
{
  RobotMessage message(robotID, command);
  socket.send(message.toString(), remoteAddress, listenPort);
}

void sendMessagePosition(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID, double x, double y)
{
  RobotMessage message(robotID, x, y);
  socket.send(message.toString(), remoteAddress, listenPort);
}

bool waitForMessage(RobotMessage &message, char expectedType) {
  return (message.getType() == expectedType);
}

bool waitForCommand(RobotMessage &message, char expectedCommand)
{
  return ((message.getType() == MSG_TYPE_COMMAND) && (message.getCommand() == expectedCommand));
}

// ************************
// Utility functions
// ************************
void calculateCentroid(RobotList &robotList, double myX, double myY, double &centroidX, double &centroidY)
{
  int count = 0;
  centroidX = 0;
  centroidY = 0;
  for (int i = 0; i < robotList.getSize(); i ++)
  {
    RobotInfo *pInfo = robotList.getInfo(i);
    if (pInfo->robotID < 0) { continue; }

    centroidX += pInfo->x;
    centroidY += pInfo->y;
    count ++;

    // DEBUG
    // char msg[100];
    // sprintf(msg, "Robot%d(%.2f, %.2f)", pInfo->robotID, pInfo->x, pInfo->y);
    // std::cout << msg << "; ";
  }

  // centroidX = centroidX/inRangeCount;
  // centroidY = centroidY/inRangeCount;
  centroidX = (centroidX + myX)/(count+1);
  centroidY = (centroidY + myY)/(count+1);
}

double findNearestDistance(RobotList &robotList, double myX, double myY)
{
  double result = -1;
  for (int i = 0; i < robotList.getSize(); i ++)
  {
    RobotInfo *pInfo = robotList.getInfo(i);
    if (pInfo->robotID < 0) { continue; }

    double x = pInfo->x - myX;
    double y = pInfo->y - myY;
    double distance = std::sqrt(x*x + y*y);

    if (result < 0 || distance < result)
    {
      result = distance;
    }
  }

  return result;
}

// ************************
// Behaviors
// ************************
void doDisperse(
  double &forwardSpeed, double &turnSpeed,
  double myX, double myY, double myYaw,
  RobotList &robotList,
  double rangeLimit)
{
  double centroidX = 0;
  double centroidY = 0;

  double nearestDistance = findNearestDistance(robotList, myX, myY);
  if (nearestDistance > 0 && nearestDistance < rangeLimit)
  { // Some body in range

    calculateCentroid(robotList, myX, myY, centroidX, centroidY);
    double targetX = myX - centroidX;
    double targetY = myY - centroidY;
    double newYaw = std::atan(targetY / targetX);
    if (targetX < 0) // transform the coordination from C to Player
    {
      newYaw = PI + newYaw;
    }

    // Normalize the Yaw values, fit in range [-PI, PI]
    if (myYaw > PI) {myYaw = myYaw - 2*PI;}
    if (newYaw > PI) {newYaw = newYaw - 2*PI;}

    // Determine speed & turn rate
    forwardSpeed = (std::fabs(newYaw - myYaw) > (PI/180*10))? 0: 0.2;
    turnSpeed = newYaw - myYaw;
  }
  else
  { // No one in range
    forwardSpeed = 0;
    turnSpeed = 0;
  }
}

void doAggregate(
  double &forwardSpeed, double &turnSpeed,
  double myX, double myY, double myYaw,
  RobotList &robotList,
  double rangeLimit)
{
  double centroidX = 0;
  double centroidY = 0;

  double nearestDistance = findNearestDistance(robotList, myX, myY);

  // DEBUG
  // std::cout << "near:" << nearestDistance << std::endl;
  // std::cout << "RangeLimit: " << rangeLimit << std::endl;

  if (nearestDistance > 0 && nearestDistance > rangeLimit)
  { // Some body in range
    calculateCentroid(robotList, myX, myY, centroidX, centroidY);
    double targetX = centroidX - myX;
    double targetY = centroidY - myY;
    double newYaw = std::atan(targetY / targetX);

    if (targetX < 0) // transform the coordination from C to Player
    {
      newYaw = PI + newYaw;
    }

    // Normalize the Yaw values, fit in range [-PI, PI]
    if (myYaw > PI) {myYaw = myYaw - 2*PI;}
    if (newYaw > PI) {newYaw = newYaw - 2*PI;}

    // DEBUG
    // char msg[100];
    // sprintf(msg, "MyPos(%.2f, %.2f)", myX, myY);
    // std::cout << msg << "; ";
    // sprintf(msg, "Centroid(%.2f, %.2f)", centroidX, centroidY);
    // std::cout << msg << "; ";
    // sprintf(msg, "Target(%.2f, %.2f)", targetX, targetY);
    // std::cout << msg << "; ";
    // sprintf(msg, "MyYaw(%.2f)", PlayerCc::rtod(myYaw));
    // std::cout << msg << "; ";
    // sprintf(msg, "NewYaw(%.2f)", PlayerCc::rtod(newYaw));
    // std::cout << msg << std::endl;

    // Determine speed & turn rate
    forwardSpeed = (std::fabs(newYaw - myYaw) > (PI/180*10))? 0: 0.2;
    turnSpeed = newYaw - myYaw;
  }
  else
  { // No one in range
    forwardSpeed = 0;
    turnSpeed = 0;
  }
}

// *********************************
// Implementation of class RobotList
// *********************************
RobotList::RobotList() {this->mSize = MAX_ROBOT_NUMBER;}

int RobotList::getSize(){return MAX_ROBOT_NUMBER;}

RobotInfo* RobotList::getInfo(int index)
{
  if (index < this->mSize) { return &(this->mList[index]); }
  return NULL;
}

void RobotList::updateInfo(const RobotInfo &info)
{
   // Update position
  int i;
  for (i = 0; i < MAX_ROBOT_NUMBER; i ++)
  {
    RobotInfo *pInfo = &(this->mList[i]);
    if (pInfo->robotID == info.robotID)
    { // Update position
      pInfo->x = info.x;
      pInfo->y = info.y;
      break;
    }
  }

  if (i < MAX_ROBOT_NUMBER) { return; }
  // Add position
  for (i = 0; i < MAX_ROBOT_NUMBER; i++)
  {
    RobotInfo *pInfo = &(this->mList[i]);
    if (pInfo->robotID < 0)
    { // Add a new robot
      pInfo->robotID = info.robotID;
      pInfo->x = info.x;
      pInfo->y = info.y;
      break;
    }
  }
}

void RobotList::deleteInfo(const int robotID)
{
  for (int i = 0; i < MAX_ROBOT_NUMBER; i++)
  {
    RobotInfo *pInfo = &(this->mList[i]);
    if (pInfo->robotID == robotID)
    {
      pInfo->robotID = -1;
      break;
    }
  }
}
