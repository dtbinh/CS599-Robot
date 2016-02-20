#include <libplayerc++/playerc++.h>
#include <math.h>
#include <iostream>
#include "behavior.h"

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

void sendMessageRegister(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID)
{
  RobotMessage message(MSG_TYPE_REGISTER, robotID);
  socket.send(message.toString(), remoteAddress, listenPort);
}

void sendMessageRequestRegister(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID)
{
  RobotMessage message(MSG_TYPE_REQUESTREGISTER, robotID);
  socket.send(message.toString(), remoteAddress, listenPort);
}

bool waitForMessage(RobotMessage &message, char expectedType) {
  return (message.getType() == expectedType);
}

bool waitForCommand(RobotMessage &message, char expectedCommand)
{
  return ((message.getType() == MSG_TYPE_COMMAND) && (message.getCommand() == expectedCommand));
}

void changeState(int& currentState, int nextState) {
  currentState = nextState;
}

void wander(double &forwardSpeed, double &turnSpeed) {
  int maxSpeed = 1;
  int maxTurn = 45;
  double fspeed, tspeed, ratio;
  
  //fspeed is between 0 and 10
  fspeed = rand()%11;

  //(fspeed/10) is between 0 and 1
  fspeed = (fspeed/10);
  fspeed = fspeed * maxSpeed;
  tspeed = rand()%(2*maxTurn);
  tspeed = tspeed-maxTurn;

  //tspeed is between -maxTurn and +maxTurn
  forwardSpeed = fspeed*0.6;
  turnSpeed = tspeed;
}

void voidObstacles(double &forwardSpeed, double &turnSpeed, PlayerCc::RangerProxy &rangerProxy)
{
  double minDistanceFront = 0.8;
  double minDistanceSide = 0.5;

  
  int minDegree = 60;
  int maxDegree = 120;
  double minFront = rangerProxy[minDegree];
  for (int i = minDegree; i < maxDegree; i ++)
  {
    if (minFront > rangerProxy[i])
      minFront = rangerProxy[i];
  }

  minDegree = 0;
  maxDegree = 60;
  double minLeft = rangerProxy[minDegree];
  for (int i = minDegree; i < maxDegree; i ++)
  {
    if (minLeft > rangerProxy[i])
      minLeft = rangerProxy[i];
  }

  minDegree = 120;
  maxDegree = 180;
  double minRight = rangerProxy[minDegree];
  for (int i = minDegree; i < maxDegree; i ++)
  {
    if (minRight > rangerProxy[i])
      minRight = rangerProxy[i];
  }

  // avoid
  if (minFront <= minDistanceFront)
  {
    forwardSpeed = 0;
    if ((minLeft <= minDistanceSide) && (minRight <= minDistanceSide)) {
      // obstacle at both sides
      turnSpeed = 180;
    }

    if ((minLeft <= minDistanceSide) && (minRight > minDistanceSide)) {
      // obstacle at left
      turnSpeed = 60;
    }

    if ((minLeft > minDistanceSide) && (minRight <= minDistanceSide)) {
      // obstacle at right
      turnSpeed = -60;
    }

    if ((minLeft > minDistanceSide) && (minRight > minDistanceSide)) {
      // no obstacle at either side
      turnSpeed = (minLeft > minRight)? -90: 90;
    }

  } else {
    if ((minLeft <= minDistanceSide) && (minRight <= minDistanceSide)) {
      turnSpeed = (minLeft < minRight)? 10: -10;
      forwardSpeed = forwardSpeed * 0.5;
    }

    if ((minLeft <= minDistanceSide) && (minRight > minDistanceSide)) {
      turnSpeed = 20;
      forwardSpeed = forwardSpeed * 0.5;
    }

    if ((minLeft > minDistanceSide) && (minRight <= minDistanceSide)) {
      turnSpeed = -20;
      forwardSpeed = forwardSpeed * 0.5;
    }

    if ((minLeft > minDistanceSide) && (minRight > minDistanceSide)) {
    }
  }


}
