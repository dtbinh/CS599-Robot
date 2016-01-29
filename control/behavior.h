#ifndef __BEHAVIOR_H__
#define __BEHAVIOR_H__

#include <libplayerc++/playerc++.h>
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

#define ROBOT_STATE_INIT 0
#define ROBOT_STATE_RUNNING 1
#define ROBOT_STATE_RUNNING2 2
#define ROBOT_STATE_PAUSE 3
#define ROBOT_STATE_EXIT 4

void wander(double &forwardSpeed, double &turnSpeed);
void voidObstacles(double &forwardSpeed, double &turnSpeed, PlayerCc::RangerProxy &rangerProxy);

void sendCommand(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID, int command);
void sendMessagePosition(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID, double x, double y);
void sendMessageRegister(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID);
void sendMessageRequestRegister(RobotSocket &socket, std::string remoteAddress, int listenPort, int robotID);

bool listenMessage(RobotSocket &socket, RobotMessage& message);
bool waitForCommand(RobotMessage &message, char expectedCommand);
bool waitForMessage(RobotMessage &message, char expectedType);
void changeState(int& currentState, int nextState);

#endif