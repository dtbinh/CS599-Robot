#include <libplayerc++/playerc++.h>
#include <iostream>

#include "args.h"
#include "behavior.h"
#include "RobotMessage.h"
#include "RobotSocketConnection.h"
#include "RobotTimer.h"

int main(int argc, char **argv)
{
  int currentState = ROBOT_STATE_INIT;
  RobotTimer timer(20);
  RobotSetting robotSetting;

  parse_args(argc, argv, robotSetting);
  bool isLeader = (robotSetting.role == ROBOT_ROLE_LEADER);
  bool registerDone = false;
  int robotNumber = 0;

  // we throw exceptions on creation if we fail
  try
  {
    PlayerCc::PlayerClient robot(robotSetting.hostName, robotSetting.port);
    PlayerCc::Position2dProxy pp(&robot, 0);
    PlayerCc::RangerProxy laserProxy(&robot, 0);
    int robotID = robotSetting.port;

    srand(time(NULL));
    pp.SetMotorEnable (true);

    RobotSocket listenSocket, sendSocket;
    listenSocket.createListen(robotSetting.listenPort);
    sendSocket.createSend();

    // wait for operator input
    if (isLeader)
    {
      std::cout << "Command? ";
      char input;
      while ((input = getchar()) != 's') {}

      // Send start command
      sendCommand(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID, CMD_START);
      std::cout << "All robots start walking." << std::endl;

      sendMessageRequestRegister(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID);

      // start the stopwatch
      timer.start();
    }

    int reportNumber = 0;
    bool exitLoop = false;
    while (!exitLoop)
    {
      double newspeed = 0;
      double newturnrate = 0;

      // message processing
      RobotMessage message;
      listenMessage(listenSocket, message);

      if (isLeader) {
        if (waitForMessage(message, MSG_TYPE_REGISTER)) {
          int senderID = message.getSenderID();
          robotNumber ++;
        }

       if (waitForMessage(message, MSG_TYPE_POSITION)) {
          char str[60];
          sprintf(str, "%d(%.2f,%.2f)", message.getSenderID(), message.getX(), message.getY());
          std::cout << str << "  ";
          
          reportNumber ++;
          if (reportNumber == robotNumber) {
            std::cout << std::endl;
            sendCommand(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID, CMD_RESUME);
          }
        }
      }

      if (!registerDone && waitForMessage(message, MSG_TYPE_REQUESTREGISTER)) {
        sendMessageRegister(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID);
        registerDone = true;
      }

      switch (currentState)
      {
        case ROBOT_STATE_INIT:
         if (waitForCommand(message, CMD_START)) {
            changeState(currentState, ROBOT_STATE_RUNNING);
            break;
          }
          break;

        case ROBOT_STATE_RUNNING:
          if (waitForCommand(message, CMD_STOP))
          {
            pp.SetSpeed(0, 0);
            changeState(currentState, ROBOT_STATE_PAUSE);
            robot.Read();
            sendMessagePosition(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID, pp.GetXPos(), pp.GetYPos());
            break;
          }
          robot.Read();
          wander(newspeed, newturnrate);
          voidObstacles(newspeed, newturnrate, laserProxy);
          pp.SetSpeed(newspeed, PlayerCc::dtor(newturnrate));

          if (isLeader && timer.isTimeout()) {
            timer.stop();
            std::cout << "20 seconds passed. Current robot locations are:" << std::endl;
            sendCommand(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID, CMD_STOP);
          }
          break;

        case ROBOT_STATE_PAUSE: // wait for resume
          if (waitForCommand(message, CMD_RESUME)) {
            changeState(currentState, ROBOT_STATE_RUNNING2);
            if (isLeader) {
              timer.start();
              std::cout << "All robots resume walking." << std::endl;
            }
            break;
          }
          // pp.SetSpeed(0, 0);
          break;

        case ROBOT_STATE_RUNNING2:
          if (waitForCommand(message, CMD_EXIT)) {
            pp.SetSpeed(0, 0);
            changeState(currentState, ROBOT_STATE_EXIT);
            break;
          }
          robot.Read();
          wander(newspeed, newturnrate);
          voidObstacles(newspeed, newturnrate, laserProxy);
          pp.SetSpeed(newspeed, PlayerCc::dtor(newturnrate));

          if (isLeader && timer.isTimeout()) {
            timer.stop();
            std::cout << "20 seconds passed. Mission accomplished" << std::endl;
            sendCommand(sendSocket, robotSetting.remoteAddress, robotSetting.listenPort, robotID, CMD_EXIT);
          }
          break;

        case ROBOT_STATE_EXIT: // exit loop
          // pp.SetSpeed(0, 0);
          exitLoop = true;
          break;

        default:
          break;
      }

      sleep(1);
    }
  }
  catch (PlayerCc::PlayerError & e)
  {
    std::cerr << e << std::endl;
    return -1;
  } catch (RobotSocketException & e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }
}

