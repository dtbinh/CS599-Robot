#include <libplayerc++/playerc++.h>
#include <iostream>

#include "args.h"
#include "behavior.h"
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

int main(int argc, char **argv)
{
  RobotSetting robotSetting;
  parse_args(argc, argv, robotSetting);
  int myID = robotSetting.robotPort;

  RobotList robotList;
  try
  {
    PlayerCc::PlayerClient robot(robotSetting.robotAddress, robotSetting.robotPort);
    PlayerCc::Position2dProxy pp(&robot, 0);

    // Enable motor
    pp.SetMotorEnable (true);

    // Create communication
    RobotSocket listenSocket, sendSocket;
    listenSocket.createListen(robotSetting.broadcastPort);
    sendSocket.createSend();

    // Main Loop
    bool exitLoop = false;
    while (!exitLoop)
    {
      // Read robot status
      robot.Read();

      // Get current position
      double myX = pp.GetXPos();
      double myY = pp.GetYPos();
      double myYaw = pp.GetYaw();

      // Listen to position report, record position of other robots
      RobotMessage message;
      while (listenMessage(listenSocket, message))
      { // Message Loop

        // Process position report
        if (waitForMessage(message, MSG_TYPE_POSITION) && message.getSenderID() != myID)
        {
          double x = message.getX() - myX;
          double y = message.getY() - myY;
          double distance = std::sqrt(x*x + y*y);
          if (distance < robotSetting.senseRange)
          {
            // Add/Update position
            RobotInfo robotInfo;
            robotInfo.robotID = message.getSenderID();
            robotInfo.x = message.getX();
            robotInfo.y = message.getY();
            robotList.updateInfo(robotInfo);
          }
          else
          {
            robotList.deleteInfo(message.getSenderID());
          }
        }

        // Process exit command
        if (waitForCommand(message, CMD_EXIT))
        {
          exitLoop = true;
        }
      }

      // DEBUG
      // char msg[100];
      // sprintf(msg, "MyPosition(%.2f, %.2f)", myX, myY);
      // std::cout << msg << "; ";
      // for (int i = 0; i < robotList.getSize(); i ++)
      // {
      //   RobotInfo *pInfo = robotList.getInfo(i);
      //   if (pInfo->robotID > 0)
      //   {
      //     sprintf(msg, "R%d(%.2f, %.2f)", pInfo->robotID, pInfo->x, pInfo->y);
      //     std::cout << msg << "; ";
      //   }
      // }
      // std::cout << std::endl;

      // Broadcast my position
      sendMessagePosition(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, myID, myX, myY);

      // Behaviors go here!
      double forwardSpeed = 0;
      double turnSpeed = 0;
      if (robotSetting.runType == RUN_TYPE_DISPERSION)
      { // Dispersion
        doDisperse(forwardSpeed, turnSpeed, myX, myY, myYaw, robotList, robotSetting.distance);
      }
      else if (robotSetting.runType == RUN_TYPE_AGGREGATION)
      { // aggregation
        doAggregate(forwardSpeed, turnSpeed, myX, myY, myYaw, robotList, robotSetting.distance);
      }

      pp.SetSpeed(forwardSpeed, turnSpeed);
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

  std::cout << "Robot(" << myID << ") exit!" << std::endl;
}

