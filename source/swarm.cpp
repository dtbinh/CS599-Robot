#include <libplayerc++/playerc++.h>
#include <iostream>

#include "args.h"
#include "behavior.h"
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

#define MAX_ROBOT_NUMBER 10
class RobotInfo
{
public:
  double x = 0;
  double y = 0;
  int robotID = -1;
};

int main(int argc, char **argv)
{
  RobotSetting robotSetting;

  parse_args(argc, argv, robotSetting);

  try
  {
    PlayerCc::PlayerClient robot(robotSetting.robotAddress, robotSetting.robotPort);
    PlayerCc::Position2dProxy pp(&robot, 0);
    int robotID = robotSetting.robotPort;
    RobotInfo robotList[MAX_ROBOT_NUMBER]; //TODO: user hash map

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
      // Listen to position report, record position of other robots
      RobotMessage message;
      while (listenMessage(listenSocket, message))
      { // Message Loop
        if (!waitForMessage(message, MSG_TYPE_POSITION) || message.getSenderID() == robotID)
        { // not a position message, or message sent from myself
          continue;
        }

        // Add/Update position
        for (int i = 0; i < MAX_ROBOT_NUMBER; i ++)
        {
          if (robotList[i].robotID == message.getSenderID())
          { // Update position
            robotList[i].x = message.getX();
            robotList[i].y = message.getY();
            break;
          }

          if (robotList[i].robotID < 0)
          { // Add a new robot
            robotList[i].robotID = message.getSenderID();
            robotList[i].x = message.getX();
            robotList[i].y = message.getY();
            break;
          }
        }
      }

      // Read robot status
      robot.Read();

      // Get current position
      double myX = pp.GetXPos();
      double myY = pp.GetYPos();

      // Broadcast my position
      sendMessagePosition(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, robotID, myX, myY);

      // pp.SetSpeed(newspeed, PlayerCc::dtor(newturnrate));

      // DEBUG
      // for (int i = 0; i < MAX_ROBOT_NUMBER; i ++)
      // {
      //   RobotInfo *info = &(robotList[i]);
      //   if (info->robotID < 0) break;
      //   char msg[60];
      //   sprintf(msg, "%d(%.2f,%.2f)", info->robotID, info->x, info->y);
      //   std::cout << msg << " " << std::flush;
      // }

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

