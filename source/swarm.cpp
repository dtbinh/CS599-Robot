#include <libplayerc++/playerc++.h>
#include <iostream>

#include "args.h"
#include "behavior.h"
#include "RobotBehavior.h"
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

int main(int argc, char **argv) {
  RobotSetting robotSetting;
  parse_args(argc, argv, robotSetting);
  int myID = robotSetting.robotPort;

  RobotList robotList;
  try {
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
    while (!exitLoop) {
      // Read robot status
      robot.Read();

      // Get current position
      RobotPosition myPosition(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());

      // Listen to position report, record position of other robots
      RobotMessage message;
      while (listenMessage(listenSocket, message)) { // Message Loop
        // Process position report
        if (waitForMessage(message, MSG_TYPE_POSITION) && message.getSenderID() != myID) {

          RobotPosition reporterPosition(message.getX(), message.getY(), 0);
          if (reporterPosition.getDistanceTo(myPosition) < robotSetting.senseRange) { // Add/Update position
            robotList[message.getSenderID()] = reporterPosition;
          } else { // Delete
            robotList.erase(message.getSenderID());
          }
        }

        // Process exit command
        if (waitForCommand(message, CMD_EXIT)) {
          exitLoop = true;
        }
      }

      // Broadcast my position
      sendMessagePosition(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, myID, myPosition.getX(), myPosition.getY());

      // Behaviors go here!
      RobotBehavior robotBehavior;
      MotorData motorData;
      if (robotSetting.runType == RUN_TYPE_DISPERSION) {
        // Dispersion
        motorData = robotBehavior.disperse(myPosition, robotList, robotSetting.distance);
      } else if (robotSetting.runType == RUN_TYPE_AGGREGATION) {
        // aggregation
        motorData = robotBehavior.aggregate(myPosition, robotList, robotSetting.distance);
      }

      pp.SetSpeed(motorData.getMagnitude(), motorData.getDirection());
      sleep(1);
    }
  }
  catch (PlayerCc::PlayerError & e) {
    std::cerr << e << std::endl;
    return -1;
  } catch (RobotSocketException & e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "Robot(" << myID << ") exit!" << std::endl;
}

