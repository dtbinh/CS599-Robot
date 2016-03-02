#include <libplayerc++/playerc++.h>
#include <iostream>

#include "args.h"
#include "RobotBehavior.h"
#include "RobotCommunication.h"
#include "SocketConnection.h"

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
    RobotCommunication::Communication robotCommunication;
    RobotNetwork::Socket listenSocket, sendSocket;
    listenSocket.createListen(robotSetting.broadcastPort);
    sendSocket.createSend();

    // Main Loop
    bool exitLoop = false;
    while (!exitLoop) {
      // Read robot status
      robot.Read();

      // Get current position
      RobotPosition myPosition(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());

      // Broadcast my position
      robotCommunication.sendMessagePosition(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, myID, myPosition.getX(), myPosition.getY());

      // Listen to position report, record position of other robots
      RobotCommunication::Message message;
      while (robotCommunication.listenMessage(listenSocket, message)) { // Message Loop
        // Process position report
        if (robotCommunication.waitForMessage(message, RobotCommunication::MSG_TYPE_POSITION) && message.getSenderID() != myID) {

          RobotPosition reporterPosition(message.getX(), message.getY(), 0);
          if (reporterPosition.getDistanceTo(myPosition) < robotSetting.senseRange) { // Add/Update position
            robotList[message.getSenderID()] = reporterPosition;
          } else { // Delete
            robotList.erase(message.getSenderID());
          }
        } 

        // Process exit command
        if (robotCommunication.waitForCommand(message, RobotCommunication::CMD_EXIT)) {
          exitLoop = true;
        }
      }


      // Behaviors go here!
      RobotBehavior robotBehavior;
      MotorData motorData;
      if (robotSetting.runType == RUN_TYPE_DISPERSION) {
        // Dispersion
        motorData = robotBehavior.disperse(myPosition, robotList, robotSetting.distance);
        motorData = motorData + robotBehavior.avoidRobot(myPosition, robotList);
      } else if (robotSetting.runType == RUN_TYPE_AGGREGATION) {
        // aggregation
        motorData = robotBehavior.aggregate(myPosition, robotList, robotSetting.distance);
        motorData = motorData + robotBehavior.avoidRobot(myPosition, robotList);
      }

      motorData.optimizeDirection();
      pp.SetSpeed(motorData.getMagnitude(), motorData.getDirection());
      sleep(1);
    }
  }
  catch (PlayerCc::PlayerError & e) {
    std::cerr << e << std::endl;
    return -1;
  } catch (RobotNetwork::SocketException & e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "Robot(" << myID << ") exit!" << std::endl;
}

