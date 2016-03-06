#include <libplayerc++/playerc++.h>
#include <iostream>
#include <set>

#include "args.h"
#include "RobotBehavior.h"
#include "RobotCommunication.h"
#include "SocketConnection.h"
#include "RobotFormation.h"

#define DEBUG_ENABLED

int main(int argc, char **argv) {
  RobotSetting robotSetting;
  parse_args(argc, argv, robotSetting);
  int myID = robotSetting.robotPort;
  int leaderID = 6665; // Simplify the code though urgly
  bool isLeader = (myID == leaderID);

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
    bool hasTask = false;
    RobotBehavior* robotBehavior = isLeader? (RobotBehavior*)(new RobotBehaviorLeader): (RobotBehavior*)(new RobotBehaviorFollower);
    while (!exitLoop) {
      robot.Read();
      RobotPosition myPosition(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());

      // Broadcast my position
      robotCommunication.sendMessagePosition(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, myID, myPosition.getX(), myPosition.getY());

      // Message loop
      RobotCommunication::Message message;
      while (robotCommunication.listenMessage(listenSocket, message)) {
        // Process position report
        if (robotCommunication.waitForMessage(message, RobotCommunication::MSG_TYPE_POSITION) && message.getSenderID() != myID) {
          RobotPosition reporterPosition(message.getX(), message.getY(), 0);
          robotList[message.getSenderID()] = reporterPosition;
          continue;
        } 

        // Process exit command
        if (robotCommunication.waitForCommand(message, RobotCommunication::CMD_EXIT)) {
          exitLoop = true;
          continue;
        }

        // Process task message
        if (robotCommunication.waitForMessage(message, RobotCommunication::MSG_TYPE_TASK)) {
          // Next task
          char formationType = message.getFormationType();
          double wayPointX = message.getX();
          double wayPointY = message.getY();
          
          // DEBUG
          #ifdef DEBUG_ENABLED
            std::cout << "RcvTask(" << formationType << ", " << wayPointX << ", " << wayPointY << ")" << std::endl;
          #endif

          RobotFormation formation;
          if (formationType == RobotFormation::TYPE_LINE) {
            formation.setLine();
          } else if (formationType == RobotFormation::TYPE_DIAMOND) {
            formation.setDiamond();
          }

          if (isLeader) {
            ((RobotBehaviorLeader *)robotBehavior) -> setTarget(wayPointX, wayPointY);
          } else {
            // Evaluate every robot's suitability to fit in each of positions
            RobotBehaviorFollower * followerBehavior = (RobotBehaviorFollower *)robotBehavior;
            std::set<int> robotsTaskAssigned;
            RobotFormation::Coordination* shapePoints = formation.getShape();
            for (int i = 0; i < RobotFormation::TEAM_SIZE; i ++) {
              // Leader is pre-specified, we dont need to go through the evaluation
              if ((shapePoints + i)->x == 0 && (shapePoints+i)->y == 0)
                continue;

              // Iterate through all robots, find the one nearest to target
              RobotPosition targetPosition(wayPointX + (shapePoints + i)->x, wayPointY + (shapePoints + i)->y, 0);
              double minEstimation = followerBehavior->taskEstimate(myPosition, targetPosition);
              int selectedRobot = myID;
              for (RobotConstIterator it = robotList.begin(); it != robotList.end(); it ++) {
                // Skip any robots with task assigned 
                if (robotsTaskAssigned.find(it->first) != robotsTaskAssigned.end())
                  continue;

                double taskEstimation = followerBehavior->taskEstimate(it->second, targetPosition);
                if (taskEstimation < minEstimation){
                  minEstimation = taskEstimation;
                  selectedRobot = it->first;
                }
              }

              // I'm the winner?
              if (selectedRobot == myID) {
                hasTask = true;
                followerBehavior -> setCoordination(*(shapePoints + i));
                break;
              }

              robotsTaskAssigned.insert(selectedRobot);
            } // end of evaluation
          } // end of follower

          continue;
        } // end of task message

      } // end of message loop


      if (hasTask) {
        // Behaviors go here!
        MotorData motorData;
        if (isLeader) {
          RobotBehaviorLeader* leaderBehavior = (RobotBehaviorLeader *)robotBehavior;
          motorData = leaderBehavior->gotoTarget(myPosition);
          motorData = motorData + leaderBehavior->avoidRobot(myPosition, robotList);
          if (motorData.getMagnitude() == 0) {
            // Arrive the way point, current task done!
            robotCommunication.sendMessageTaskDone(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, myID);
            hasTask = 0;
            #ifdef DEBUG_ENABLED
              std::cout << "Task done!" << std::endl;
            #endif
          }
        } else {
          RobotBehaviorFollower* followerBehavior = (RobotBehaviorFollower *)robotBehavior;
          auto leaderIterator = robotList.find(leaderID);
          if (leaderIterator != robotList.end()) {
            motorData = followerBehavior->follow(myPosition, leaderIterator->second);
            motorData = motorData + followerBehavior->avoidRobot(myPosition, robotList);
          } else {
            // No leader found, this should not happen!
            std::cout << "Error: Leader not found!" << std::endl;
            motorData.setDirection(0);
            motorData.setMagnitude(0);
            motorData.setWeight(1);
          }
        }

        motorData.optimizeDirection();
        pp.SetSpeed(motorData.getMagnitude(), motorData.getDirection());
      }

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

