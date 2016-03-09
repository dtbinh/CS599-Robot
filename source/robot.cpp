#include <libplayerc++/playerc++.h>
#include <iostream>
#include <set>

#include "args.h"
#include "RobotBehavior.h"
#include "RobotCommunication.h"
#include "SocketConnection.h"
#include "RobotFormation.h"

// #define DEBUG_ENABLED

using namespace Robot;

int main(int argc, char **argv) {
  RobotSetting robotSetting;
  parse_args(argc, argv, robotSetting);
  int myID = robotSetting.robotPort;
  int leaderID = 6665; // Simplify the code though urgly
  bool isLeader = (myID == leaderID);

  PoseList robotList;
  try {
    // Initialize
    PlayerCc::PlayerClient robot(robotSetting.robotAddress, robotSetting.robotPort);
    PlayerCc::Position2dProxy pp(&robot, 0);
    pp.SetMotorEnable (true);
    Communication robotCommunication(robotSetting.broadcastAddress, robotSetting.broadcastPort);

    // Main Loop
    bool exitLoop = false;
    Behavior* robotBehavior = isLeader?
      (Behavior*)(new BehaviorLeader):
      (Behavior*)(new BehaviorFollower(myID, leaderID));

    while (!exitLoop) {
      robot.Read();
      Pose myPose(pp.GetXPos(), pp.GetYPos(), pp.GetYaw());

      // Broadcast my position
      robotCommunication.sendMessagePosition(myID, myPose.x, myPose.y);

      // Message loop
      Message message;
      while (robotCommunication.listenMessage(message)) {
        // Process position report
        if (robotCommunication.waitForMessage(message, MSG_TYPE_POSITION) && message.getSenderID() != myID) {
          robotList[message.getSenderID()] = Pose(message.getX(), message.getY(), 0);
          continue;
        } 

        // Process exit command
        if (robotCommunication.waitForCommand(message, CMD_EXIT)) {
          exitLoop = true;
          continue;
        }

        // Process task message
        if (robotCommunication.waitForMessage(message, MSG_TYPE_TASK)) {
          // Next task
          char formationType = message.getFormationType();
          Pose wayPoint(message.getX(), message.getY(), 0);
          Formation formation;
          switch (formationType) {
            case Formation::TYPE_LINE:
              formation.setLine();
              break;
            case Formation::TYPE_DIAMOND:
              formation.setDiamond();
              break;
            default:
              formation.setLine();
              break;            
          }

          if (isLeader) {
            ((BehaviorLeader *)robotBehavior) -> assignTask(wayPoint);
          } else {
            BehaviorFollower * followerBehavior = (BehaviorFollower *)robotBehavior;
            // Local algorithm
            Pose refPose = robotList.find(leaderID)->second; // Leader as reference position

            // Global algorithm
            // Pose refPose = wayPoint; // Way point as reference position

            Pose myTask = followerBehavior->bid(formation, refPose, robotList, myPose);
            followerBehavior->assignTask(myTask);
          } // end of follower
          continue;
        } // end of task message

      } // end of message loop


      // Behaviors go here!
      WVelocity wvel;
      if (isLeader) {
        BehaviorLeader* leaderBehavior = (BehaviorLeader *)robotBehavior;
        wvel = WVelocity(leaderBehavior->gotoTarget(myPose), LEADER_TASK_WEIGHT);
        Velocity avoidVel;
        if (leaderBehavior->avoidRobot(myPose, robotList, avoidVel)) {
          wvel = wvel + WVelocity(avoidVel, AVOID_WEIGHT);
        }
        if (wvel == Velocity(0,0,0) && leaderBehavior->hasTask()) {
          // Arrive the way point, current task done!
          leaderBehavior->setHasTask(false);
          robotCommunication.sendMessageTaskDone(myID);
        }
      } else {
        BehaviorFollower* followerBehavior = (BehaviorFollower *)robotBehavior;
        auto leaderIterator = robotList.find(leaderID);
        if (leaderIterator != robotList.end()) {
          wvel = WVelocity(followerBehavior->follow(myPose, leaderIterator->second), FOLLOWER_TASK_WEIGHT);
          Velocity avoidVel;
          if (followerBehavior->avoidRobot(myPose, robotList, avoidVel)) {
            wvel = wvel + WVelocity(avoidVel, AVOID_WEIGHT);
          }
        } else {
          // No leader found, happens at the first iteration (due to message delay)!
          // std::cout << "Error: Leader not found!" << std::endl;
        }
      }

      pp.SetSpeed(wvel.x, wvel.a);

      // sleep(1);
    }
  }
  catch (PlayerCc::PlayerError & e) {
    std::cerr << e << std::endl;
    return -1;
  } catch (SocketException & e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "Robot(" << myID << ") exit!" << std::endl;
}

