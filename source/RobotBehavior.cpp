#include <cmath>
#include <iostream>
#include <set>
#include "RobotBehavior.h"

// #define DEBUG_ENABLED

namespace Robot {

  Behavior::Behavior(): mHasTask(false) { }

  bool Behavior::hasTask() {
    return mHasTask;
  }

  void Behavior::setHasTask(bool hasTask) {
    mHasTask = hasTask;
  }

  bool Behavior::avoidRobot(Pose myPose, const PoseList &robotList, Velocity &vel) {
    Pose nearestRobotToAvoid;
    double smallestDistance = AVOID_DISTANCE + 1;

    // Iterate all robots in sense range, find out the nearest one to the front/side of me
    for (PoseConstIterator it = robotList.begin(); it != robotList.end(); it ++){
      Pose robotPose = it->second;
      double distanceToRobot = myPose.distance(robotPose);
      
      if (distanceToRobot > AVOID_DISTANCE)
        continue;

      if (distanceToRobot < smallestDistance) {
        double directionToRobot = normalizeAngle(myPose.angle(robotPose) - myPose.a);
        if (directionToRobot <= (PI / 2) && directionToRobot >= -(PI / 2)) {
          nearestRobotToAvoid = robotPose;
          smallestDistance = distanceToRobot;
        }
      }
    }

    if (smallestDistance < AVOID_DISTANCE) {
      double avoidAngle = normalizeAngle(myPose.angle(nearestRobotToAvoid) - myPose.a);
      if (smallestDistance > AVOID_DISTANCE_DANGER) {
        vel.x = std::min(AVOID_SPEED, smallestDistance/2);
      }
      vel.a = std::min(-2 * avoidAngle, MAX_ANGLE_SPEED);
      vel.a = std::max(-2 * avoidAngle, -MAX_ANGLE_SPEED);
      return true;
    } else {
      return false;
    }
  }

  double Behavior::normalizeAngle(double a) {
    while(a < -PI) a += 2.0*PI;
    while(a > PI) a -= 2.0*PI;  
    return a;
  }

  // This function comes from Stage code base, modified to enable customizing max speed
  // libstage/model_position.cc
  Velocity Behavior::diffGoTo(Pose goal, Pose est_pose, double maxSpeedX, double maxSpeedAngle) {

    double x_error = goal.x - est_pose.x;
    double y_error = goal.y - est_pose.y;
    double a_error = normalizeAngle( goal.a - est_pose.a );
   
    double max_speed_x = maxSpeedX;
    double max_speed_a = maxSpeedAngle;       

    Velocity calc;
    double close_enough = 0.01; // fudge factor

    // if we're at the right spot
    if (fabs(x_error) < close_enough && fabs(y_error) < close_enough) {
      // turn on the spot to minimize the error
      calc.a = std::min( a_error, max_speed_a );
      calc.a = std::max( a_error, -max_speed_a );
    } else {
      // turn to face the goal point
      double goal_angle = atan2( y_error, x_error );
      double goal_distance = hypot( y_error, x_error );

      a_error = normalizeAngle(goal_angle - est_pose.a);
      calc.a = std::min( a_error, max_speed_a );
      calc.a = std::max( a_error, -max_speed_a );

      // if we're pointing about the right direction, move forward
      if( fabs(a_error) < PI/16 ) {
        calc.x = std::min( goal_distance, max_speed_x );
      }
    }

    return calc;
  }

  /*
  ** Implementation of BehaviorLeader
  */
  BehaviorLeader::BehaviorLeader(): Behavior() { }

  void BehaviorLeader::assignTask(Pose targetPose) {
    mTargetPose = targetPose;
    setHasTask(true);
  }

  Velocity BehaviorLeader::gotoTarget(Pose& myPose) {
    if (!hasTask())
      return Velocity(0, 0, 0);
    if (myPose.distance(mTargetPose) < LEADER_FINISH_DISTANCE)
      return Velocity(0, 0, 0);

    return diffGoTo(mTargetPose, myPose, MAX_MOTOR_SPEED * 0.7, 1);
  }

  /*
  ** Implementation of BehaviorFollower
  */
  BehaviorFollower::BehaviorFollower(int myID, int leaderID): Behavior(), mMyID(myID), mLeaderID(leaderID) { }

  void BehaviorFollower::assignTask(Pose pose) {
    mCoordination = pose;
    setHasTask(true);
  }

  double BehaviorFollower::taskEstimate(Pose myPose, Pose targetPose) {
    return myPose.distance(targetPose);
  }

  Pose BehaviorFollower::bid(Formation fmt, Pose refPose, const PoseList robotList, Pose myPose) {
    std::set<int> robotsTaskAssigned;

    Pose* shapePoints = fmt.getShape();
    for (int i = 0; i < Formation::TEAM_SIZE; i ++) {
      Pose *fmtPose = shapePoints + i;
      if (i == fmt.getLeaderIndex())
        continue; // Skip the position for leader

      // Iterate through all robots, find the one with lowest estimation
      Pose targetPose(refPose.x + fmtPose->x, refPose.y + fmtPose->y, 0);
      double minEstimation = taskEstimate(myPose, targetPose);
      int selectedRobot = mMyID;
      for (PoseConstIterator it = robotList.begin(); it != robotList.end(); it ++) {
        if (it->first == mLeaderID)
          continue; // Skip leader
        if (robotsTaskAssigned.find(it->first) != robotsTaskAssigned.end())
          continue; // Skip those already assigned task

        double taskEstimation = taskEstimate(it->second, targetPose);
        if (taskEstimation < minEstimation){
          minEstimation = taskEstimation;
          selectedRobot = it->first;
        }
      }

      // I'm the winner?
      if (selectedRobot == mMyID) {
        return *fmtPose;
      }

      robotsTaskAssigned.insert(selectedRobot);
    } // end of evaluation

    return Pose(); // This should never happen
  }

  Velocity BehaviorFollower::follow(Pose& myPose, Pose& leaderPose) {
    if (!hasTask())
      return Velocity(0, 0, 0);

    Pose goalPose(leaderPose.x + mCoordination.x, leaderPose.y + mCoordination.y, 0);
    if (goalPose.x <= myPose.x) {
      return Velocity(0,0,0);
    }
    return diffGoTo(goalPose, myPose, MAX_MOTOR_SPEED, 1);
  }

} // end of namespace
