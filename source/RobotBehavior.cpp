#include <cmath>
#include <iostream>
#include "RobotBehavior.h"

// #define DEBUG_ENABLED

const double PI = 3.1415926535;

/*
* Implementation of MotorData
*/
MotorData::MotorData(): mMagnitude(0), mDirection(0), mWeight(0) { }
MotorData::MotorData(const MotorData& data): mMagnitude(data.getMagnitude()), mDirection(data.getDirection()), mWeight(data.getWeight()) { }
MotorData::MotorData(const double magnitude, const double direction): mMagnitude(magnitude), mDirection(direction), mWeight(0) { }
MotorData::MotorData(const double magnitude, const double direction, const int weight): mMagnitude(magnitude), mDirection(direction), mWeight(weight) { }

double MotorData::getMagnitude() const {
	return this->mMagnitude;
}

MotorData& MotorData::setMagnitude(double magnitude) {
	this->mMagnitude = magnitude;
	return *this;
}

double MotorData::getDirection() const {
	return this->mDirection;
}

MotorData& MotorData::setDirection(double direction) {
	this->mDirection = direction;
	return *this;
}

int MotorData::getWeight() const {
  return this->mWeight;
}

MotorData& MotorData::setWeight(int weight) {
  this->mWeight = weight;
  return *this;
}

MotorData& MotorData::operator =(const MotorData& data) {
	this->setMagnitude(data.getMagnitude()).setDirection(data.getDirection()).setWeight(data.getWeight());
	return *this;
}

MotorData MotorData::operator+ (const MotorData& data) {
  int totalWeight = this->getWeight() + data.getWeight();
  double magnitude = (this->getMagnitude() * this->getWeight() / totalWeight) + (data.getMagnitude() * data.getWeight() / totalWeight);
  double direction = (this->getDirection() * this->getWeight() / totalWeight) + (data.getDirection() * data.getWeight() / totalWeight);

	return MotorData(magnitude, direction, totalWeight);
}

MotorData& MotorData::optimizeDirection() {
  double a = this->mDirection;
  while(a < -PI) a += 2.0*PI;
  while(a > PI) a -= 2.0*PI;
  this->mDirection = a;

	// if (this->mDirection > PI) {
	// 	this->mDirection = this->mDirection - 2 * PI; // turn left over 180 = turn right
	// } else if (this->mDirection < -PI) {
	// 	this->mDirection = this->mDirection + 2 * PI; // turn right over 180 = turn left
	// }
	// this->mDirection = fmod(this->mDirection, 2 * PI); // make sure direction < 360

	return *this;
}

MotorData& MotorData::convertToTurn(double currentDirection) {
  this->setDirection(this->getDirection() - currentDirection);
  return *this;
}

/*
* Implementation of RobotBehavior
*/

RobotBehavior::RobotBehavior(): mHasTask(false) { }

bool RobotBehavior::hasTask() {
  return mHasTask;
}

void RobotBehavior::setHasTask(bool hasTask) {
  mHasTask = hasTask;
}

MotorData RobotBehavior::avoidRobot(RobotPosition myPosition, const RobotList &robotList) {
  RobotPosition nearestRobotToAvoid;
  double smallestDistance = AVOID_DISTANCE + 1;

  // Iterate all robots in sense range, find out the nearest one to the front/side of me
  for (RobotConstIterator it = robotList.begin(); it != robotList.end(); it ++){
    RobotPosition robotPosition = it->second;
    double distanceToRobot = myPosition.getDistanceTo(robotPosition);
    
    if (distanceToRobot > AVOID_DISTANCE)
      continue;

    if (distanceToRobot < smallestDistance) {
      double directionToRobot = normalizeAngle(myPosition.getDirectionTo(robotPosition) - myPosition.getYaw());
      if (directionToRobot <= (PI / 2) && directionToRobot >= -(PI / 2)) {
        nearestRobotToAvoid = robotPosition;
        smallestDistance = distanceToRobot;
      }
    }
  }

  MotorData motorData(0, 0, 0);
  double calcSpeed = 0;
  double calcAngle = 0;
  if (smallestDistance < AVOID_DISTANCE) {
    double avoidAngle = normalizeAngle(myPosition.getDirectionTo(nearestRobotToAvoid) - myPosition.getYaw());
    if (smallestDistance > AVOID_DISTANCE_DANGER) {
      calcSpeed = std::min(AVOID_SPEED, smallestDistance/2);
    }
    calcAngle = std::min(-2 * avoidAngle, MAX_ANGLE_SPEED);
    calcAngle = std::max(-2 * avoidAngle, -MAX_ANGLE_SPEED);
    motorData.setDirection(calcAngle);
    motorData.setMagnitude(calcSpeed);
    motorData.setWeight(AVOID_WEIGHT);
  }

	return motorData;
}

double RobotBehavior::normalizeAngle(double a) {
  while(a < -PI) a += 2.0*PI;
  while(a > PI) a -= 2.0*PI;  
  return a;
}

MotorData RobotBehavior::diffGoTo(RobotPosition goalPose, RobotPosition currPose, double maxSpeedX, double maxSpeedAngle) {
  Velocity goal, est_pose;
  goal.x = goalPose.getX();
  goal.y = goalPose.getY();
  goal.a = goalPose.getYaw();
  est_pose.x = currPose.getX();
  est_pose.y = currPose.getY();
  est_pose.a = currPose.getYaw();


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

    // if we're pointing about the right direction, move
    // forward
    if( fabs(a_error) < PI/16 ) {
      calc.x = std::min( goal_distance, max_speed_x );
    }
  }

  return MotorData(calc.x, calc.a);
}

/*
** Implementation of RobotBehaviorLeader
*/
RobotBehaviorLeader::RobotBehaviorLeader(): RobotBehavior() { }

void RobotBehaviorLeader::assignTask(double x, double y) {
  mTargetX = x;
  mTargetY = y;
  setHasTask(true);

  #ifdef DEBUG_ENABLED
    std::cout << "WayPoint(" << x << ", " << y << ")" << std::endl;
  #endif
}

MotorData RobotBehaviorLeader::gotoTarget(RobotPosition& myPosition) {
  if (!hasTask()) return MotorData(0, 0, TASK_WEIGHT);

  RobotPosition goalPose(mTargetX, mTargetY, 0);
  MotorData motorData = diffGoTo(goalPose, myPosition, MAX_MOTOR_SPEED * 0.7, 1);
  motorData.setWeight(TASK_WEIGHT);

  if (myPosition.getDistanceTo(goalPose) < FINISH_DISTANCE) {
    motorData.setMagnitude(0);
  }

  return motorData;
}

/*
** Implementation of RobotBehaviorFollower
*/
RobotBehaviorFollower::RobotBehaviorFollower(): RobotBehavior() { }

void RobotBehaviorFollower::assignTask(RobotFormation::Coordination coordination) {
  mCoordination = coordination;
  setHasTask(true);

  #ifdef DEBUG_ENABLED
    std::cout << "TeamPose(" << coordination.x << ", " << coordination.y << ")" << std::endl;
  #endif
}

double RobotBehaviorFollower::taskEstimate(RobotPosition myPosition, RobotPosition targetPosition) {
  return myPosition.getDistanceTo(targetPosition);
}

MotorData RobotBehaviorFollower::follow(RobotPosition& myPosition, RobotPosition& leaderPosition) {
  if (!hasTask()) return MotorData(0, 0, TASK_WEIGHT);

  RobotPosition nextPosition(leaderPosition.getX() + mCoordination.x, leaderPosition.getY() + mCoordination.y, 0);
  if (nextPosition.getX() <= myPosition.getX()) {
    nextPosition.setX(myPosition.getX());
  }
  MotorData motorData = diffGoTo(nextPosition, myPosition, MAX_MOTOR_SPEED, 1);
  motorData.setWeight(TASK_WEIGHT);

  return motorData;
}

/*
** Implementation of RobotPosition
*/
RobotPosition::RobotPosition(): mX(0), mY(0), mYaw(0) { }
RobotPosition::RobotPosition(double x, double y, double yaw): mX(x), mY(y), mYaw(yaw) { }
RobotPosition::RobotPosition(const RobotPosition& robotPosition): mX(robotPosition.getX()), mY(robotPosition.getY()), mYaw(robotPosition.getYaw()) { }

double RobotPosition::getX() const {
	return this->mX;
}

double RobotPosition::getY() const {
	return this->mY;
}

double RobotPosition::getYaw() const {
	return this->mYaw;
}

RobotPosition& RobotPosition::setX(double x) {
	this->mX = x;
	return *this;
}

RobotPosition& RobotPosition::setY(double y) {
	this->mY = y;
	return *this;
}

RobotPosition& RobotPosition::setYaw(double yaw) {
	this->mYaw = yaw;
	return *this;
}

RobotPosition& RobotPosition::operator= (const RobotPosition& position) {
	this->setX(position.getX()).setY(position.getY()).setYaw(position.getYaw());
	return *this;
}

double RobotPosition::getDistanceTo(const RobotPosition robotPosition) const {
	double x = this->getX() - robotPosition.getX();
	double y = this->getY() - robotPosition.getY();
	return hypot(x, y);
}

double RobotPosition::getDirectionTo(const RobotPosition robotPosition) const {
    double targetX = robotPosition.getX() - this->getX();
    double targetY = robotPosition.getY() - this->getY();
    if (targetX == 0 && targetY == 0) return 0;

    return atan2(targetY, targetX);

    // if (targetX == 0 && targetY > 0) {
    //   return PI / 2;
    // } else if (targetX == 0 && targetY < 0) {
    //   return 3 * PI / 2;
    // } else if (targetX == 0 && targetY == 0) {
    //   return 0;
    // } else {
    //   double direction = atan(targetY / targetX);
    //   if (targetX < 0) // transform the coordination from C to Player
    //     direction = PI + direction;
    //   else
    //     direction = fmod((2 * PI + direction), (2 * PI));

      // return direction;
    // }
}
