#include <cmath>
#include <iostream>
#include "RobotBehavior.h"

#define DEBUG_ENABLED

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
	if (this->mDirection > PI) {
		this->mDirection = this->mDirection - 2 * PI; // turn left over 180 = turn right
	} else if (this->mDirection < -PI) {
		this->mDirection = this->mDirection + 2 * PI; // turn right over 180 = turn left
	}
	this->mDirection = fmod(this->mDirection, 2 * PI); // make sure direction < 360

	return *this;
}

MotorData& MotorData::convertToTurn(double currentDirection) {
  this->setDirection(this->getDirection() - currentDirection);
  return *this;
}

/*
* Implementation of RobotBehavior
*/
RobotBehavior::RobotBehavior() {}

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
      double directionToRobot = myPosition.getDirectionTo(robotPosition) - fmod(myPosition.getYaw() + 2 * PI, 2 * PI);
      if (directionToRobot <= (PI / 2) || directionToRobot >= (3 * PI / 2)) {
        nearestRobotToAvoid = robotPosition;
        smallestDistance = distanceToRobot;
      }
    }
  }

  MotorData motorData(0, 0, 0);
  if (smallestDistance < AVOID_DISTANCE) {
    double directionToRobotAvoid = myPosition.getDirectionTo(nearestRobotToAvoid) - fmod(myPosition.getYaw() + 2 * PI, 2 * PI);
    double extraAngle = 5 * PI / 190;
    if (directionToRobotAvoid >= 0 && directionToRobotAvoid <= (PI / 2)) {
      // Robot is in my left, turn right
      // motorData.setDirection(directionToRobotAvoid - (PI / 2) - extraAngle);
      motorData.setDirection( - (PI / 2));
      motorData.optimizeDirection();
      // motorData.setMagnitude(AVOID_SPEED * (smallestDistance/AVOID_DISTANCE));
      motorData.setMagnitude(AVOID_SPEED);
      motorData.setWeight(AVOID_WEIGHT);
    } else if (directionToRobotAvoid >= -(PI / 2) && directionToRobotAvoid <= 0) {
      // Robot is in my right, turn left
      // motorData.setDirection(directionToRobotAvoid + (PI / 2) + extraAngle);
      motorData.setDirection(PI / 2);
      motorData.optimizeDirection();
      // motorData.setMagnitude(AVOID_SPEED * (smallestDistance/AVOID_DISTANCE));
      motorData.setMagnitude(AVOID_SPEED);
      motorData.setWeight(AVOID_WEIGHT);
    }
  }

	return motorData;
}

/*
** Implementation of RobotBehaviorLeader
*/
RobotBehaviorLeader::RobotBehaviorLeader(): RobotBehavior() { }

void RobotBehaviorLeader::setTarget(double x, double y) {
  mTargetX = x;
  mTargetY = y;

  #ifdef DEBUG_ENABLED
    std::cout << "WayPoint(" << x << ", " << y << ")" << std::endl;
  #endif
}

MotorData RobotBehaviorLeader::gotoTarget(RobotPosition& myPosition) {
  MotorData motorData(0, 0, TASK_WEIGHT);

  // Turn rate
  RobotPosition targetPosition(mTargetX, mTargetY, 0);
  motorData.setDirection(myPosition.getDirectionTo(targetPosition) - myPosition.getYaw());
  motorData.optimizeDirection();

  // Speed
  double distance = myPosition.getDistanceTo(targetPosition);
  double magnitude = (distance < NEAR_DISTANCE)? (MAX_MOTOR_SPEED * (distance / NEAR_DISTANCE)): MAX_MOTOR_SPEED;
  if (distance < FINISH_DISTANCE)
    magnitude = 0;
  motorData.setMagnitude(magnitude);

  #ifdef DEBUG_ENABLED
    std::cout << "Distance(" << distance << ") ";
    std::cout << "MotorData(" << motorData.getMagnitude() << ", " << motorData.getDirection() * 180 / PI << ")";
    std::cout << std::endl;
  #endif

  return motorData;
}

/*
** Implementation of RobotBehaviorFollower
*/
RobotBehaviorFollower::RobotBehaviorFollower(): RobotBehavior() { }

void RobotBehaviorFollower::setCoordination(RobotFormation::Coordination coordination) {
  mCoordination = coordination;
  #ifdef DEBUG_ENABLED
    std::cout << "TeamPose(" << coordination.x << ", " << coordination.y << ")" << std::endl;
  #endif
}

double RobotBehaviorFollower::taskEstimate(RobotPosition myPosition, RobotPosition targetPosition) {
  return myPosition.getDistanceTo(targetPosition);
}

MotorData RobotBehaviorFollower::follow(RobotPosition& myPosition, RobotPosition& leaderPosition) {
  MotorData motorData(0, 0, TASK_WEIGHT);

  // Turn rate
  RobotPosition nextPosition(leaderPosition.getX() + mCoordination.x, leaderPosition.getYaw() + mCoordination.y, 0);
  motorData.setDirection(myPosition.getDirectionTo(nextPosition) - myPosition.getYaw());
  motorData.optimizeDirection();

  // Speed
  double distance = myPosition.getDistanceTo(nextPosition);
  double magnitude = MAX_MOTOR_SPEED;
  if (distance < FINISH_DISTANCE)
    magnitude = 0;
  motorData.setMagnitude(magnitude);

  #ifdef DEBUG_ENABLED
    std::cout << "Distance(" << distance << ") ";
    std::cout << "MotorData(" << motorData.getMagnitude() << ", " << motorData.getDirection() * 180 / PI << ")";
    std::cout << std::endl;
  #endif

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
	return sqrt(x * x + y * y);
}

double RobotPosition::getDirectionTo(const RobotPosition robotPosition) const {
    double targetX = robotPosition.getX() - this->getX();
    double targetY = robotPosition.getY() - this->getY();

    if (targetX == 0 && targetY > 0) {
      return PI / 2;
    } else if (targetX == 0 && targetY < 0) {
      return 3 * PI / 2;
    } else if (targetX == 0 && targetY == 0) {
      return 0;
    } else {
      double direction = atan(targetY / targetX);
      if (targetX < 0) // transform the coordination from C to Player
        direction = PI + direction;
      else
        direction = fmod((2 * PI + direction), (2 * PI));

      return direction;
    }
}
