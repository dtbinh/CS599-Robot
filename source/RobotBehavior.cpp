#include <cmath>
#include <iostream>
#include "RobotBehavior.h"
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

MotorData RobotBehavior::disperse(RobotPosition myPosition, const RobotList &robotList, double rangeLimit) {

	MotorData motorData(0, 0, DISPERSE_WEIGHT);
  double nearestDistance = this->getNearestRobotDistance(robotList, myPosition);
  if (nearestDistance > 0 && nearestDistance < rangeLimit)
  { // at least one robot near me

    // I should go away from the centroid
    // Find out the centeroid position
    RobotPosition centroidPosition = this->getCentroid(robotList);

    // Find out the target direction
    double direction = centroidPosition.getDirectionTo(myPosition);

    // Determine speed & turn rate
    motorData.setDirection(direction);
    motorData.convertToTurn(myPosition.getYaw());
    motorData.optimizeDirection();
    motorData.setMagnitude(MAX_MOTOR_SPEED * (1 - fabs(motorData.getDirection()) / PI));
  }

	return motorData;
}

MotorData RobotBehavior::aggregate(RobotPosition myPosition, const RobotList &robotList, double rangeLimit) {

	MotorData motorData(0, 0, AGGREGATE_WEIGHT);
  double nearestDistance = this->getNearestRobotDistance(robotList, myPosition);
  if (nearestDistance > rangeLimit)
  { // the nearest robot is out of range

    // I should go toward the centroid
    // Find out the centeroid position
    RobotPosition centroidPosition = this->getCentroid(robotList);

    // Find out the target direction
    double direction = myPosition.getDirectionTo(centroidPosition);

    // Determine speed & turn rate
    motorData.setDirection(direction);
    motorData.convertToTurn(myPosition.getYaw());
    motorData.optimizeDirection();
    motorData.setMagnitude(MAX_MOTOR_SPEED * (1 - fabs(motorData.getDirection()) / PI));
  }

    return motorData;
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
    if (directionToRobotAvoid >= 0 && directionToRobotAvoid <= (PI / 2)) {
      // Robot is in my left, turn right
      motorData.setDirection(directionToRobotAvoid - (PI / 2));
      motorData.optimizeDirection();
      motorData.setMagnitude(AVOID_SPEED * (smallestDistance/AVOID_DISTANCE));
      motorData.setWeight(AVOID_WEIGHT);
    } else if (directionToRobotAvoid >= -(PI / 2) && directionToRobotAvoid <= 0) {
      // Robot is in my right, turn left
      motorData.setDirection(directionToRobotAvoid + (PI / 2));
      motorData.optimizeDirection();
      motorData.setMagnitude(AVOID_SPEED * (smallestDistance/AVOID_DISTANCE));
      motorData.setWeight(AVOID_WEIGHT);
    }
  }

	return motorData;
}

RobotPosition RobotBehavior::getCentroid(const RobotList &robotList) {
  double centroidX = 0;
  double centroidY = 0;
  for (RobotConstIterator it = robotList.begin(); it != robotList.end(); it ++) {
    RobotPosition position = it->second;
    centroidX += position.getX();
    centroidY += position.getY();
  }

  centroidX = centroidX / robotList.size();
  centroidY = centroidY / robotList.size();
  return RobotPosition(centroidX, centroidY, 0);
}

double RobotBehavior::getNearestRobotDistance(const RobotList &robotList, const RobotPosition myPosition) {
  double result = -1;
  for (RobotConstIterator it = robotList.begin(); it != robotList.end(); it ++)
  {
  	RobotPosition robotPosition= it->second;
    double distance = robotPosition.getDistanceTo(myPosition);

    if (result < 0 || distance < result)
    {
      result = distance;
    }
  }

  return result;
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
