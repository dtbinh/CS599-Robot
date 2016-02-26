#include <cmath>
#include "RobotBehavior.h"

/*
* Implementation of MotorData
*/
MotorData::MotorData(): mMagnitude(0), mDirection(0) { }

MotorData::MotorData(const MotorData& data) {
	this->setMagnitude(data.getMagnitude()).setDirection(data.getDirection());
}

MotorData::MotorData(const double magnitude, const double direction) {
	this->setMagnitude(magnitude).setDirection(direction);
}

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

MotorData& MotorData::operator =(const MotorData& data) {
	this->setMagnitude(data.getMagnitude()).setDirection(data.getDirection());
	return *this;
}

// Vector sumation: to combine two behaviors
// Return: direction cast into [0, 2*PI]
MotorData MotorData::operator+ (const MotorData& data) {
	double myX = this->mMagnitude * cos(this->mDirection);
	double myY = this->mMagnitude * sin(this->mDirection);
	double x = data.getMagnitude() * cos(data.getDirection());
	double y = data.getMagnitude() * sin(data.getDirection());
	double newX = x + myX;
	double newY = y + myY;
	double direction = atan(newY / newX);
	direction = (newX < 0)? (PI + direction): fmod((PI * 2 + direction), (PI * 2)); // Convert to [0, 2*PI]
	double magnitude = sqrt(newX * newX + newY * newY);

	return MotorData(magnitude, direction);
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


/*
* Implementation of RobotBehavior
*/
RobotBehavior::RobotBehavior() {}

MotorData RobotBehavior::disperse(RobotPosition myPosition, const RobotList &robotList, double rangeLimit) {

	MotorData motorData(0, 0);
  double nearestDistance = this->getNearestRobotDistance(robotList, myPosition);
  if (nearestDistance > 0 && nearestDistance < rangeLimit)
  { // at least one robot near me

    // I should go away from the centroid
    // Find out the centeroid position
    RobotPosition centroidPosition = this->getCentroid(robotList);

    // Find out the target direction
    double direction = centroidPosition.getDirectionTo(myPosition);

    // Determine speed & turn rate
    motorData.setDirection(direction - myPosition.getYaw());
    motorData.optimizeDirection();
    if (fabs(motorData.getDirection()) < (180 / PI * 20)) { // small angle turn: go and turn
    	motorData.setMagnitude(0.2);
    }
  }

	return motorData;
}

MotorData RobotBehavior::aggregate(RobotPosition myPosition, const RobotList &robotList, double rangeLimit) {
	MotorData motorData(0, 0);
  double nearestDistance = this->getNearestRobotDistance(robotList, myPosition);
  if (nearestDistance > rangeLimit)
  { // the nearest robot is out of range

    // I should go toward the centroid
    // Find out the centeroid position
    RobotPosition centroidPosition = this->getCentroid(robotList);

    // Find out the target direction
    double direction = myPosition.getDirectionTo(centroidPosition);

    // Determine speed & turn rate
    motorData.setDirection(direction - myPosition.getYaw());
    motorData.optimizeDirection();
    if (fabs(motorData.getDirection()) < (180 / PI * 20)) { // small angle turn: go and turn
    	motorData.setMagnitude(0.2);
    }
  }

    return motorData;
}

MotorData RobotBehavior::avoidRobot(RobotPosition myPosition, const RobotList &robotList) {
	return MotorData(0, 0);
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

    double direction = atan(targetY / targetX);
    if (targetX < 0) // transform the coordination from C to Player
      direction = PI + direction;
    else
      direction = fmod((2 * PI + direction), (2 * PI));

    return direction;
}
