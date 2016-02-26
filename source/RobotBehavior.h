#ifndef __ROBOT_BEHAVIOR_H__
#define __ROBOT_BEHAVIOR_H__

#include <map>

#define PI 3.1415926535

class RobotPosition {
public:
	// Constructor
	RobotPosition();
	RobotPosition(double x, double y, double yaw);
	RobotPosition(const RobotPosition& robotPosition);
	// Setter/Getter
	double getX() const;
	double getY() const;
	double getYaw() const;
	RobotPosition& setX(double x);
	RobotPosition& setY(double y);
	RobotPosition& setYaw(double yaw);
	// Operator over-riding
	RobotPosition& operator= (const RobotPosition& position);
	// Other methods
	double getDistanceTo(const RobotPosition robotPosition) const;
	double getDirectionTo(const RobotPosition robotPosition) const;

private:
	double mX;
	double mY;
	double mYaw;
};

typedef std::map<int, RobotPosition> RobotList;
typedef std::map<int, RobotPosition>::iterator RobotIterator;
typedef std::map<int, RobotPosition>::const_iterator RobotConstIterator;

class MotorData {
public:
	// Constructor
	MotorData();
	MotorData(const MotorData& data);
	MotorData(const double magnitude, const double direction);
	// Setter/Getter
	double getMagnitude() const;
	double getDirection() const;
	MotorData& setDirection(double direction);
	MotorData& setMagnitude(double magnitude);
	// Operator overloading
	MotorData& operator= (const MotorData& data);
	MotorData operator+ (const MotorData& data);
	// Other methods
	MotorData& optimizeDirection();

private:
	double mMagnitude;
	double mDirection;
};

class RobotBehavior {
public:
	RobotBehavior();
	MotorData disperse(RobotPosition myPosition, const RobotList &robotList, double rangeLimit);
	MotorData aggregate(RobotPosition myPosition, const RobotList &robotList, double rangeLimit);
	MotorData avoidRobot(RobotPosition myPosition, const RobotList &robotList);

private:
	RobotPosition getCentroid(const RobotList &robotList);
	double getNearestRobotDistance(const RobotList &robotList, const RobotPosition myPosition);
};

#endif