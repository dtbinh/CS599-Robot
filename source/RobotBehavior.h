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
	MotorData(const double magnitude, const double direction, const int weight);
	// Setter/Getter
	double getMagnitude() const;
	double getDirection() const;
	int getWeight() const;
	MotorData& setDirection(double direction);
	MotorData& setMagnitude(double magnitude);
	MotorData& setWeight(int weight);
	// Operator overloading
	MotorData& operator= (const MotorData& data);
	MotorData operator+ (const MotorData& data);
	// Other methods
	MotorData& optimizeDirection();
	MotorData& convertToTurn(double currentDirection);

private:
	double mMagnitude;
	double mDirection;
	int mWeight;
};

class RobotBehavior {
public:
	RobotBehavior();
	MotorData disperse(RobotPosition myPosition, const RobotList &robotList, double rangeLimit);
	MotorData aggregate(RobotPosition myPosition, const RobotList &robotList, double rangeLimit);
	MotorData avoidRobot(RobotPosition myPosition, const RobotList &robotList);

private:
	static constexpr double MAX_MOTOR_SPEED = 0.3;
	static constexpr double AVOID_SPEED = 0.1;

	static constexpr double DISPERSE_WEIGHT = 2;
	static constexpr double AGGREGATE_WEIGHT = 2;
	static constexpr double AVOID_WEIGHT = 5;

	static constexpr double SMOOTH_TURN_ANGLE = 20;
	static constexpr double AVOID_DISTANCE = 0.5;
	static constexpr double AVOID_ROBOT_SIZE = 0.3;

	RobotPosition getCentroid(const RobotList &robotList);
	double getNearestRobotDistance(const RobotList &robotList, const RobotPosition myPosition);
};

#endif