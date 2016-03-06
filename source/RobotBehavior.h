#ifndef __ROBOT_BEHAVIOR_H__
#define __ROBOT_BEHAVIOR_H__

#include <map>
#include "RobotFormation.h"

extern const double PI;

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
	MotorData avoidRobot(RobotPosition myPosition, const RobotList &robotList);

protected:
	static constexpr double MAX_MOTOR_SPEED = 0.5;
	static constexpr double AVOID_SPEED = 0.1;
	static constexpr double AVOID_WEIGHT = 100;
	static constexpr double AVOID_DISTANCE = 0.8;
};

class RobotBehaviorLeader: public RobotBehavior {
public:
	RobotBehaviorLeader();
	void setTarget(double x, double y);
	MotorData gotoTarget(RobotPosition& myPosition);

private:
	static constexpr double TASK_WEIGHT = 2;
	static constexpr double NEAR_DISTANCE = 1;
	static constexpr double FINISH_DISTANCE = 0.01;
	double mTargetX;
	double mTargetY;
};

class RobotBehaviorFollower: public RobotBehavior {
public:
	RobotBehaviorFollower();
	void setCoordination(RobotFormation::Coordination coordination);
	double taskEstimate(RobotPosition myPosition, RobotPosition targetPosition);
	MotorData follow(RobotPosition& myPosition, RobotPosition& leaderPosition);

private:
	static constexpr double TASK_WEIGHT = 2;
	static constexpr double FINISH_DISTANCE = 0.01;
	RobotFormation::Coordination mCoordination;
};

#endif