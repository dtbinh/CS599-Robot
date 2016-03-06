#ifndef __ROBOT_FORMATION_H__
#define __ROBOT_FORMATION_H__

class RobotFormation {
public:
	static const int TEAM_SIZE = 4;
	static const char TYPE_LINE = 'l';
	static const char TYPE_DIAMOND = 'd';
	static constexpr double DISTANCE = 1;
	typedef struct Coordination {
		Coordination();
		Coordination(double, double);
		double x;
		double y;
	} Coordination;
	RobotFormation();
	void setLine();
	void setDiamond();
	Coordination* getShape();

private:
	char mType;
	RobotFormation::Coordination mShape[TEAM_SIZE];
	int mLeaderIndex;
};

#endif