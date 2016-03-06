#include "RobotFormation.h"

RobotFormation::Coordination::Coordination(double x, double y): x(x), y(y) { }
RobotFormation::Coordination::Coordination() { }

RobotFormation::RobotFormation() { }

void RobotFormation::setLine() {
	// Coordinations are relative to the leader
	// Points are sorted descending by x and y
	mType = TYPE_LINE;
	mLeaderIndex = 1;

	mShape[0].x = 0; mShape[0].y = DISTANCE;
	mShape[1].x = 0; mShape[1].y = 0;
	mShape[2].x = 0; mShape[2].y = -1 * DISTANCE;
	mShape[3].x = 0; mShape[3].y = -2 * DISTANCE;
}

void RobotFormation::setDiamond() {
	mType = TYPE_DIAMOND;
	mLeaderIndex = 0;

	mShape[0].x = 0; mShape[0].y = 0;
	mShape[1].x = -1 * DISTANCE; mShape[1].y = DISTANCE;
	mShape[2].x = -1 * DISTANCE; mShape[2].y = -1 * DISTANCE;
	mShape[3].x = -2 * DISTANCE; mShape[3].y = 0;
}

RobotFormation::Coordination* RobotFormation::getShape() {
	return mShape;
}