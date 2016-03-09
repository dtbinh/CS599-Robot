#include "RobotFormation.h"

namespace Robot {
	Formation::Formation() { }

	void Formation::setLine() {
		// Coordinations are relative to the leader
		// Points are sorted descending by x and y
		mType = TYPE_LINE;
		mLeaderIndex = 1;

		mShape[0].x = 0; mShape[0].y = DISTANCE;
		mShape[1].x = 0; mShape[1].y = 0;
		mShape[2].x = 0; mShape[2].y = -1 * DISTANCE;
		mShape[3].x = 0; mShape[3].y = -2 * DISTANCE;
	}

	void Formation::setDiamond() {
		mType = TYPE_DIAMOND;
		mLeaderIndex = 0;

		mShape[0].x = 0; mShape[0].y = 0;
		mShape[1].x = -1 * DISTANCE; mShape[1].y = DISTANCE;
		mShape[2].x = -1 * DISTANCE; mShape[2].y = -1 * DISTANCE;
		mShape[3].x = -2 * DISTANCE; mShape[3].y = 0;
	}

	Pose* Formation::getShape() {
		return mShape;
	}

	int Formation::getLeaderIndex() {
		return mLeaderIndex;
	}

}
