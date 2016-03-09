#ifndef __ROBOT_FORMATION_H__
#define __ROBOT_FORMATION_H__

#include "RobotBase.h"

namespace Robot {
	class Formation {
	public:
		static const int TEAM_SIZE = 4;
		static const char TYPE_LINE = 'l';
		static const char TYPE_DIAMOND = 'd';
		static constexpr double DISTANCE = 1;
		Formation();
		void setLine();
		void setDiamond();
		Pose* getShape();
		int getLeaderIndex();

	private:
		char mType;
		Pose mShape[TEAM_SIZE];
		int mLeaderIndex;
	};
}

#endif