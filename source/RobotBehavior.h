#ifndef __ROBOT_BEHAVIOR_H__
#define __ROBOT_BEHAVIOR_H__

#include <map>
#include "RobotFormation.h"
#include "RobotBase.h"

namespace Robot {

  constexpr double MAX_MOTOR_SPEED = 0.5;
  constexpr double MAX_ANGLE_SPEED = 1;
  constexpr double AVOID_SPEED = 0.1;
  const int AVOID_WEIGHT = 2000;
  constexpr double AVOID_DISTANCE = 0.5;
  constexpr double AVOID_DISTANCE_DANGER = 0.3;

  const int LEADER_TASK_WEIGHT = 20;
  constexpr double LEADER_FINISH_DISTANCE = 0.02;

  const int FOLLOWER_TASK_WEIGHT = 20;
  constexpr double FOLLOWER_FINISH_DISTANCE = 0.02;

	class Behavior {
	public:
		Behavior();
		bool avoidRobot(Pose myPose, const PoseList &robotList, Velocity &vel);
		void setHasTask(bool hasTask);
		bool hasTask();
		Velocity diffGoTo(Pose goalPose, Pose currPose, double maxSpeedX, double maxSpeedAngle);

	protected:
		bool mHasTask;
		double normalizeAngle(double a);
	};

	class BehaviorLeader: public Behavior {
	public:
		BehaviorLeader();
		void assignTask(Pose targetPose);
		Velocity gotoTarget(Pose& myPosition);

	protected:
		Pose mTargetPose;

	};

	class BehaviorFollower: public Behavior {
	public:
		BehaviorFollower(int myID, int leaderID);
		void assignTask(Pose pos);
		double taskEstimate(Pose myPose, Pose targetPose);
		Velocity follow(Pose& myPose, Pose& leaderPose);
		Pose bid(Formation fmt, Pose refPose, const PoseList robotList, Pose myPose);

	private:
		Pose mCoordination;
		int mMyID;
		int mLeaderID;
	};

}
#endif