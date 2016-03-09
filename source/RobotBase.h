#ifndef __ROBOT_BASE_H__
#define __ROBOT_BASE_H__

#include <map>
#include <string>

namespace Robot {
	extern const double PI;

	class Pose {
	public:
		Pose();
		Pose(double x, double y, double a);
		Pose(const Pose &p);
		double distance(const Pose &p) const;
		double angle(const Pose &p) const;
		std::string toString();
		bool operator== (const Pose& other) const;

		double x;
		double y;
		double a;
	};

	class Velocity: public Pose {
	public:
		Velocity();
		Velocity(double x, double y, double a);
		Velocity(const Velocity &v);
	};

	class WVelocity: public Velocity {
	public:
		WVelocity();
		WVelocity(Velocity vel, int w);
		WVelocity(double x, double y, double a, int w);
		WVelocity operator+ (const WVelocity& other);
		std::string toString();

		int w;
	};

	typedef std::map<int, Pose> PoseList;
	typedef std::map<int, Pose>::iterator PoseIterator;
	typedef std::map<int, Pose>::const_iterator PoseConstIterator;

}

#endif