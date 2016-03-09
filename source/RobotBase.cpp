#include <cmath>
#include "RobotBase.h"

namespace Robot {

  const double PI = 3.1415926535;

  Pose::Pose(): x(0), y(0), a(0) { }
  Pose::Pose(double x, double y, double a): x(x), y(y), a(a) { }
  Pose::Pose(const Pose &p): x(p.x), y(p.y), a(p.a) { }
  double Pose::distance(const Pose &p) const {
    return std::hypot(x - p.x, y - p.y);
  }

  double Pose::angle(const Pose &p) const {
    if (x == p.x && y == p.y)
      return 0;
    return std::atan2(p.y - y, p.x - x);
  }

  std::string Pose::toString() {
    char buf[256];
    snprintf(buf, 256, "[ %.3f %.3f %.3f]", x, y, a);
    return std::string(buf);
  }

  bool Pose::operator== (const Pose& other) const {
    return (x == other.x &&
      y == other.y &&
      a == other.a);
  }

  Velocity::Velocity(): Pose() { }
  Velocity::Velocity(double x, double y, double a): Pose(x, y, a) { }
  Velocity::Velocity(const Velocity &v): Pose(v.x, v.y, v.a) { }

  WVelocity::WVelocity(): Velocity() { }
  WVelocity::WVelocity(Velocity vel, int w): Velocity(vel), w(w) { }
  WVelocity::WVelocity(double x, double y, double a, int w): Velocity(x, y, a), w(w) { }

  WVelocity WVelocity::operator+ (const WVelocity& other) {
    int totalWeight = w + other.w;
    double x = x * w / totalWeight + other.x * other.w / totalWeight;
    double y = y * w / totalWeight + other.y * other.w / totalWeight;
    double a = a * w / totalWeight + other.a * other.w / totalWeight;
    return WVelocity(x, y, a, totalWeight);
  }

	std::string WVelocity::toString() {
    char buf[256];
    snprintf(buf, 256, "[ %.3f %.3f %.3f %d]", x, y, a, w);
    return std::string(buf);
  }
}