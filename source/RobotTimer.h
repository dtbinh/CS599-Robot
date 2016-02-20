#ifndef __ROBOTTIMER_H__
#define __ROBOTTIMER_H__

#include <chrono>

class RobotTimer {
public:
  RobotTimer();
  RobotTimer(int durationSeconds);
  void setDuration(int durationSeconds);
  void start();
  void stop();
  bool isTimeout();

private:
  int mDuration;
  bool mStarted;
  std::chrono::system_clock::time_point mStartTime;
};

#endif