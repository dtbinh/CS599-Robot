#include "RobotTimer.h"
#include <chrono>

RobotTimer::RobotTimer() {
  mStarted = false;
}

RobotTimer::RobotTimer(int durationSeconds) {
  this->setDuration(durationSeconds);
}

void RobotTimer::setDuration(int durationSeconds) {
  this->mDuration = durationSeconds;
}

void RobotTimer::start() {
  this->mStartTime = std::chrono::system_clock::now();
  this->mStarted = true;
}

void RobotTimer::stop() {
  this->mStarted = false;
}

bool RobotTimer::isTimeout() {
  if (!mStarted) {
    return false;
  }
  std::chrono::system_clock::time_point endTime = std::chrono::system_clock::now();
  std::chrono::duration<double> timeDiff = endTime - mStartTime;
  return timeDiff.count() >= mDuration;
}
