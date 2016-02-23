#include <libplayerc++/playerc++.h>
#include <iostream>

#include "args.h"
#include "behavior.h"
#include "RobotMessage.h"
#include "RobotSocketConnection.h"

#define PI 3.1415926535

#define MAX_ROBOT_NUMBER 10
class RobotInfo
{
public:
  double x = 0;
  double y = 0;
  int robotID = -1;
};

class RobotList
{
public:
  RobotList(){ this->mSize = 0; };
  int getSize() { return this->mSize; }

  RobotInfo* getInfo(int index)
  {
    if (index < this->mSize) { return &(this->mList[index]); }
    return NULL;
  }

  void updateInfo(const RobotInfo &info)
  {
     // Add/Update position
    for (int i = 0; i < MAX_ROBOT_NUMBER; i ++)
    {
      RobotInfo *pInfo = &(this->mList[i]);
      if (pInfo->robotID == info.robotID)
      { // Update position
        pInfo->x = info.x;
        pInfo->y = info.y;
        break;
      }

      if (pInfo->robotID < 0)
      { // Add a new robot
        pInfo->robotID = info.robotID;
        pInfo->x = info.x;
        pInfo->y = info.y;
        this->mSize = i + 1;
        break;
      }
    }
  }

private:
  RobotInfo mList[MAX_ROBOT_NUMBER];
  int mSize;
};

int main(int argc, char **argv)
{
  RobotSetting robotSetting;
  parse_args(argc, argv, robotSetting);
  int myID = robotSetting.robotPort;
  RobotList robotList; //TODO: use hash map

  try
  {
    PlayerCc::PlayerClient robot(robotSetting.robotAddress, robotSetting.robotPort);
    PlayerCc::Position2dProxy pp(&robot, 0);

    // Enable motor
    pp.SetMotorEnable (true);

    // Create communication
    RobotSocket listenSocket, sendSocket;
    listenSocket.createListen(robotSetting.broadcastPort);
    sendSocket.createSend();

    // Main Loop
    bool exitLoop = false;
    while (!exitLoop)
    {
      // Listen to position report, record position of other robots
      RobotMessage message;
      while (listenMessage(listenSocket, message))
      { // Message Loop
        if (waitForMessage(message, MSG_TYPE_POSITION) && message.getSenderID() != myID)
        {
          // Add/Update position
          RobotInfo robotInfo;
          robotInfo.robotID = message.getSenderID();
          robotInfo.x = message.getX();
          robotInfo.y = message.getY();
          robotList.updateInfo(robotInfo);
        }

        if (waitForCommand(message, CMD_EXIT))
        {
          exitLoop = true;
        }
      }

      // Read robot status
      robot.Read();

      // Get current position
      double myX = pp.GetXPos();
      double myY = pp.GetYPos();
      double myYaw = pp.GetYaw();

      // Broadcast my position
      sendMessagePosition(sendSocket, robotSetting.broadcastAddress, robotSetting.broadcastPort, myID, myX, myY);

      // Calculate range
      double rangeLimit = 10;
      double centroidX = 0;
      double centroidY = 0;
      int inRangeCount = 0;
      for (int i = 0; i < robotList.getSize(); i ++)
      {
        RobotInfo *pInfo = robotList.getInfo(i);
        double x = std::fabs(pInfo->x - myX);
        double y = std::fabs(pInfo->y - myY);
        double distance = std::sqrt(x*x + y*y);
        if (distance > rangeLimit) { continue; }
        centroidX += pInfo->x;
        centroidY += pInfo->y;
        inRangeCount ++;

        // DEBUG
        // char msg[100];
        // sprintf(msg, "Robot%i(%.2f, %.2f, %.2f)", inRangeCount, pInfo->x, pInfo->y, distance);
        // std::cout << msg << " ";
      }
      if (inRangeCount > 0) {
        // centroidX = centroidX/inRangeCount;
        // centroidY = centroidY/inRangeCount;
        centroidX = (centroidX + myX)/(inRangeCount+1);
        centroidY = (centroidY + myY)/(inRangeCount+1);
        double targetX = myX - centroidX;
        double targetY = myY - centroidY;
        double newYaw = std::atan(targetY / targetX);
        double newSpeed = 0;
        if (targetX < 0)
        {
          newYaw = PI + newYaw;
        }

        // Normalize the Yaw values
        if (myYaw > PI) {myYaw = myYaw - 2*PI;}
        if (newYaw > PI) {newYaw = newYaw - 2*PI;}
        // Determine speed
        newSpeed = (std::fabs(newYaw - myYaw) > (PI/180*10))? 0: 1;
        pp.SetSpeed(newSpeed, newYaw - myYaw);

        // DEBUG
        // char msg[100];
        // sprintf(msg, "MyPosition(%.2f, %.2f, %.2f); Centroid(%.2f, %.2f); NewYaw(%.2f)", myX, myY, PlayerCc::rtod(myYaw), centroidX, centroidY, PlayerCc::rtod(newYaw));
        // std::cout << msg << std::endl;
      }
      else
      {
        pp.SetSpeed(0, 0);
        // DEBUG
        // std::cout << "No one in range" << std::endl;
      }
      // pp.SetSpeed(newspeed, PlayerCc::dtor(newturnrate));

      // DEBUG
      // for (int i = 0; i < robotList.getSize(); i ++)
      // {
      //   RobotInfo *info = robotList.getInfo(i);
      //   char msg[60];
      //   sprintf(msg, "%d(%.2f,%.2f)", info->robotID, info->x, info->y);
      //   std::cout << msg << " " << std::flush;
      // }

      sleep(1);
    }
  }
  catch (PlayerCc::PlayerError & e)
  {
    std::cerr << e << std::endl;
    return -1;
  } catch (RobotSocketException & e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::cout << "Robot(" << myID << ") exit!" << std::endl;
}

