#ifndef __ARGS_H__
#define __ARGS_H__

#include <libplayerc++/playerc++.h>
#include <string>

#define ROBOT_ROLE_LEADER 1
#define ROBOT_ROLE_WORKER 2
#define DEFAUL_BROADCAST_NETMASK "127.255.255.255"
#define DEFAUL_BROADCAST_PORT 9090

class RobotSetting {
public:
  std::string hostName;
  uint32_t port;
  int role;
  std::string remoteAddress;
  int listenPort;

  RobotSetting();
};

void print_usage(int argc, char** argv);
int parse_args(int argc, char** argv, RobotSetting &setting);

#endif