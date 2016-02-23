#ifndef __ARGS_H__
#define __ARGS_H__

#include <libplayerc++/playerc++.h>
#include <string>

#define DEFAUL_BROADCAST_NETMASK "127.255.255.255"
#define DEFAUL_BROADCAST_PORT 9090
#define RUN_TYPE_AGGREGATION 'a'
#define RUN_TYPE_DISPERSION 'd'

class RobotSetting
{
public:
  std::string robotAddress;
  uint32_t robotPort;
  std::string broadcastAddress;
  int broadcastPort;
	char runType;
	double senseRange;
	double robotDistance;

	RobotSetting();
};

void print_usage(int argc, char** argv);
int parse_args(int argc, char** argv, RobotSetting &setting);

#endif