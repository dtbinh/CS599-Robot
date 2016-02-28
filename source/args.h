#ifndef __ARGS_H__
#define __ARGS_H__

#include <libplayerc++/playerc++.h>
#include <string>

#ifdef __APPLE__
	#define DEFAULT_BROADCAST_NETMASK "192.168.0.255"
#else
	#define DEFAULT_BROADCAST_NETMASK "127.255.255.255"
#endif
#define DEFAULT_BROADCAST_PORT 9090
#define DEFAULT_SENSE_DISTANCE 140
#define RUN_TYPE_AGGREGATION 'a'
#define RUN_TYPE_DISPERSION 'd'
#define INCHE_METER_FACTOR 0.0254

class RobotSetting
{
public:
  std::string robotAddress;
  uint32_t robotPort;
  std::string broadcastAddress;
  int broadcastPort;
	char runType;
	double senseRange;
	double distance;

	RobotSetting();
};

void print_usage(int argc, char** argv);
int parse_args(int argc, char** argv, RobotSetting &setting);
bool checkSetting(RobotSetting &setting);

#endif