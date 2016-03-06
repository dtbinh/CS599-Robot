#ifndef __ARGS_H__
#define __ARGS_H__

#include <libplayerc++/playerc++.h>
#include <string>

extern const std::string DEFAULT_BROADCAST_NETMASK;
extern const int DEFAULT_BROADCAST_PORT;
extern const double DEFAULT_SENSE_DISTANCE;
extern const char RUN_TYPE_AGGREGATION;
extern const char RUN_TYPE_DISPERSION;
extern const double INCHE_METER_FACTOR;

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