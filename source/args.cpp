#include <cstdlib>
#include <cstring>
#include <libplayerc++/playerc++.h>
#include <iostream>
#if !defined (WIN32) || defined (__MINGW32__)
  #include <unistd.h>
#endif
#if !HAVE_GETOPT
  #include <replace.h>
#endif

#include "args.h"

bool checkSetting(RobotSetting &setting)
{
  if (setting.senseRange < setting.distance)
  {
    return false;
  }
  return true;
}

int parse_args(int argc, char** argv, RobotSetting &setting)
{
  // set the flags
  const char* optflags = "?h:p:B:P:t:d:s:";
  int ch;

  // use getopt to parse the flags
  while(-1 != (ch = getopt(argc, argv, optflags)))
  {
    switch(ch)
    {
      // case values must match long_options
      case 'h': // hostname
      {
        setting.robotAddress = optarg;
        break;
      }
      case 'p': // port
      {
        setting.robotPort = atoi(optarg);
        break;
      }
      case 'B': // broadcast network address
      {
        setting.broadcastAddress = optarg;
        break;
      }
      case 'P': // listen port number
      {
        setting.broadcastPort = atoi(optarg);
        break;
      }
      case 't': // run type(a:aggression, d:dispersion)
      {
        if (optarg[0] == RUN_TYPE_AGGREGATION || optarg[0] == RUN_TYPE_DISPERSION)
        {
          setting.runType = optarg[0];
        }
        else
        { // unknow type
        }
        break;
      }
      case 's': // sense range
      {
        setting.senseRange = atof(optarg);
        break;
      }
      case 'd': // distance
      {
        setting.distance = atof(optarg);
        break;
      }
      case '?': // print help
      {
        print_usage(argc, argv);
        exit(0);
      }
      default:  // unknown
      {
        print_usage(argc, argv);
        exit (-1);
      }
    }
  }

  if (setting.senseRange < setting.distance) {
    std::cerr << "Error: sensory range is smaller than distance!" << std::endl;
    print_usage(argc, argv);
    exit (-1);
  }

  // Inches to meters
  setting.senseRange *= INCHE_METER_FACTOR;
  setting.distance *= INCHE_METER_FACTOR;

  // DEBUG
  std::cout << "Settings: (";
  std::cout << "Player address = " << setting.robotAddress << "; ";
  std::cout << "Player port = " << setting.robotPort << "; ";
  std::cout << "Broadcast address = " << setting.broadcastAddress << "; ";
  std::cout << "Listen port = " << setting.broadcastPort << "; ";
  std::cout << "Run type = " << setting.runType << "; ";
  std::cout << "Sense range = " << setting.senseRange << "; ";
  std::cout << "Distance = " << setting.distance <<  "; )" << std::endl;

  return 0;
} // end parse_args

void print_usage(int argc, char** argv)
{
  using namespace std;
  cerr << "USAGE:  " << *argv << " [options]" << endl << endl;
  cerr << "Where [options] can be:" << endl;
  cerr << "  -?             : show help" << endl;
  cerr << "  -h <hostname>  : hostname of Player (default: " << PlayerCc::PLAYER_HOSTNAME << ")" << endl;
  cerr << "  -p <port>      : port of Player (default: " << PlayerCc::PLAYER_PORTNUM << ")" << endl;
  cerr << "  -B <ip>        : broadcast address for inter-robot communication (default: " DEFAULT_BROADCAST_NETMASK<< ")" << endl;
  cerr << "  -P <port>      : listen port for inter-robot communication (default: " << DEFAULT_BROADCAST_PORT << ")" << endl;
  cerr << "  -t [a,d]       : (Required) running type (a:aggregation, d:dispersion)" << endl;
  cerr << "  -s <distance>  : sensory distance, in unit inches (default: " << DEFAULT_SENSE_DISTANCE << ")" << endl;
  cerr << "  -d <distance>  : (Required) maximum inter-robot distance(aggregation) / minimum inter-robot distance(dispersion), in unit inches" << endl;
  cerr << "Note: sensory distance(-s) should be larger than inter-robot distance(-d)" << endl;

} // end print_usage

RobotSetting::RobotSetting()
{
  robotAddress = std::string(PlayerCc::PLAYER_HOSTNAME);
  robotPort = PlayerCc::PLAYER_PORTNUM;
  broadcastAddress = std::string(DEFAULT_BROADCAST_NETMASK);
  broadcastPort = DEFAULT_BROADCAST_PORT;
  senseRange = DEFAULT_SENSE_DISTANCE;
}