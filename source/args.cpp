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


int parse_args(int argc, char** argv, RobotSetting &setting)
{
  // set the flags
  const char* optflags = "h:p:B:i:P:t";
  int ch;

  // use getopt to parse the flags
  while(-1 != (ch = getopt(argc, argv, optflags)))
  {
    switch(ch)
    {
      // case values must match long_options
      case 'h': // hostname
        setting.robotAddress = optarg;
        break;
      case 'p': // port
        setting.robotPort = atoi(optarg);
        break;
      case 'B': // broadcast network address
        setting.broadcastAddress = optarg;
        break;
      case 'P': // listen port number
        setting.broadcastPort = atoi(optarg);
        break;
      case 't': // run type(a:aggression, d:dispersion)
        if (optarg[0] != RUN_TYPE_AGGREGATION && optarg[0] != RUN_TYPE_DISPERSION)
        {
          // error handling
        }
        else
        {
          setting.runType = optarg[0];
        }
        break;
      default:  // unknown
        print_usage(argc, argv);
        exit (-1);
    }
  }

  return (0);
} // end parse_args

void print_usage(int argc, char** argv)
{
  using namespace std;
  cerr << "USAGE:  " << *argv << " [options]" << endl << endl;
  cerr << "Where [options] can be:" << endl;
  cerr << "  -h <hostname>  : hostname to connect to (default: "
       << PlayerCc::PLAYER_HOSTNAME << ")" << endl;
  cerr << "  -p <port>      : port where Player will listen (default: "
       << PlayerCc::PLAYER_PORTNUM << ")" << endl;
  cerr << "  -r [l,w]            : Act as leader" << endl;
  cerr << "  -i <network>   : broadcast network address (default: " << DEFAUL_BROADCAST_NETMASK << ")" << endl;
  cerr << "  -P <port>      : listen port (default: " << DEFAUL_BROADCAST_PORT << ")" << endl;
} // end print_usage

RobotSetting::RobotSetting()
{
  robotAddress = std::string(PlayerCc::PLAYER_HOSTNAME);
  robotPort = PlayerCc::PLAYER_PORTNUM;
  broadcastAddress = std::string(DEFAUL_BROADCAST_NETMASK);
  broadcastPort = DEFAUL_BROADCAST_PORT;
}