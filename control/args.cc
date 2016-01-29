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
  const char* optflags = "h:p:r:i:P:";
  int ch;

  // use getopt to parse the flags
  while(-1 != (ch = getopt(argc, argv, optflags)))
  {
    switch(ch)
    {
      // case values must match long_options
      case 'h': // hostname
          setting.hostName = optarg;
          break;
      case 'p': // port
          setting.port = atoi(optarg);
          break;
      case 'r': // role
          if (optarg[0] == 'l') {
            setting.role = ROBOT_ROLE_LEADER;
          } else {
            setting.role = ROBOT_ROLE_WORKER;
          }
          break;
      case 'i': // broadcast network address
          setting.remoteAddress = optarg;
          break;
      case 'P': // listen port number
          setting.listenPort = atoi(optarg);
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

RobotSetting::RobotSetting() {
  hostName = std::string(PlayerCc::PLAYER_HOSTNAME);
  port = PlayerCc::PLAYER_PORTNUM;
  role = ROBOT_ROLE_WORKER;
  remoteAddress = std::string(DEFAUL_BROADCAST_NETMASK);
  listenPort = DEFAUL_BROADCAST_PORT;
}
