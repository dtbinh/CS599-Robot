#include <iostream>
#include <math.h>
#include <time.h>

#define PI 3.1415926535

typedef struct Position {
  double x;
  double y;
  double yaw;
} Position;

void randomPosition(double startAngle, double endAngle, double radius, Position &pos) {
  double randAngle = fmod(rand(),(endAngle - startAngle)) + startAngle;
  double randLength = fmod(rand(), radius - 0.8) + 0.5;
  pos.x = cos(randAngle) * randLength;
  pos.y = sin(randAngle) * randLength;
  pos.yaw = rand() % 360;
}

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "Required parameters: [radius]" << std::endl;
    return (-1);
  }
  double radius = atof(argv[1]);
  int numPositions = 6;
  std::string colors[6] = {"red", "NavyBlue", "green", "cyan", "yellow", "magenta"};

  srand(time(NULL));
  double angleStep = 2 * PI / numPositions;
  for (int i = 0; i < numPositions; i ++) {
    Position pos;
    randomPosition(i * angleStep, (i + 1) * angleStep, radius, pos);
    char buf[100];
    sprintf(buf, "[%.3f %.3f 0 %.3f]", pos.x, pos.y, pos.yaw);

    std::cout << "myrobot(";
    std::cout << " color \"" << colors[i] << "\"";
    std::cout << " name \"r" << (i+1) << "\"";
    std::cout << " pose " << buf;
    std::cout << ")" << std::endl;
  }
}
