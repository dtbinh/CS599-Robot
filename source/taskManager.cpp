#include <iostream>
#include <ctime>

#include "RobotCommunication.h"
#include "ArgumentsParser.h"

// Description of TaskManager
// Read user input for a list of tasks(formation, waypoint_x, waypoint_y)
// Iterate through the task list
// 		Broadcast current task
//		Record task start time
//		Message loop: Wait for the task completion report from leader robot
//		Record task complete time
//		Display task duration
//		Do next iteration
// Broadcast STOP command

// Global variables
#ifdef __APPLE__
	#define DEFAULT_BC_ADDRESS "192.168.0.255"
#else
	#define DEFAULT_BC_ADDRESS "127.255.255.255"
#endif
#define DEFAULT_LISTEN_PORT 9090

// Declarations
typedef struct RobotTask {
	RobotTask(char, double, double);
	const char TYPE_LINE = 'l';
	const char TYPE_DIAMOND = 'd';
	char type;
	double x;
	double y;
} RobotTask;

typedef std::list<RobotTask> RobotTaskList;

void printUsage();

// main function
int main(int argc, char** argv) {
	std::string GLOBAL_BC_ADDRESS = DEFAULT_BC_ADDRESS;
	int GLOBAL_LISTEN_PORT = DEFAULT_LISTEN_PORT;

	// Parse command line parameters
	ArgumentsParser argParser("hB:P:");
	ArgumentsMap argMap = argParser.parse(argc, argv).getMap();

	auto value = argMap.find('B');
	if (value != argMap.end()) {
		GLOBAL_BC_ADDRESS = value->second;
	}
	value = argMap.find('P');
	if (value != argMap.end()) {
		GLOBAL_LISTEN_PORT = atoi(value->second.c_str());
	}
	value = argMap.find('h');
	if (value != argMap.end()) {
		printUsage();
		return 0;
	}

	// Create communication sockets
	RobotCommunication::Communication communication(GLOBAL_BC_ADDRESS, DEFAULT_LISTEN_PORT);

	// Read task definition
	RobotTaskList taskList;
	std::cout << "Input task list:<formation> <x> <y> : " << std::endl;
	std::cout << "When finish, type in 's'" << std::endl;
	while (true) {
		char taskType;
		std::cin >> taskType;
		if (taskType == 's')
			break;
		double x, y;
		std::cin >> x >> y;
		RobotTask task(taskType, x, y);
		taskList.push_back(task);
		std::cout << "Input next task ..." << std::endl;
	}

	// Iterate through task list
	while (!taskList.empty()) {
		RobotTask currentTask = taskList.front();
		// Broadcast the task
		communication.sendMessageTask(0, currentTask.type, currentTask.x, currentTask.y);

		// Record start time
		std::cout << "Task(" << currentTask.type << ", " << currentTask.x << ", " << currentTask.y << ") start ... ";
		std::cout << std::flush;
		time_t startTime;
		time(&startTime);

		// Wait for task completion message
		bool exitLoop = false;
		while (!exitLoop) {
			RobotCommunication::Message message;
			while (communication.listenMessage(message)) {
				if (communication.waitForMessage(message, RobotCommunication::MSG_TYPE_TASK_DONE)) {
					exitLoop = true;
					break;
				}
			}
			if (!exitLoop)
				sleep(1);
		}

		// Record end time
		time_t endTime;
		time(&endTime);
		double duration = difftime(endTime, startTime);

		// Display duration
		std::cout << "done ";
		std::cout << "in " << duration << " seconds" << std::endl;

		// Current task done, delete it
		taskList.pop_front();
	}

	// All tasks done! Send STOP command
	communication.sendCommand(0, RobotCommunication::CMD_EXIT);

	return 0;
}

// Implementations begin
void printUsage() {
	std::cout << "Usage:" << std::endl;
	std::cout << "  -h           : (optional) Print help message" << std::endl;
	std::cout << "  -B<address>  : (optional) Broadcast address(DEFAULT:" << DEFAULT_BC_ADDRESS << ")" << std::endl;
	std::cout << "  -P<port>     : (optional) Listen port(DEFAULT:" << DEFAULT_LISTEN_PORT << ")" << std::endl;
}

RobotTask::RobotTask(char type, double x, double y): type(type), x(x), y(y) { }

// Implementations end