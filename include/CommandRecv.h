


#include <string>
#include <sstream>
#include <vector>
#include <stdexcept>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <fstream>
#include <array>
#include <cstring>

#include <configor/json.hpp>

using namespace configor;
#include <thread>
#include <filesystem>

#include <sys/socket.h>		// For socket()
#include <netinet/in.h> 	// For socket() and sockaddr_in struct */
#include <netinet/udp.h>	// For socket()
#include <unistd.h>			// For close()
#include <arpa/inet.h>		// For inet_aton()

#include <wiringPi.h>

void Command_task(int n);
int readConfig(json& j);
extern bool gRestart;
extern json gconfig;
