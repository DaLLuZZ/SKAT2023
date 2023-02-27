#define _WIN32_WINNT 0x0600

#include <sstream>
#include <vector>
#include <algorithm>

#include "server.h"
#include "client.h"
#include "autogen-CONNECTOR-macros.h"

#define RAD_TO_DEG 57.29577951
#define DEG_TO_RAD 1.0/RAD_TO_DEG
#define FT_TO_M 0.3048

#define MESSAGE_LEN 26
#define MESSAGE_BACK_LEN 128

// from JSBSim Init File
#define INITIAL_LATITUDE 47
#define INITIAL_LONGITUDE 122

fd_set master;
fd_set read_fds;
int fdmax = 0;

// this = wrapper.exe
connector::server<connector::UDP> receiver(4444); // SimInTech -> this
connector::client<connector::TCP> sender(1139, "127.0.0.1"); // this -> JSBSim
connector::server<connector::UDP> backreceiver(1138); // JSBSim -> this
connector::client<connector::UDP> backsender(4445, "127.0.0.1"); // this -> SimInTech

bool holding = true; // Is JSBSim being held?
bool initialized = false; // Was SimInTech project already initialized?

struct aircraft
{
	float time;
	float height;
	float latitude;
	float longitude;
	float pitch;
	float roll;
	float yaw;
	float X;
	float Y;
};

struct aircmd
{
	float RUD;
	float elevator;
	float rudder;
	float aileron;
};

void reinit_socket();
void parse_JSBSim_output(char* szOutPut, aircraft& plane);
void send_aircommand(aircmd aircontrolcmd);

void init_socket()
{
	printf("Initializing all sockets...\n");
	FD_ZERO(&master);
	FD_ZERO(&read_fds);
	fdmax = 0;

    WSAData wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);

    receiver.init();
    if (sender.init() == -1)
    {
    	printf("It looks like JSBSim is not running yet! Waiting...");
    	system("timeout 5");
    	reinit_socket();
    	return;
	}
    backreceiver.init();
    backsender.init();

	FD_SET(receiver._skt, &master);
	FD_SET(backreceiver._skt, &master);

	fdmax = backreceiver._skt > receiver._skt ? backreceiver._skt : receiver._skt;

	printf("All sockets are ready!\n");
}

void close_socket()
{
	printf("Closing all sockets...\n");
	closesocket(receiver._skt);
	closesocket(sender._skt);
	closesocket(backreceiver._skt);
	closesocket(backsender._skt);

	FD_ZERO(&master);
	FD_ZERO(&read_fds);
	fdmax = 0;

	WSACleanup();

	printf("All sockets were closed!\n");
}

void reinit_socket()
{
	printf("\nLost TCP-Connection to JSBSim, retrying...\n");
	close_socket();
	holding = true;
	initialized = false;
	init_socket();
}

int main()
{
	init_socket();

	char recieved[MESSAGE_LEN + 1];
    char backreceived[MESSAGE_BACK_LEN + 1];

    while (true)
    {
    	read_fds = master;
		for (int i = 0; i < sizeof(recieved); i++)
			recieved[i] = 0x00;
		for (int i = 0; i < sizeof(backreceived); i++)
			backreceived[i] = 0x00;

		if (select(fdmax + 1, &read_fds, NULL, NULL, NULL) == -1)
			break;

		for (int i = 0; i <= fdmax; i++)
		{
			if (FD_ISSET(i, &read_fds))
			{
				if (i == receiver._skt)
				{
			        receiver.receive_udp<char>(recieved[0], sizeof(char) * MESSAGE_LEN);
			        if (holding && initialized)
			        {
			        	char szMsg[8];
			        	szMsg[7] = 0;
			        	sprintf(szMsg, "resume\n");
				        if (!sender.send_tcp<char>(szMsg[0], sizeof(char) * strlen(szMsg)))
							reinit_socket();
						else
							holding = false;
					}
					if (!initialized)
					{
						printf("SimInTech initialization was detected!\n");
						initialized = true;
						continue;
					}
//			        printf("Got \"%s\" from SimInTech -> Sending to JSBSim\n", recieved);
					aircmd aircontrolcmd = reinterpret_cast<aircmd&>(recieved);
					send_aircommand(aircontrolcmd);
				}
				else if (i == backreceiver._skt)
				{
			        backreceiver.receive_udp<char>(backreceived[0], sizeof(char) * MESSAGE_BACK_LEN);
			        if (backreceived[0] != 0x00)
			        {
//			            printf("Got \"%s\" from JSBSim -> Sending to SimInTech\n", backreceived);
			            aircraft plane;
			            parse_JSBSim_output(backreceived, plane);
			            backsender.send_udp<char>(reinterpret_cast<char&>(plane.time), sizeof(aircraft) * 7);
			        }
				}
				break;
			}
		}
    }
}

/** 
 * JSBSim output string parser (all 4-byte floats):
 * 1) Height above ground level (ft)
 * 2) Latitude (deg)
 * 3) Longitude (deg)
 * 4) Pitch (rad)
 * 5) Roll (rad)
 * 6) Yaw (rad)
**/
void parse_JSBSim_output(char* szOutPut, aircraft& plane)
{
	std::stringstream str(szOutPut);
	std::vector<std::string> result;

	while (str.good())
	{
	    std::string substr;
	    getline(str, substr, ',');
	    result.push_back(substr);
	}

	for (int i = 0; i < result.size(); i++)
		result[i].erase(std::remove_if(result[i].begin(), result[i].end(), ::isspace), result[i].end());

	plane.time = std::stof(result[0]); // time is always sent by JSBSim
	plane.height = FT_TO_M * std::stof(result[1]);
	plane.latitude = std::stof(result[2]);
	plane.longitude = std::stof(result[3]);
	plane.pitch = RAD_TO_DEG * std::stof(result[4]);
	plane.roll = RAD_TO_DEG * std::stof(result[5]);
	plane.yaw = RAD_TO_DEG * std::stof(result[6]);
	plane.X = (INITIAL_LONGITUDE - plane.longitude) * 111300;
	plane.Y = (INITIAL_LATITUDE - plane.latitude) * 111300 * cos(DEG_TO_RAD * plane.longitude);
}

void send_aircommand(aircmd aircontrolcmd)
{
	static float old_aileron = 0.0, old_elevator = 0.0, old_rudder = 0.0, old_RUD = 0.0;
	char szBuffer[64];
	szBuffer[0] = 0x00;

	if (aircontrolcmd.aileron != old_aileron)
	{
		sprintf(szBuffer, "set fcs/aileron-cmd-norm %f\n", aircontrolcmd.aileron);
		for (int i = 0; i < sizeof(szBuffer); i++)
			if (szBuffer[i] == 0x0A)
				szBuffer[i + 1] = 0x00;
		if (!sender.send_tcp<char>(szBuffer[0], sizeof(char) * strlen(szBuffer)))
			reinit_socket();
	}

	if (aircontrolcmd.elevator != old_elevator)
	{
		sprintf(szBuffer, "set fcs/elevator-cmd-norm %f\n", aircontrolcmd.elevator);
		for (int i = 0; i < sizeof(szBuffer); i++)
			if (szBuffer[i] == 0x0A)
				szBuffer[i + 1] = 0x00;
		if (!sender.send_tcp<char>(szBuffer[0], sizeof(char) * strlen(szBuffer)))
			reinit_socket();
	}

	if (aircontrolcmd.rudder != old_rudder)
	{
		sprintf(szBuffer, "set fcs/rudder-cmd-norm %f\n", aircontrolcmd.rudder);
		for (int i = 0; i < sizeof(szBuffer); i++)
			if (szBuffer[i] == 0x0A)
				szBuffer[i + 1] = 0x00;
		if (!sender.send_tcp<char>(szBuffer[0], sizeof(char) * strlen(szBuffer)))
			reinit_socket();
	}

	if (aircontrolcmd.RUD != old_RUD)
	{
		sprintf(szBuffer, "set fcs/throttle-cmd-norm %f\n", aircontrolcmd.RUD);
		for (int i = 0; i < sizeof(szBuffer); i++)
			if (szBuffer[i] == 0x0A)
				szBuffer[i + 1] = 0x00;
		if (!sender.send_tcp<char>(szBuffer[0], sizeof(char) * strlen(szBuffer)))
			reinit_socket();
	}

	old_aileron = aircontrolcmd.aileron;
	old_elevator = aircontrolcmd.elevator;
	old_rudder = aircontrolcmd.rudder;
	old_RUD = aircontrolcmd.RUD;
}
