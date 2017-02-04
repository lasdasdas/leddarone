
//This is a Linux only script for the LeddarOne by Leddartech, based on the example or theyr SDK LeddarOneLibModbusDemo.cpp:
//


#include <sstream>


//This is the proper way to add the modbus libraries, they MUST be installed with apt install modbus5 modbus-dev and so on
//If you wanna download them and compile them by yourself, you might have to change the directory below

#include <modbus/modbus.h>
#include <modbus/modbus-rtu.h>

//IMPORTANT!!!!!
//The modbus.so dinlibrary must be included in the CMake file

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <thread>
#include "HighResClock.hpp"


//Ros libraries
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include <sensor_msgs/LaserScan.h>
#include <ros/console.h>

// ***********************************************************************************************
// MACROS
// ***********************************************************************************************

#define LEDDAR_MAX_DETECTIONS 3

#define GETS_S(str) { \
	fgets(str, sizeof(str), stdin); \
	if (strlen(str) > 0) str[strlen(str)-1] = 0; }

#ifndef _MSC_VER
	typedef unsigned int UINT;
#endif

#ifndef MAX
	#define MAX(x,y) (((x) > (y)) ? (x) : (y))
#endif

// ***********************************************************************************************
// TYPES
// ***********************************************************************************************
using namespace std;

struct SDetection
{
	double dDistance;						 // distance from the sensor, in meters
	double dAmplitude;						 // signal amplitude
};
typedef struct laserstrucall
{
    double distance;                        //The same as S Detection but I want to use another handle
    double intensity;                       //To avoid interferance with the funtions that were provided in the sdk
                                            //I know, i know... -.-
} laserstructtype;




// ***********************************************************************************************
// IMPLEMENTATION
// ***********************************************************************************************



// Ask for the slave address
static bool AskSlaveAddr(modbus_t* mb , char str[128])
{

    if (modbus_set_slave(mb, atoi(str)) != 0)
    {
        printf("Error setting slave id\n");
        return false;
    }
	return true;
}


//Read detections from the regestry, called in testonereading
static bool ReadDetections(modbus_t* mb, SDetection tabDetections[LEDDAR_MAX_DETECTIONS],
						   UINT& uDetectionCount, UINT& uTimestamp)
{
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Read 10 registers from the address 20
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    uint16_t tabRegValues[32];
    //usleep(10000);//This waits a bit in order to make sure an adress is found
    int numRead = modbus_read_input_registers(mb, 20, 10, tabRegValues);

        if (numRead == 10)//Ten means alles gut
	{
		uTimestamp = tabRegValues[0] + (tabRegValues[1] << 16);
        //float fTemperature = (float)tabRegValues[2] / 256.f; This wont be used nor returned for now
		uDetectionCount = tabRegValues[3] < LEDDAR_MAX_DETECTIONS ? tabRegValues[3] : LEDDAR_MAX_DETECTIONS;

        for (UINT i = 0; i < 1; ++i)
//            for (UINT i = 0; i < uDetectionCount; ++i)
		{
			tabDetections[i].dDistance = (double)tabRegValues[i * 2 + 4] / 1000.0;
			tabDetections[i].dAmplitude = (double)tabRegValues[i * 2 + 5] / 256.0;
		}
	}


        else
	{
        printf("Error reading registers: numRead=%i, errno=%i\n", numRead, errno);
        return false; //failure
	}

	return true;    // success
}




// Entry point
int main(int argc, char* argv[])
{
	//Ros setup

	ros::init(argc, argv, "laser_scan_publisher");
	ros::NodeHandle n;
	ros::Publisher scan_pub = n.advertise<sensor_msgs::LaserScan>("leddaronescan", 1500);

	//ROS PARSING
	int baud_rate ;
	std::string port;
	std::string slave_id;
	char Port[512];                         //Serial and Slave data
	char Slave[128];

	n.param<std::string>("/leddarone/serial_port" , port     , "/dev/ttyUSB0");
	n.param<int>(        "/leddarone/baud_rate" , baud_rate    , 115200);;
	n.param<std::string>("/leddarone/slaveid" , slave_id    , "/dev/ttyUSB0");
  strcpy(Port , port.c_str());
	strcpy(Slave , slave_id.c_str());
	printf("Starting program  \n");

	std::cout << "The port and slave id are" << Port<< Slave << baud_rate <<'\n';
	//Connecting to the leddartech
	modbus_t* mb = nullptr;       // "handle" for the libmodbus library


	// Selects the serial modbus interface:
	mb = modbus_new_rtu(Port, baud_rate, 'N', 8, 1);
	if (mb == nullptr)
	{printf("Unable to create the libmodbus context\n");
	return 2;
}


	//modbus_set_debug(mb, true);      // uncomment to view debug info


	// Connects to the sensor:
	int counterror=0;
	while(modbus_connect(mb) != 0)
	{

		if (counterror == 5)
		{
			printf("Connection error, please, check whether the port, the slave Id \n" );
			printf("or connection of the device and permisions are correct. You can \n");
			printf("get more info by running leddaronfde -h or --help \n");
			counterror=0;
			return 2;
		}
		usleep(1200000);
		std::cout <<"Try "<< counterror << "Cant connect to the port, will retry again"<<'\n';
		counterror++;

	}


	//Declare msg variables for ros

	int count = 0;
	ros::Rate r(75);


	//Setting ip the leddarone parameters
	// Register 0 is acumulation
	// Register 1 is oversampling
	// Register 2 is base sample count

	// antes era 1 1 12

	if (!AskSlaveAddr(mb , const_cast<char*> (Slave)))
	{printf("Can't reach the device, probably your slave ID is wrong\\n");}

	if (modbus_write_register(mb, 0, 1)==1)
	{printf("Modbus parameter 1 set\n");}
	else{printf("Failed to set the mod bus parameter 1 \n");}
	usleep(500000);

	if (modbus_write_register(mb, 1, 1)==1)
	{printf("Modbus parameter 2 set \n");}
	else{printf("Failed to set the mod bus parameter 2 \n");}
	usleep(500000);

	if (modbus_write_register(mb, 2,15)==1)
	{printf("Modbus paramenter 3 set\n");}
	else{printf("Failed to set the mod bus parameter 3 \n");}


	//Ros functioning bucle

	while(n.ok()){

		ros::Time scan_time = ros::Time::now();

		//populate the LaserScan message
		sensor_msgs::LaserScan scan;
		scan.header.stamp = scan_time;
		scan.header.frame_id = "laser_frame";
		scan.angle_min = 3;
		scan.angle_max = 3;
		scan.range_min = 0.0;
		scan.range_max = 40.0;


		//Parameters needed for the sensor reading
		laserstructtype laserdata;  //This struct contains the data
		SDetection tabDetections[LEDDAR_MAX_DETECTIONS];
		UINT uDetectionCount=1;
		UINT uTimestamp;
		laserstructtype laserstruct;

		//If read detections returns true, measures were taken and we are going to grab them
		if (ReadDetections( mb, tabDetections, uDetectionCount, uTimestamp))
		{
			uDetectionCount=1;
			printf(" Distance = %.3lfm    Amplitude = %.1lf   \t",
			tabDetections[0].dDistance, tabDetections[0].dAmplitude);

			//Transfering the data to the struct
			laserstruct.distance=tabDetections[0].dDistance;
			laserstruct.intensity=tabDetections[0].dAmplitude;

			//Putting the values on the rostopic and publishing

			scan.ranges.push_back(laserstruct.distance);
			scan.intensities.push_back(laserstruct.intensity);
			scan_pub.publish(scan);
			++count;
			r.sleep();
			scan.ranges.clear();
			scan.intensities.clear();
			}


		//If readDetections not true
		else{
		printf("got no data \n");
		}
	}
}
