#include <stdio.h>
#include <time.h>
#include <signal.h>

// ROS
#include "ros/ros.h"
#include "baby_project/tempAndHumidity.h"

// wiringPi
#include <wiringPi.h>
#include <wiringPiI2C.h>

// Wait between reading the sensor, the sensor goes into sleep mode as constantly running
// effects measurements (data sheet says constantly running creates heat), so wait between readings
// enough so that the device should stay cool
#define WAIT_BETWEEN_READINGS	10000

// Node shutdown handler
void NodeShutdown(int sig)
{
	ROS_INFO("Temperature and Humidity Sensor Node - Shutdown");

	// Shutdown ROS
	ros::shutdown();
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "tempAndHumiditySensor");
	ros::NodeHandle nodeHandle;

	// Shutdown handler
	signal(SIGINT, NodeShutdown);

	// ROS - Adverstise that we will be pushlishing messages
	ros::Publisher tempAndHumidityPublisher = nodeHandle.advertise<baby_project::tempAndHumidity>("tempAndHumidityFeed", 1);


	int tempAndHumiditySensor = -1;

	// Setup the WiringPi I2C library and connect to the sensor
	tempAndHumiditySensor = wiringPiI2CSetup(0x5c);
	if (tempAndHumiditySensor  == -1)
	{
		ROS_INFO("Temperature and Humidity Sensor Node - wiringPi setup failed");
		return -1;
	}

	// Wake from sleep data
	unsigned char wakeFromSleepData[1] = {0};

	// Read Request data - {read register command, start address, # of bytes to read
	unsigned char readRequestData[3] = {3, 0, 4};

	// Response buffer - command byte, length byte, 4 bytes for response, 2 bytes checksum
	unsigned char responseData[8];

	ROS_INFO("Temperature and Humidity Sensor Node - started");

	// Continue until killed
	while (ros::ok())
	{
		int ret = -1;

		// Send the wakeup data twice to wake it up
		ret = write(tempAndHumiditySensor, wakeFromSleepData, 1);
		ret = write(tempAndHumiditySensor, wakeFromSleepData, 1);

		// The sensor should have waken up now, send the read request
		ret = write(tempAndHumiditySensor, readRequestData, 3);

		// A small delay to allow the sensor to work
		delay(5);

		// Read the response into the buffer
		ret = read(tempAndHumiditySensor, responseData, 8);

		// Improper response
		if (ret != 8 || responseData[0] != 3 || responseData[1] != 4)
		{

			ROS_INFO("Temperature and Humidity Sensor Node - invalid response: %02x %02x %02x %02x %02x %02x %02x %02x\n",
				responseData[0],
				responseData[1],
				responseData[2],
				responseData[3],
				responseData[4],
				responseData[5],
				responseData[6],
				responseData[7]);
		}

		// Proper response
		else
		{
			// Calculate the Temperature
			float temperature = (float)((256 * (responseData[4] & 0x7F)) + responseData[5]) / 10.0;

			// Negate if the sign bit is set
			if (responseData[4] >> 7)
				temperature = -temperature;

			// Calculate the Humidity
			float humidity = (float)((256 * responseData[2]) + responseData[3]) / 10.0;

			ROS_INFO("Temp: %.1f\t\tHum: %.1f\n", temperature, humidity);

			// Only send if the data is reasonable, filter data anomalies
			if (temperature != 0.0 && temperature != -55.0)
			{
				// Create the message to publish
				baby_project::tempAndHumidity tempAndHumidityMsg;
				tempAndHumidityMsg.temperature = temperature;
				tempAndHumidityMsg.humidity = humidity;

				// Publish the message
				tempAndHumidityPublisher.publish(tempAndHumidityMsg);
			}
		}

		ros::spinOnce();

		delay(5000);
	}

	return 0;
}
