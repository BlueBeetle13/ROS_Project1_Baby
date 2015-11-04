#include <stdio.h>
#include <time.h>
#include <signal.h>

#include "ros/ros.h"
#include <wiringPi.h>
#include <softPwm.h>

void NodeShutdown(int sig)
{
	ROS_INFO("Mobile Servo Node - Shutdown");

	ros::shutdown();
}

int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "mobileServo");
	ros::NodeHandle nodeHandle;

	signal(SIGINT, NodeShutdown);

	// Init wiringPi
	if (wiringPiSetup() == -1)
	{
		ROS_INFO("Mobile Servo Node - wiringPi setup failed");
		return -1;
	}

	// Create the software PWM
	softPwmCreate(3, 0, 255);

	// Contine until killed
	int value = 0;
	while (ros::ok())
	{
		softPwmWrite(3, value);

		value ++;
		if (value == 255)
			value = 0;

		// Set the servo speed
		//softPwmWrite(3, value);
		printf("Speed: %d\n", value);

		ros::spinOnce();

		delay(5000);
	}

	return 0;
}
