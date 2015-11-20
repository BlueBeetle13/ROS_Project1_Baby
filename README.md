ROS - Project1 - Baby Device
============================

Baby temperature, humidity, music, mobile project for raspberry pi - written with ROS

Purpose
-------

There were 2 main purposes for working on this project. 

The first was to create a device for my baby son (at the time of this writing still unborn) that would give temperature and humidity readings of the area around the crib. My wife wanted to buy a device for this but I thought it would be more fun to make one. Also we wanted a mobile for the crib and were disapointed that most only last 20-40 minutes and thought this might be too short. So another function of the device is to control the motor on a mobile and play music. As an added bonus we could supply and change the music rather than being stuck with whatever the mobile company wanted to give us.

The second purpose is to develope with ROS (Robot Oerating System). I have recently started learning ROS, and having completed all of the tutorials I wanted to apply this knowledge to a project (as I find this provides a better way of retaining this information).


Description
-----------

For this project I need small computer that can run ROS, a sensor to give temperature and humidity readings, audio ouput for the sound, and a motor to spin the mobile.


Design
------

Since I will be developing with ROS, I am using an Raspberry Pi (in this case version 1, model B) with Rasbian Jessy installed as my computer.

Instead of trying to make a mobile myself (which I briefly considered) I bought a used mobile to modify. Additionaly my wife will be making the hanging ornaments.

I am using an AM2315 temperature and humidity sensor (https://www.adafruit.com/products/1293) which communicates over I2C. I will be using the wiringPi (http://wiringpi.com/) library to control the GPIO pins and the AM2315 datasheet provides information on how to interface with the device over I2C. 

In order to display information to the user I am using a 16x2 LCD screen (https://www.adafruit.com/products/1115) and as luck would have it, the wiringPi library contains code for this exact device. The LCD will display the temperature, humidity, current volume, and a shutdown timer. The LCD uses I2C as well as the sensor and also has 5 buttons which I will use.

I'll use the audio output from the RPi to connect to the mobile speaker for playing music, and play the music and control the voume through software libraries. I will just download MP3's, place them in a folder, and have the code scan the folder and create a playlist.

To control the motor I will use a voltage regulator to supply the correct voltage in order to produce the correct speed. The voltage regulator also have a shutdown pin that I can tie to one of the GPIO pins in order to stop the motor.

