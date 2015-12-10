ROS - Project1 - Baby Device
============================

Baby mobile project featuring temperature gauge, humidity measure and music selection for raspberry pi, written with ROS.

Purpose
-------

There were 2 main purposes for working on this project: 

1. My primary objective was to create a device for my baby son (at the time of this writing, still unborn) that would produce temperature and humidity readings of the area surrounding the crib. 1. My wife suggested we buy a device but we were disappointed to learn the units currently in market are limited in their functionality. The problem? Humidity monitors have a singular purpose and mobiles have short spin cycles and a limited music choice. The solution? Create a motorized mobile/humidity/temperature device that could achieve all our requirements. As an added bonus, we could load and play our own music versus using whatever music the manufacturer supplied.

2. Secondly, I wanted to develop using ROS (Robot Operating System). I have recently started learning ROS and have completed all of the tutorials.  I was eager to apply this knowledge to an actual project and put my training into action. I find this provides a better way of retaining information.


Description
-----------

For this project, I needed a small computer to run ROS, a sensor to produce temperature and humidity readings, audio output for the sound and a motor to spin the mobile.


Design
------

Since I will be developing with ROS, I am using a Raspberry Pi (in this case version 1, model B) with Rasbian Jessy installed on my computer.

Instead of trying to make a mobile myself (which I briefly considered), I bought a used mobile to modify. Additionally, my wife will be making the hanging ornaments.

I am using an AM2315 temperature and humidity sensor (https://www.adafruit.com/products/1293) which communicates over I2C. I will be using the wiringPi (http://wiringpi.com/) library to control the GPIO pins and the AM2315 datasheet to provide information on how to interface with the device over I2C. For the display, I am using a 16x2 LCD screen (https://www.adafruit.com/products/1115) and, as luck would have it, the wiringPi library contains code for this exact device. The LCD features 5 buttons and will display the temperature, humidity, current volume, and a shutdown timer. The LCD and sensor use I2C.

I'll use the audio output from the RPi to connect to the mobile speaker for playing music. Playing the music and controlling the volume will be achieved through software libraries. I will just download MP3's, place them in a folder, and have the code scan the folder to create a playlist.

To control the motor, I will use a voltage regulator to supply the correct voltage in order to mimic the same speed of the mobile. The voltage regulator also has a shutdown pin that I can tie to one of the GPIO pins in order to start/stop the motor.

To summarize, I will have a wall-mounted device with an LCD and 5 buttons. 1 button will turn the LCD backlight on/off. 2 buttons will control the volume, and 2 buttons will increase or decrease the shutdown timer (including an infinite mode). The device will have to connect with wires to the mobile so the wires must be securely attached to the crib so strongly that not even baby Superman could detach them. I used non-toxic, child-safe, ECO-BOND caulk to fasten the wire to the center of the wood panel.


Construction
------------

1) I started with a simple wooden box as seen below:

![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project1_Baby/babyProject1.jpg "Dollar store box")

2) The box was cut down to reduce the depth, and holes were cut for the LCD and buttons. I glued the LCD and buttons in place with a hot glue gun. You can see this in the next picture along with the temperature and humidity sensor. In the following picture you can see the wiring for the buttons. Initially, the buttons were on the board itself but I didn't like the placement so I bought some cheap buttons and moved them for a little more room.

![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project1_Baby/babyProject2.jpg "Box with LCD and buttons")
![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project1_Baby/babyProject3.jpg "Button wiring")

3) I decided to break the project into 2 nodes. 

The first node will connect to the sensor and read the temperature and humidity values. These values will be read every 10 seconds and then the values will be published on a topic. I decided to create my own custom message for this, just for the learning experience even though I could have used an existing message more easily.

The second node will control the display and buttons. The node will be responsible for outputting to the LCD display and listening for button activity. This node will subscribe to messages from the first node. When a message is received, the LCD will be updated. This node will also handle the playing of music, adjusting the volume, and adjusting the countdown timer. When the timer reaches 0 the music and motor stop, so this node will also set the GPIO pin that shuts down the voltage regulator to HIGH or LOW.

4) The next step is modifying the mobile. I needed to open the mobile and run it normally in order to find out what kind of voltage the motor in order to set the speed correctly. It turned out to be 2.47V and the voltage regulator has a dial to adjust. I will be using the 5V from the RPi to power the motor and speaker so I set the voltage regulator to output the 2.47V with 5V coming in. The speaker needed to be amplified so I broke apart an old external speaker (seen as the blue object in the first picture), and used this to connect to the speaker. In total, there are 6 wires required:

- 3 wires for the speaker (left, right, and ground)
- 2 wires for power
- 1 wire to control shutdown of the voltage regulator

You can see in the picture below I have stripped out the contents of the mobile, added the amplifier (green board) for the speaker, the voltage regulator (red board - difficult to see but to the left of the green board), and I am testing everything with alligator clips. I also wired a switch on the mobile to turn on/off the motor manually, so I could allow for music with no motor.

![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project1_Baby/babyProject4.jpg "Testing the wiring")

5) Once everything was set, I wrapped the wires, closed up the mobile and tested again. As a finishing touch, my wife painted a kid-appropriate motif on the device box. The wires were securely fastened with ECO-BOND, and we added MP3's to the music selection from our libraries. It will be interesting to see what songs he likes best!

In order to have the ROS package start when the device is powered up, I added a launch file (baby.launch) as well as a script to start ROS, set up the environment and launch the package.

```
#!  /bin/bash

cd /home/pi/catkin_ws
source devel/setup.bash
roslaunch src/baby_project/baby.launch
```

I started this script automatically by modifying the /etc/rc.local file, which I can comment out when I don't want it to run:

```
/home/pi/startupScripts/babyProject.sh &
```

Below are pictures of the completed device and mobile. So far so good and a month left before the baby arrives for further testing!

Inside the controller

![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project1_Baby/babyProject5.jpg "Inside the controller")

My wife painted the box

![alt text](http://www.typhoonsoftware.com/GitHub/ROS_Project1_Baby/babyProject6.jpg "Painted controller")


Additional Thoughts
-------------------

- I debated adding a wifi dongle to the device but decided against it because I didn't want wifi so close to the crib. Wifi uses a lot of amps and with the RPi, sound amplifier, and motor I was too close to the limit for my 1 amp power source. Also, I find the wifi on RPi's to be less than spectacular so far. This would, however, make updates to the code and music easier since I could just ssh into the RPi.

- I thought about spliting the audio into another node and having the displayAndButtons_node send messages to the audio_node to control it. That was an option to help separate the code into files with more distinct purpose.

- Once the little fellow becomes more active I might go wireless and remove the wires completely even though they are securely attached to the crib.
