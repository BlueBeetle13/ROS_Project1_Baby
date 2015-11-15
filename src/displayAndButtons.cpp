#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <dirent.h>
#include <vector>
#include <string>

// ROS
#include "ros/ros.h"
#include "baby_project/tempAndHumidity.h"

// wiringPi
#include <wiringPi.h>
#include <mcp23017.h>
#include <lcd.h>

// Audio
#include <ao/ao.h>
#include <mpg123.h>
#include <alsa/asoundlib.h>
#include <math.h>
#define AUDIO_BITS		8

// Wiring PI defines for I2C
#define AF_BASE			100

#define AF_RED			(AF_BASE + 6)
#define AF_GREEN		(AF_BASE + 7)
#define AF_BLUE			(AF_BASE + 8)

#define AF_E			(AF_BASE + 13)
#define AF_RW			(AF_BASE + 14)
#define AF_RS			(AF_BASE + 15)

#define AF_DB4			(AF_BASE + 12)
#define AF_DB5			(AF_BASE + 11)
#define AF_DB6			(AF_BASE + 10)
#define AF_DB7			(AF_BASE + 9)

#define AF_SELECT		(AF_BASE + 0)
#define AF_RIGHT		(AF_BASE + 1)
#define AF_DOWN			(AF_BASE + 2)
#define AF_UP			(AF_BASE + 3)
#define AF_LEFT			(AF_BASE + 4)


// LCD display and screen
int _lcdScreen = -1;
int _displayOn = 1;

// Audio playing
pthread_t _audioThread;
int _audioThreadKill = 0;
int _audioThreadPaused = 1;

// Volume control
double _actualVolume;
int _displayVolume;
static char s_card[64] = "hw:0";
snd_mixer_t *_volumeHandle = NULL;
snd_mixer_elem_t *_volumeElem = NULL;

// Shutdown Timer
pthread_t _shutdownTimerThread;
int _shutdownTimerThreadKill = 0;
int _shutdownTimerSecondsRemaining = 0;


// Timer to count down seconds for shutdown
void* ShutdownTimerThread(void *arg)
{
	// Continue the countdown as long as the program is running
	while (_shutdownTimerThreadKill == 0)
	{
		// Decrease timer if set
		if (_shutdownTimerSecondsRemaining > 0)
		{
			_shutdownTimerSecondsRemaining --;
			printf("Seconds: %d\n", _shutdownTimerSecondsRemaining);
			
			// Did we just hit 0? Stop motor and set volume = 0
			if (_shutdownTimerSecondsRemaining == 0)
			{
				// Stop motor
				system("gpio write 7 0");
				_audioThreadPaused = 1;
				
				// Update LCD
				lcdPosition(_lcdScreen, 11, 1);
				lcdPrintf(_lcdScreen, "0  ");
			}
		
			// Did we just round down to an even minute?
			else if (_shutdownTimerSecondsRemaining % 60 == 0)
			{
				// Update LCD
				lcdPosition(_lcdScreen, 11, 1);
				lcdPrintf(_lcdScreen, "%d  ", (_shutdownTimerSecondsRemaining / 60));
			}
		}
			
		delay(1000);
	}

	pthread_exit(NULL);

	return NULL;
}


// Play the audio tracks repeatedly
void* PlayAudioThread(void *arg)
{
	mpg123_handle *mpgHandle;
	unsigned char *buffer;
	size_t bufferSize;
	size_t done;
	int err;

	int driver;
	ao_device *dev;
	ao_sample_format format;
	int channels, encoding;
	long rate;

	std::vector<std::string> musicList;

	// Get a list of all the songs in the /home/pi/music directory
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir("/home/pi/music")) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			if (strlen(ent->d_name) > 2)
			{
				std::string filePath = "/home/pi/music/";
				filePath.append(ent->d_name);

				printf("%s\n", filePath.c_str());
				musicList.push_back(filePath);
			}
		}

		closedir(dir);
	}


	// Init the audio
	ao_initialize();
	driver = ao_default_driver_id();
	mpg123_init();
	mpgHandle = mpg123_new(NULL, &err);
	bufferSize = mpg123_outblock(mpgHandle);
	buffer = (unsigned char *)malloc(bufferSize * sizeof(unsigned char));


	// Continually repeat through the list of songs
	std::vector<std::string>::const_iterator musicListIterator = musicList.begin();
	while (_audioThreadKill == 0 && musicListIterator != musicList.end())
	{
		printf("Track: %s\n", (*musicListIterator).c_str());

		// Open the file and get the decoding format
		mpg123_open(mpgHandle, (*musicListIterator).c_str());
		mpg123_getformat(mpgHandle, &rate, &channels, &encoding);

		// Set the output format and open the output device
		format.bits = mpg123_encsize(encoding) * AUDIO_BITS;
		format.rate = rate;
		format.channels = channels;
		format.byte_format = AO_FMT_NATIVE;
		format.matrix = 0;
		dev = ao_open_live(driver, &format, NULL);

		// Decode and play as long as we aren't shutting down
		while (_audioThreadKill == 0 && mpg123_read(mpgHandle, buffer, bufferSize, &done) == MPG123_OK)
		{
			if (_audioThreadPaused == 0)
				ao_play(dev, (char *)buffer, done);
		}

		musicListIterator ++;

		if (musicListIterator == musicList.end())
			musicListIterator = musicList.begin();
	}

	// Clean up
	free(buffer);
	ao_close(dev);
	mpg123_close(mpgHandle);
	mpg123_delete(mpgHandle);
	mpg123_exit();
	ao_shutdown();


	ROS_INFO("Audio Thread Finished");

	pthread_exit(NULL);

	return NULL;
}


// Init volume
void InitVolumeControl()
{
	snd_mixer_selem_id_t *sid;
	snd_mixer_selem_id_alloca(&sid);
	snd_mixer_selem_id_set_index(sid, 0);
	snd_mixer_selem_id_set_name(sid, "PCM");

	if (snd_mixer_open(&_volumeHandle, 0) < 0)
	{
		ROS_INFO("Error opening volume mixer");
		return;
	}

	if (snd_mixer_attach(_volumeHandle, s_card) < 0)
	{
		ROS_INFO("Error attaching mixer");
		snd_mixer_close(_volumeHandle);
		return;
	}

	if (snd_mixer_selem_register(_volumeHandle, NULL, NULL) < 0)
	{
		ROS_INFO("Error registering mixer");
		snd_mixer_close(_volumeHandle);
		return;
	}

	if (snd_mixer_load(_volumeHandle) < 0)
	{
		ROS_INFO("Error loading mixer");
		snd_mixer_close(_volumeHandle);
		return;
	}

	_volumeElem = snd_mixer_find_selem(_volumeHandle, sid);
	if (!_volumeElem)
	{
		ROS_INFO("Error finding volume control");
		snd_mixer_close(_volumeHandle);
	}
}

double GetCurrentVolume()
{
	long max, min, value;
	int err = -1;

	// Volume Range
	err = snd_mixer_selem_get_playback_dB_range(_volumeElem, &min, &max);

	if (err >= 0)
	{
		// Current volume
		err = snd_mixer_selem_get_playback_dB(_volumeElem, (snd_mixer_selem_channel_id_t)0, &value);
		printf("V: %ld", value);

		if (err >= 0)
		{
			// Volume is logarithmic, convert to value of 0.0 -> 1.0
			return exp10((value - max) / 6000.0);
		}
	}

	// Fail
	return 0;
}

void SetVolume(double newVolume)
{
	long min, max, value;
	int err= -1;

	if (newVolume < 0.017170)
		newVolume = 0.017170;
	else if (newVolume > 1.0)
		newVolume = 1.0;

	err = snd_mixer_selem_get_playback_dB_range(_volumeElem, &min, &max);
	if (err >= 0)
	{
		value = lrint(6000.0 * log10(newVolume)) + max;
		snd_mixer_selem_set_playback_dB(_volumeElem, (snd_mixer_selem_channel_id_t)0, value, 0);
	}
}


// Custom degree symbol character
static unsigned char degreeChar[8] =
{
0b01000,
0b10100,
0b01000,
0b00000,
0b00000,
0b00000,
0b00000,
0b00000
};

// Custom timer symbol character
static unsigned char timerChar[8] =
{
0b11111,
0b10001,
0b01010,
0b00100,
0b00100,
0b01010,
0b10001,
0b11111
};



// LCD Display Toggle screen on/off
void ToggleDisplay(int  displayOn)
{
	digitalWrite(AF_RED, !displayOn);
	digitalWrite(AF_GREEN, !displayOn);
	digitalWrite(AF_BLUE, !displayOn);
}


// Callback for receiving Temperature and Humidity measurements
void TempAndHumidityFeedCallback(const baby_project::tempAndHumidity::ConstPtr& msg)
{
	ROS_INFO("Callback: %.1f - %.1f", msg->temperature, msg->humidity);

	// Clear the separator spaces
	lcdPosition(_lcdScreen, 7, 0);
	lcdPuts(_lcdScreen, "  ");

	// Temperature
	lcdPosition(_lcdScreen, 2, 0);
	lcdPrintf(_lcdScreen, "%.1f", msg->temperature);
	lcdPutchar(_lcdScreen, 2);

	// Humidity
	lcdPosition(_lcdScreen, 11, 0);
	lcdPrintf(_lcdScreen, "%.1f%%", msg->humidity);
}


// Node shutdown code
void NodeShutdown(int sig)
{
	ROS_INFO("Display and Buttons Node - Shutdown");

	// Kill the Audio Thread and wait a little for it to die
	_audioThreadKill = 1;
	delay(1000);

	if (_volumeHandle != NULL)
		snd_mixer_close(_volumeHandle);
		
	// Kill the Shutdown Timer Thread
	_shutdownTimerThreadKill = 1;
	delay(1000);
	
	// Stop the motor
	system("gpio write 7 0");


	// Clear and turn off lcd display
	lcdClear(_lcdScreen);
	ToggleDisplay(0);


	ros::shutdown();
}


int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "displayAndButtons");
	ros::NodeHandle nodeHandle;

	signal(SIGINT, NodeShutdown);

	// ROS - Subscribe to temp and humidity messages
	ros::Subscriber tempAndHumiditySubscriber = nodeHandle.subscribe("tempAndHumidityFeed", 1, TempAndHumidityFeedCallback);



	// Setup the WiringPi library
	if (wiringPiSetupSys() == -1)
	{
		ROS_INFO("wiringPi setup failed");
		return -1;
	}
	
	// Init the shutdown for the voltage regulator (which controls the motor)
	system("gpio mode 7 out");
	system("gpio write 7 0");
	pthread_create(&_shutdownTimerThread, NULL, &ShutdownTimerThread, NULL);
	

	// Init the I2C expander chip
	mcp23017Setup(AF_BASE, 0x20);

	// Init the output pins to control the display
	pinMode(AF_RED, OUTPUT);
	pinMode(AF_GREEN, OUTPUT);
	pinMode(AF_BLUE, OUTPUT);
	ToggleDisplay(1);
	_displayOn = 1;

	// Init the control pins
	pinMode(AF_SELECT, INPUT);
	pullUpDnControl(AF_SELECT, PUD_UP);

	pinMode(AF_UP, INPUT);
	pullUpDnControl(AF_UP, PUD_UP);

	pinMode(AF_DOWN, INPUT);
	pullUpDnControl(AF_DOWN, PUD_UP);

	pinMode(AF_LEFT, INPUT);
	pullUpDnControl(AF_LEFT, PUD_UP);

	pinMode(AF_RIGHT, INPUT);
	pullUpDnControl(AF_RIGHT, PUD_UP);

	pinMode(AF_RW, OUTPUT);
	digitalWrite(AF_RW, LOW);

	// Init the screen
	_lcdScreen = lcdInit(2, 16, 4, AF_RS, AF_E, AF_DB4, AF_DB5, AF_DB6, AF_DB7, 0, 0, 0, 0);
	if (_lcdScreen < 0)
	{
		ROS_INFO("Failed to Init LCD");
		return -1;
	}

	ROS_INFO("Node started");

	// Clear the screen
	lcdClear(_lcdScreen);

	// Define the new degree char
	lcdCharDef(_lcdScreen, 2, degreeChar);
	lcdCharDef(_lcdScreen, 3, timerChar);

	// Init the Display
	lcdPosition(_lcdScreen, 0, 0);
	lcdPuts(_lcdScreen, "T:");

	lcdPosition(_lcdScreen, 9, 0);
	lcdPuts(_lcdScreen, "H:");
	lcdPosition(_lcdScreen, 0, 1);
	lcdPuts(_lcdScreen, "V:");

	lcdPosition(_lcdScreen, 9, 1);
	lcdPutchar(_lcdScreen, 3);
	lcdPosition(_lcdScreen, 10, 1);
	lcdPuts(_lcdScreen, ":");


	// Init the Volume control
	InitVolumeControl();
	_actualVolume = 0.5;
	_displayVolume = 50;
	SetVolume(_actualVolume);
	printf("Volume: %f", GetCurrentVolume());

	// Display the volume
	lcdPosition(_lcdScreen, 2, 1);
	lcdPrintf(_lcdScreen, "%d%% ", _displayVolume);

	// Create the Audio thread
	int err = pthread_create(&_audioThread, NULL, &PlayAudioThread, NULL);


	//  Allows faster looping and only accepts one button press while the button is down
	bool displayButtonDown = false;
	bool volumeUpButtonDown = false;
	bool volumeDownButtonDown = false;
	bool timerIncreaseButtonDown = false;
	bool timerDecreaseButtonDown = false;

	// Loop until exist
	while (ros::ok())
	{
		// Check the state of the buttons

		// Display On/Off (SELECT)
		if (!displayButtonDown && digitalRead(AF_SELECT) == LOW)
		{
			ROS_INFO("Display Toggle");

			displayButtonDown = true;
			_displayOn = !_displayOn;
			ToggleDisplay(_displayOn);
		}
		else if (displayButtonDown && digitalRead(AF_SELECT) == HIGH)
		{
			displayButtonDown = false;
		}


		// Volume Up (UP)
		if (!volumeUpButtonDown && digitalRead(AF_UP) == LOW)
		{
			ROS_INFO("Volume Up");

			volumeUpButtonDown = true;

			// Update volume
			_displayVolume += 5;
			if (_displayVolume > 100)
				_displayVolume = 100;
			_actualVolume = ((double)(_displayVolume)) / 100.0;
			SetVolume(_actualVolume);

			// Update display
			lcdPosition(_lcdScreen, 2, 1);
			lcdPrintf(_lcdScreen, "%d%% ", _displayVolume);
		}
		else if (volumeUpButtonDown && digitalRead(AF_UP) == HIGH)
		{
			volumeUpButtonDown = false;
		}


		// Volume Down (DOWN)
		if (!volumeDownButtonDown && digitalRead(AF_DOWN) == LOW)
		{
			ROS_INFO("Volume Down");

			volumeDownButtonDown = true;

			// Update volume
			_displayVolume -= 5;
			if (_displayVolume < 0)
				_displayVolume = 0;
			_actualVolume = ((double)(_displayVolume)) / 100.0;
			SetVolume(_actualVolume);

			// Update display
			lcdPosition(_lcdScreen, 2, 1);
			lcdPrintf(_lcdScreen, "%d%% ", _displayVolume);
		}
		else if (volumeDownButtonDown && digitalRead(AF_DOWN) == HIGH)
		{
			volumeDownButtonDown = false;
		}


		// Shutdown Timer Increase (RIGHT)
		if (!timerIncreaseButtonDown && digitalRead(AF_RIGHT) == LOW)
		{
			ROS_INFO("Shutdown Timer Increase");
			
			// If the timer was at -1, stop the motor and set to 0 (-1 means continually run)
			if (_shutdownTimerSecondsRemaining == -1)
			{
				// Zero timer
				_shutdownTimerSecondsRemaining = 0;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Pause audio
				_audioThreadPaused = 1;
			}
			
			// The timer was at 0 and we are starting, or greater than 1 and the time is being increased
			else
			{
				// If the timer was at 0, start the motor
				if (_shutdownTimerSecondsRemaining == 0)
				{
					// Start motor
					system("gpio write 7 1");
					
					// Play audio
					_audioThreadPaused = 0;
				}
			
				// Find the remainder to the nearest 5 minute interval
				int remainder = _shutdownTimerSecondsRemaining % (5 * 60);
				
				int secondsRemainingRoundedDown = _shutdownTimerSecondsRemaining - remainder;
			
				// We are within 10 seconds of a 5 minute interval
				if (remainder > ((5 * 60) - 10))
				{
					_shutdownTimerSecondsRemaining += ((5 * 60) - remainder);
					_shutdownTimerSecondsRemaining += (5 * 60);
				}
					
				// Increase to the nearest 5 minute interval
				else
					_shutdownTimerSecondsRemaining = secondsRemainingRoundedDown + (5 * 60);
			}
			
			// Update display
			lcdPosition(_lcdScreen, 11, 1);
			if (_shutdownTimerSecondsRemaining == -1)
				lcdPrintf(_lcdScreen, "Inf");
			else
				lcdPrintf(_lcdScreen, "%d  ", (_shutdownTimerSecondsRemaining - (_shutdownTimerSecondsRemaining % (5 * 60))) / 60);

			timerIncreaseButtonDown = true;
		}
		else if (timerIncreaseButtonDown && digitalRead(AF_RIGHT) == HIGH)
		{
			timerIncreaseButtonDown = false;
		}


		// Shutdown Timer Decrease (LEFT)
		if (!timerDecreaseButtonDown && digitalRead(AF_LEFT) == LOW)
		{
			ROS_INFO("Shutdown Timer Decrease");
			
			// If the timer was at -1, stop the motor and set to 0 (-1 means continually run)
			if (_shutdownTimerSecondsRemaining == -1)
			{
				// Zero timer
				_shutdownTimerSecondsRemaining = 0;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Pause audio
				_audioThreadPaused = 1;
			}
			
			// If the timer is at 0, set the -1 to continually run
			else if (_shutdownTimerSecondsRemaining == 0)
			{
				// Set timer to Inf
				_shutdownTimerSecondsRemaining = -1;
				
				// Start motor
				system("gpio write 7 1");
				
				// Play audio
				_audioThreadPaused = 0;
			}
			
			// The timer is between 0 and 5 minutes, stop
			else if (_shutdownTimerSecondsRemaining < 5 * 60)
			{
				// Zero timer
				_shutdownTimerSecondsRemaining = 0;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Pause audio
				_audioThreadPaused = 1;
			}
			
			// The timer is greater than 5 minutes, reduce to nearest 5 minutes value lower
			else
			{
				int remainder = _shutdownTimerSecondsRemaining % (5 * 60);
				
				// Within 10 seconds of the even 5 minute interval, remove the remainder and 5 minutes
				if (remainder < 10)
				{
					_shutdownTimerSecondsRemaining -= remainder;
					_shutdownTimerSecondsRemaining -= (5 * 60);
				}
				
				// Greater than 10 seconds remainder, just round down to nearest 5 minutes
				else
				{
					_shutdownTimerSecondsRemaining -= remainder;
				}
			}
			
			
			// Update display
			lcdPosition(_lcdScreen, 11, 1);
			if (_shutdownTimerSecondsRemaining == -1)
				lcdPrintf(_lcdScreen, "Inf");
			else
				lcdPrintf(_lcdScreen, "%d  ", (_shutdownTimerSecondsRemaining - (_shutdownTimerSecondsRemaining % (5 * 60))) / 60);

			timerDecreaseButtonDown = true;
		}
		else if (timerDecreaseButtonDown && digitalRead(AF_LEFT) == HIGH)
		{
			timerDecreaseButtonDown = false;
		}


		// ROS - Check if any callbacks were made
		ros::spinOnce();


		// Wait a little bit
		delay(50);
	}

	return 0;
}
