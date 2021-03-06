#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <dirent.h>
#include <vector>
#include <string>
#include <math.h>

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

// Debug Mode verbose output
#define DEBUG_MODE		1


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


// LCD display and screen
int _lcdScreen = -1;
int _displayOn = 1;

// Temperature and Humidity values
float _currentTemperature = 0;
float _currentHumidity = 0;

// Audio playing
pthread_t _audioThread;
int _audioThreadKill = 0;
std::vector<std::string> _musicList;

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
bool _isTimerShutdown = true;




// ***
// Audio
// ***

void CreateMusicList()
{
	// Get a list of all the songs in the /home/pi/music directory
	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir("/home/pi/music")) != NULL)
	{
		while ((ent = readdir(dir)) != NULL)
		{
			// Get all files, not the . and .. folders
			if (strlen(ent->d_name) > 2)
			{
				std::string filePath = "/home/pi/music/";
				filePath.append(ent->d_name);

				if (DEBUG_MODE)
					printf("%s\n", filePath.c_str());

				_musicList.push_back(filePath);
			}
		}

		closedir(dir);
	}
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

	// Randomly shuffle the tracks
	for (int k = 0; k < _musicList.size(); k++)
	{
	    int r = k + rand() % (_musicList.size() - k);
	    swap(_musicList[k], _musicList[r]);
	}

	// Init the audio
	ao_initialize();
	driver = ao_default_driver_id();
	mpg123_init();
	mpgHandle = mpg123_new(NULL, &err);
	bufferSize = mpg123_outblock(mpgHandle);
	buffer = (unsigned char *)malloc(bufferSize * sizeof(unsigned char));


	// Continually repeat through the list of songs, as long as the thread is not killed
	std::vector<std::string>::const_iterator musicListIterator = _musicList.begin();
	while (_audioThreadKill == 0 && musicListIterator != _musicList.end())
	{
		if (DEBUG_MODE)
			printf("Now playing track: %s\n", (*musicListIterator).c_str());

		// Open the file and get the decoding format
		mpg123_open(mpgHandle, (*musicListIterator).c_str());
		mpg123_getformat(mpgHandle, &rate, &channels, &encoding);

		// Set the output format and open the output device
		format.bits = mpg123_encsize(encoding) * 8;
		format.rate = rate;
		format.channels = channels;
		format.byte_format = AO_FMT_NATIVE;
		format.matrix = 0;
		dev = ao_open_live(driver, &format, NULL);

		// Decode and play as long as we aren't shutting down
		while (_audioThreadKill == 0 && mpg123_read(mpgHandle, buffer, bufferSize, &done) == MPG123_OK)
		{
			// Play the audio in the buffer
			ao_play(dev, (char *)buffer, done);
		}

		// Move on to the next song
		musicListIterator ++;

		// If at the end of the playlist, repeat the playlist
		if (musicListIterator == _musicList.end())
			musicListIterator = _musicList.begin();
	}

	// Clean up and close
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
int InitVolumeControl()
{
	snd_mixer_selem_id_t *sid;
	snd_mixer_selem_id_alloca(&sid);
	snd_mixer_selem_id_set_index(sid, 0);
	snd_mixer_selem_id_set_name(sid, "PCM");

	if (snd_mixer_open(&_volumeHandle, 0) < 0)
	{
		ROS_INFO("Error opening volume mixer");
		return -1;
	}

	if (snd_mixer_attach(_volumeHandle, s_card) < 0)
	{
		ROS_INFO("Error attaching mixer");
		snd_mixer_close(_volumeHandle);
		return -1;
	}

	if (snd_mixer_selem_register(_volumeHandle, NULL, NULL) < 0)
	{
		ROS_INFO("Error registering mixer");
		snd_mixer_close(_volumeHandle);
		return -1;
	}

	if (snd_mixer_load(_volumeHandle) < 0)
	{
		ROS_INFO("Error loading mixer");
		snd_mixer_close(_volumeHandle);
		return -1;
	}

	_volumeElem = snd_mixer_find_selem(_volumeHandle, sid);
	if (!_volumeElem)
	{
		ROS_INFO("Error finding volume control");
		snd_mixer_close(_volumeHandle);
		return -1;
	}

	// Success
	return 1;
}

// Get the current device volume
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

		if (DEBUG_MODE)
			printf("Current Volume: %ld", value);

		if (err >= 0)
		{
			// Volume is logarithmic, convert to value of 0.0 -> 1.0
			return exp10((value - max) / 6000.0);
		}
	}

	// Fail
	return 0;
}

// Set the device volume
void SetVolume(double newVolume)
{
	long min, max, value;
	int err= -1;

	if (newVolume < 0.017170)
		newVolume = 0.017170;
	else if (newVolume > 1.0)
		newVolume = 1.0;

	// Get the volume range
	err = snd_mixer_selem_get_playback_dB_range(_volumeElem, &min, &max);

	if (err >= 0)
	{
		// Set volume using logarithmic scale
		value = lrint(6000.0 * log10(newVolume)) + max;
		snd_mixer_selem_set_playback_dB(_volumeElem, (snd_mixer_selem_channel_id_t)0, value, 0);
	}
}



// ***
// Shutdown Motor and Audio
// ***

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
			
			if (DEBUG_MODE)
				printf("Seconds remaining: %d\n", _shutdownTimerSecondsRemaining);

			// Did we just hit 0? Stop motor and set volume = 0
			if (_shutdownTimerSecondsRemaining == 0)
			{
				_isTimerShutdown = true;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Silence audio
				SetVolume(0.0);
				_audioThreadKill = 1;
				
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
			
		// Allow 1 second to pass
		delay(1000);
	}

	pthread_exit(NULL);

	return NULL;
}






// LCD Display Toggle back-light on/off
void ToggleDisplay(int  displayOn)
{
	digitalWrite(AF_RED, !displayOn);
	digitalWrite(AF_GREEN, !displayOn);
	digitalWrite(AF_BLUE, !displayOn);
}

// Clear the display and reset to initial display
void ClearAndInitDisplay()
{
	// Clear the screen
	lcdClear(_lcdScreen);

	// Don't blink or show the cursor
	lcdCursor(_lcdScreen, false);
	lcdCursorBlink(_lcdScreen, false);

	// Temperature
	lcdPosition(_lcdScreen, 0, 0);
	lcdPuts(_lcdScreen, "T:");

	lcdPosition(_lcdScreen, 2, 0);
	lcdPrintf(_lcdScreen, "%.1f", _currentTemperature);
	lcdPutchar(_lcdScreen, 2);


	// Humidity
	lcdPosition(_lcdScreen, 9, 0);
	lcdPuts(_lcdScreen, "H:");

	lcdPosition(_lcdScreen, 11, 0);
	lcdPrintf(_lcdScreen, "%.1f%%", _currentHumidity);


	// Volume
	lcdPosition(_lcdScreen, 0, 1);
	lcdPuts(_lcdScreen, "V:");

	lcdPosition(_lcdScreen, 2, 1);
	lcdPrintf(_lcdScreen, "%d%% ", _displayVolume);


	// Timer
	lcdPosition(_lcdScreen, 9, 1);
	lcdPutchar(_lcdScreen, 3);
	lcdPosition(_lcdScreen, 10, 1);
	lcdPuts(_lcdScreen, ":");

	lcdPosition(_lcdScreen, 11, 1);
	if (_shutdownTimerSecondsRemaining == -1)
		lcdPrintf(_lcdScreen, "Inf");
	else
		lcdPrintf(_lcdScreen, "%d  ", (_shutdownTimerSecondsRemaining / 60));
}




// Callback for receiving Temperature and Humidity measurements
void TempAndHumidityFeedCallback(const baby_project::tempAndHumidity::ConstPtr& msg)
{
	ROS_INFO("Callback: %.1f - %.1f", msg->temperature, msg->humidity);

	// Save the current values
	_currentTemperature = msg->temperature;
	_currentHumidity = msg->humidity;

	// Clear the separator spaces
	lcdPosition(_lcdScreen, 7, 0);
	lcdPuts(_lcdScreen, "  ");

	// Temperature - output to LCD
	lcdPosition(_lcdScreen, 2, 0);
	lcdPrintf(_lcdScreen, "%.1f", _currentTemperature);
	lcdPutchar(_lcdScreen, 2);

	// Humidity - output to LCD
	lcdPosition(_lcdScreen, 11, 0);
	lcdPrintf(_lcdScreen, "%.1f%%", _currentHumidity);
}


// Node shutdown handler
void NodeShutdown(int sig)
{
	ROS_INFO("Display and Buttons Node - Shutdown");

	// Kill the Audio Thread and wait a little for it to die
	_audioThreadKill = 1;
	delay(1000);

	if (_volumeHandle != NULL)
		snd_mixer_close(_volumeHandle);
		
	// Kill the Shutdown Timer Thread
	_isTimerShutdown = true;
	_shutdownTimerThreadKill = 1;
	delay(1000);
	
	// Stop the motor
	system("gpio write 7 0");


	// Clear and turn off LCD display
	lcdClear(_lcdScreen);
	ToggleDisplay(0);

	// Shutdown ROS
	ros::shutdown();
}


int main(int argc, char **argv)
{
	// Init ROS
	ros::init(argc, argv, "displayAndButtons");
	ros::NodeHandle nodeHandle;

	// Shutdown handler
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
	_isTimerShutdown = true;
	

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

	// Define the new degree and timer characters
	lcdCharDef(_lcdScreen, 2, degreeChar);
	lcdCharDef(_lcdScreen, 3, timerChar);

	// Clear and init the screen
	ClearAndInitDisplay();


	// Init the Volume control
	if (InitVolumeControl() == -1)
	{
		ROS_INFO("Failed to init Volume control");
		return -1;
	}
	_actualVolume = 0.5;
	_displayVolume = 50;
	SetVolume(0.0);

	if (DEBUG_MODE)
		printf("Current volume: %f", GetCurrentVolume());


	// Display the volume on the LCD
	lcdPosition(_lcdScreen, 2, 1);
	lcdPrintf(_lcdScreen, "%d%% ", _displayVolume);


	// Create the music list
	CreateMusicList();


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

			// About to tuen on, clear and init the screen
			if (_displayOn)
				ClearAndInitDisplay();

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
				
			// Allow stored volume setting to change, but don't change device volume if the timer is stopped
			if (!_isTimerShutdown)
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
			
			// Allow stored volume setting to change, but don't change device volume if the timer is stopped
			if (!_isTimerShutdown)
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
				_isTimerShutdown = true;
				
				// Zero timer
				_shutdownTimerSecondsRemaining = 0;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Silence audio
				SetVolume(0.0);
				_audioThreadKill = 1;
			}
			
			// The timer was at 0 and we are starting, or greater than 1 and the time is being increased
			else
			{
				// If the timer was at 0, start the motor
				if (_shutdownTimerSecondsRemaining == 0)
				{
					_isTimerShutdown = false;
					
					// Start motor
					system("gpio write 7 1");
					
					// Play audio
					SetVolume(_actualVolume);
					_audioThreadKill = 0;
					pthread_create(&_audioThread, NULL, &PlayAudioThread, NULL);
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
				_isTimerShutdown = true;
				
				// Zero timer
				_shutdownTimerSecondsRemaining = 0;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Silence audio
				SetVolume(0.0);
				_audioThreadKill = 1;
			}
			
			// If the timer is at 0, set the -1 to continually run
			else if (_shutdownTimerSecondsRemaining == 0)
			{
				_isTimerShutdown = false;
				
				// Set timer to Inf
				_shutdownTimerSecondsRemaining = -1;
				
				// Start motor
				system("gpio write 7 1");
				
				// Play audio
				SetVolume(_actualVolume);
				_audioThreadKill = 0;
				pthread_create(&_audioThread, NULL, &PlayAudioThread, NULL);
			}
			
			// The timer is between 0 and 5 minutes, stop
			else if (_shutdownTimerSecondsRemaining <= (5 * 60) + 10)
			{
				_isTimerShutdown = true;
				
				// Zero timer
				_shutdownTimerSecondsRemaining = 0;
				
				// Stop motor
				system("gpio write 7 0");
				
				// Silence audio
				SetVolume(0.0);
				_audioThreadKill = 1;
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


		// ROS - Check if any callbacks were made, can't call spin() as I need to check the state of the buttons
		ros::spinOnce();


		// Wait a little bit
		delay(50);
	}

	return 0;
}
