//============================================================================
// Name        : unldaemon.cpp
// Author      : Verifynd (Madhu, Alex)
// Version     :
// Copyright   : Copyright (C) Verifynd - All Rights Reserved
//			   : Unauthorized copying of this file, via any medium is strictly prohibited
//			   : Proprietary and confidential
// Description : Main entry point for the unldaemon process
//============================================================================
#include <stdio.h>
#include <getopt.h>
#include <signal.h>
#include <stdlib.h>
#include "rc/mpu.h"
#include "rc/time.h"
#include <math.h>
#include "rc/led.h"
#include "rc/start_stop.h"
#include "rc/button.h"
#include "rc/gpio.h"
#include <time.h>

#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string.h>
#include <fstream>
#include <unistd.h>
#include <vector>

#include "monitor/gpsmonitor.h"
#include "threading/thread.h"
#include "board/gps.h"
#include "board/gpio.h"
#include "board/serial.h"
#include "rfid/CFHidApi.h"
#include "sys/log.h"
using namespace std;
using namespace BB;

#define FALSE 0
#define TRUE  1
#define ANSWERMODE 0
#define ACTIVEMODE 1
#define CSVFILE "/home/debian/bin/unldaemon_data_new.csv"

// bus for Robotics Cape and BeagleboneBlue is 2, interrupt pin is on gpio3.21
#define I2C_BUS 2
#define GPIO_INT_PIN_CHIP 3
#define GPIO_INT_PIN_PIN  21
#define RC_BTN_GROVE 3,1
#define RC_EXTERNAL_LED 1,17
#define QUIT_TIMEOUT_US 1500000 // quit after 1.5 seconds holding pause button
#define QUIT_CHECK_US   100000  // check every 1/10 second


static const int us_delay[] = {400000, 170000, 100000};
static int mode;

void CallBackFunc(int msg, int param1, unsigned char *param2, int param3,
		unsigned char *param4);
void get_local_time(char *output);
double convert_to_heading(rc_mpu_data_t *sensor_data);
int calibrate_mpu(void);

typedef string arg_t[2];
static const arg_t ANTENNA_MODE = {"rfid", "Mode to run RFID antenna, valid modes are \"OFF\", \"ANSWER\", or \"ACTIVE\""};
static const arg_t GPS_MODE = {"gps", "Mode to run GPS, options are \"OFF\", or port where it is connected eg. \"/dev/ttyS1\""};
static const arg_t ARDUINO_MODE = {"arduino", "Mode to run Arduino, options are \"OFF\" or port eg. \"/dev/ttyS5\""};
static const arg_t MAG_MODE = {"mag", "Mode to run magnetometer, options are \"OFF\" or \"ON\""};
static const arg_t ACCEL_MODE = {"accel", "Mode to run accelerometer, options are \"OFF\" or \"ON\""};
static const arg_t CALIBRATE_MODE = {"--calibrate", "Execute the calibration functions for the onboard sensors"};
static const arg_t VERSION = {"--version", "Mode to run accelerometer, options are \"OFF\" or \"ON\""};
static const arg_t HELP_MSG = {"--help", "Mode to run accelerometer, options are \"OFF\" or \"ON\""};

static void __on_long_press(void)
{
        int i=0;
        const int samples = QUIT_TIMEOUT_US/QUIT_CHECK_US;
        // now keep checking to see if the button is still held down
        for(i=0;i<samples;i++){
                rc_usleep(QUIT_CHECK_US);
                if(rc_button_get_state(RC_BTN_GROVE)==RC_BTN_STATE_RELEASED){
                	log_info("Short press detected toggling state");
                	if(rc_get_state()==RUNNING){
                		log_info("Setting state to PAUSED");
                		rc_set_state(PAUSED);
                		return;
                	}
                	else{
                		log_info("Setting state to RUNNING");
                		rc_set_state(RUNNING);
                		return;
                	}

                }
        }
        log_info("long press detected, setting state to EXITING");
        rc_set_state(EXITING);
        return;
}

static void __on_long_press_mode(void)
{
        int i=0;
        const int samples = QUIT_TIMEOUT_US/QUIT_CHECK_US;
        // now keep checking to see if the button is still held down
        for(i=0;i<samples;i++){
                rc_usleep(QUIT_CHECK_US);
                if(rc_button_get_state(RC_BTN_PIN_MODE)==RC_BTN_STATE_RELEASED){
                	log_info("Short press detected toggling state");
                	if(rc_get_state()==RUNNING){
                		log_info("Setting state to PAUSED");
                		rc_set_state(PAUSED);
                		return;
                	}
                	else{
                		log_info("Setting state to RUNNING");
                		rc_set_state(RUNNING);
                		return;
                	}

                }
        }
        log_info("long press detected, setting state to EXITING");
        rc_set_state(EXITING);
        return;
}

int main(int argc, char *argv[]) {

	// check if the process is already running
    if(rc_kill_existing_process(2.0)<-2){
    	return -1;
    }

	if (rc_enable_signal_handler() != 0){
		log_error("Unable to initialize the signal handler tearing down application ...");
		return -1;
	}

	// set up defaults for command line args
	string rfid_mode = "OFF";
	string gps_mode = "OFF";
	string arduino_mode = "OFF";
	string mag_mode = "OFF";
	string accel_mode = "OFF";
	int sensor_count = 0;
	// parse command line args and setup unldaemon process
	for(int i=1; i < argc; i++){
		string arg = argv[i];
		if(arg.empty() == false){
			// got a non empty arg need to parse this
			// must have an '='
			// split on the '='

			const size_t eqPos = arg.find('=');
			string key = arg.substr(0, eqPos);
			// compare key to known arguments
			if (CALIBRATE_MODE[0].compare(key) == 0){
				printf("Calibration mode selected calibrating mpu on beaglebone ....\n");
				if(calibrate_mpu() != 0){
					log_info("ERROR ::: Failed to calibrate MPU ");
				}
				exit(0);
			}

			else if (ANTENNA_MODE[0].compare(key) == 0){
				string val = arg.substr(eqPos + 1, arg.npos);
				// add some sort of error checking here to eliminate bad arguments
				rfid_mode = val;
				sensor_count++;
			}
			else if (GPS_MODE[0].compare(key) == 0){
				string val = arg.substr(eqPos + 1, arg.npos);
				// add some sort of error checking here to eliminate bad arguments
				gps_mode = val;
				sensor_count++;
			}

			else if (ARDUINO_MODE[0].compare(key) == 0){
				string val = arg.substr(eqPos + 1, arg.npos);
				// add some sort of error checking here to eliminate bad arguments
				arduino_mode = val;
				sensor_count++;
			}

			else if (MAG_MODE[0].compare(key) == 0){
				string val = arg.substr(eqPos + 1, arg.npos);
				// add some sort of error checking here to eliminate bad arguments
				mag_mode = val;
				sensor_count++;
			}

			else if (ACCEL_MODE[0].compare(key) == 0){
				string val = arg.substr(eqPos + 1, arg.npos);
				// add some sort of error checking here to eliminate bad arguments
				accel_mode = val;
				sensor_count++;
			}

		}
	}

	// can add some check to ensure some amount of sensors are enabled if none are enabled then exit with error
	if(sensor_count == 0){
		printf("ERROR ::: At least one sensor must be activated to run the unldaemon, tearing down application\n");
		exit(-1);
	}
	// setup logging
	log_set_level(LOG_DEBUG);
	FILE *logfile;
	logfile = fopen("/home/debian/bin/system-log", "a");
	log_add_fp(logfile, LOG_DEBUG);

	rc_mpu_data_t sensor_data;

	// setup each sensor that is not OFF
	if(rfid_mode != "OFF"){
		if(CFHid_OpenDevice() == FALSE)
		{
			log_error("Unable to use CFHid_OpenDevice");
			exit(-1);
		}
		unsigned char bValue;
		unsigned char bParamAddr = 0x02;

		// antenna is open now set the appropriate mode
		if(rfid_mode == "ANSWER"){
			bValue = ANSWERMODE;
			log_info("Setting device in AnswerMode");
			if (CFHid_SetDeviceOneParam(0xFF,bParamAddr, bValue) == FALSE){
				log_error("Failed to write the workmode parameter");
				return 0;
			}
		}
		else if(rfid_mode == "ACTIVE"){
			log_info("Setting device in ActiveMode");
			bValue = ACTIVEMODE;
			if (CFHid_SetDeviceOneParam(0xFF,bParamAddr, bValue) == FALSE){
				log_error("Failed to write the workmode parameter");
				return 0;
			}
		}

		CFHid_SetCallback(CallBackFunc);
	}
	else {
		log_info("rfid_mode is OFF, setting up sensors to read on time interrupts not on antenna reads");

	}

	if(gps_mode != "OFF"){
		log_info("gps_mode is %s, setting up device ...", gps_mode.c_str());
	}
	if(arduino_mode != "OFF"){
		log_info("arduino_mode is %s, setting up device ...", arduino_mode.c_str());
		// need to abstract this to an arduino monitor, constructor would need the path to where arduino is connected
	}
	if(mag_mode != "OFF"){
		log_info("mag_mode is %s, setting up device ...", mag_mode.c_str());
		// is there a test case where we want just the mag but not accel?  If not we should just handle setup of both sensors in one block
	}
	if(accel_mode != "OFF"){
		log_info("accel_mode is %s, setting up device ...", accel_mode.c_str());
        rc_mpu_config_t conf = rc_mpu_default_config();
        conf.i2c_bus = I2C_BUS;
        conf.gpio_interrupt_pin_chip = GPIO_INT_PIN_CHIP;
        conf.gpio_interrupt_pin = GPIO_INT_PIN_PIN;
        conf.dmp_fetch_accel_gyro=1;
        conf.enable_magnetometer = 1;
        rc_mpu_initialize_dmp(&sensor_data, conf);

	}
	// setup the external button for managing process state
	if(rc_button_init(RC_BTN_GROVE, RC_BTN_POLARITY_NORM_LOW, RC_BTN_DEBOUNCE_DEFAULT_US)){
		log_error("Failed to initialize the external GPIO button");
	}
	else{
		rc_button_set_callbacks(RC_BTN_GROVE, __on_long_press, NULL);
	}

    if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US)){
		log_error("Failed to initialize the internal MODE button");
        return -1;
    }

	rc_button_set_callbacks(RC_BTN_PIN_MODE, __on_long_press_mode, NULL);
    // setup external LED for indicating process state
//    if(rc_gpio_init(RC_EXTERNAL_LED, GPIOHANDLE_REQUEST_OUTPUT)){
//    	log_error("Failed to initialize the external variable LED");
//    	return -1;
//    }
	log_info("Device setup complete setting process state to RUNNING");
	rc_make_pid_file();
	rc_set_state(RUNNING);
//	rc_gpio_set_value(RC_EXTERNAL_LED, 1);
	rc_led_set(RC_LED_GREEN,1);
	rc_led_set(RC_LED_RED,0);

	char currTime[30] = {0};
	double heading, latitude, longitude;
	Gps gps(gps_mode.c_str());
	while(rc_get_state() != EXITING){
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_RED,0);
			rc_led_set(RC_LED_GREEN,1);
			if(rfid_mode != "OFF"){
				CFHid_StartRead(0xFF);
			}
			else{
				// currently aquiring file pointer and writing to csv file on each write.  This will be changed in later build.
				FILE *datafile;
				datafile = fopen(CSVFILE, "a");
				log_info("Reading gps location ... ");
				gps.get_location(&longitude, &latitude);
				log_info("Found gps location!");
				log_info("The board location is (%f,%f)", latitude, longitude);
				get_local_time(currTime);
				fprintf(datafile, "%s,%f,%f", currTime, latitude, longitude);

				if(accel_mode == "ON"){
					if(rc_mpu_read_accel(&sensor_data) != 1){
						log_info("Read sensor_data from acclerometer ...");
						fprintf(datafile, ",%lf,%lf,%lf", sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2]);
					}
					if(mag_mode == "ON"){
						log_info("Read sensor_data from magnetometer ...");
						heading = convert_to_heading(&sensor_data);
						fprintf(datafile, ",%lf", heading);
					}
				}

				fprintf(datafile, "\n");

				fclose(datafile);
			}
		} else if (rc_get_state() == PAUSED) {
			log_info("unldaemon is PAUSED ...");
			rc_led_set(RC_LED_GREEN,0);
			rc_led_set(RC_LED_RED,1);
			if(rfid_mode != "OFF"){
				CFHid_StopRead(0xFF);
			}
		}
		rc_usleep(us_delay[mode]);
	}
	log_info("Turning off LED");
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,0);

	// if any other thread deadlocks then the watchdog thread will never own the processor and
	// will therefore never write to our own watchdog file

	// join each thread in thread container (might not be able to join everything since a thread could be stuck)
	//
	// clean up logging thread

//	if(rfid_mode != "OFF"){
//		CFHid_StartRead(0xFF);
//
//		// wait for application to be closed
//		sleep(30);
//		CFHid_StopRead(0xFF);
//
//		log_info("Closing USB device ...");
//		CFHid_CloseDevice();
//	}

	fclose(logfile);
	rc_button_cleanup();
	if(rfid_mode != "OFF"){
		log_info("Closing USB device ...");
		CFHid_CloseDevice();
	}
//	rc_gpio_cleanup(RC_EXTERNAL_LED);
	rc_remove_pid_file();
	return 0;
}

// this can be added to a MPU Monitor thread class I think
double convert_to_heading(rc_mpu_data_t *sensor_data){
	double heading = 180 * atan2(sensor_data->mag[0], sensor_data->mag[1])/3.14;
	if(heading < 0){
		heading += 360;
	}
	return heading;

}

// move this to a antenna monitor thread class
void CallBackFunc(int msg, int param1, unsigned char *param2, int param3,
		unsigned char *param4) {
	if (msg == 2){ //Data
		log_info("Entered callback function reading data from antenna ...");
		//unsigned short iTagLength = 0;
		unsigned short iTagNumber = 0;
		//iTagLength = param3;  //
		iTagNumber = param1;  //
		unsigned char *pBuffer = NULL;
		pBuffer = (unsigned char*) param2;
		//param4

		if (iTagNumber == 0)
			return;
		int iIndex = 0;
		int iLength = 0;
		unsigned char *pID;
		unsigned char bPackLength = 0;

		char buf[512] = { 0 };

		for (iIndex = 0; iIndex < iTagNumber; iIndex++) {
			bPackLength = pBuffer[iLength];
			pID = (unsigned char*) &pBuffer[1 + iLength];
			string str1 = "", str2 = "", strTemp = "";

			sprintf(buf, "Type:%.2X ", pID[0]);
			strTemp = buf;

			str2 = str2 + strTemp;  //Tag Type

			sprintf(buf, "Ant:%.2X Tag:", pID[1]);
			strTemp = buf;
			string tagid = "";
			str2 = str2 + strTemp;  //Ant
			for (int i = 2; i < bPackLength - 1; i++) {
				sprintf(buf, "%.2X ", pID[i]);
				str1 = buf;
				tagid = tagid + str1;
				str2 = str2 + str1;
			}
			sprintf(buf, "RSSI:%.2X", pID[bPackLength - 1]);
			strTemp = buf;
			string rssi = "";
			rssi = rssi + strTemp;
			str2 = str2 + strTemp; //RSSI
			char *c = const_cast<char*>(str2.c_str());
			char *tagid_c = const_cast<char*>(tagid.c_str());
			char *rssi_c = const_cast<char*>(rssi.c_str());
			log_info(c);

			// get local time to timestamp reading
			char currTime[20] = {0};
			get_local_time(currTime);
			// write to result file (replace this with queue later to be managed by separate thread)

			// getting a file pointer for each read is super slow
			// this needs to be handled by a separate thread and every time this callback function
			// executes it signals the thread that is waiting on antenna data
			// for now (progress update for November) leave this as is but needs to be changed asap

			rc_mpu_data_t sensor_data;
			FILE *datafile;
			datafile = fopen(CSVFILE, "a");
			get_local_time(currTime);
			fprintf(datafile, "%s,%s,%s", currTime, tagid_c, rssi_c);

			if(rc_mpu_read_accel(&sensor_data) != 1){
				log_info("Read sensor_data from acclerometer ...");
				fprintf(datafile, ",%lf,%lf,%lf", sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2]);
				log_info("Read sensor_data from magnetometer ...");
				double heading = convert_to_heading(&sensor_data);
				fprintf(datafile, ",%lf", heading);
			}

			fprintf(datafile, "\n");
			fflush(datafile);
			log_info("Closing file pointer ... ");
			if(fclose(datafile) == 0){
				log_info("Data file closed successfully");
			}
			else{
				log_error("Error when trying to close the data file");
			}
			iLength = iLength + bPackLength + 1;
		}
	} else if (msg == 1) //Device Out
			{

		log_error("No Device");
	} else if (msg == 0) //Device Insert
			{
		log_info("Device Insert");
	}
}

void test_arduino_connection(void){
	serial_init("/dev/ttyS1");
	serial_config(9600);
	sleep(3);             // give the Arduino a chance to respond
	char receive[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //declare a buffer for receiving data
	serial_readln(receive, 6);
	float f;
	sscanf(receive, "%f", &f);
	printf("The following was read in [1]: %f\n", f);

	serial_close();
}

// move this to utils?
void get_local_time(char *output){
	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );

	sprintf(output, "[%d %d %d %d:%d:%d]",timeinfo->tm_mday, timeinfo->tm_mon + 1, timeinfo->tm_year + 1900, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
}

// definitely move this to the MPU monitor class
int calibrate_mpu(void){
    rc_mpu_config_t config = rc_mpu_default_config();
    config.i2c_bus = I2C_BUS;
    printf("\nThis program will generate a new accelerometer calibration file\n");
    printf("Starting calibration routine\n");
    if(rc_mpu_calibrate_accel_routine(config)<0){
            printf("Failed to complete accelerometer calibration\n");
            return -1;
    }
    printf("\nacceleometer calibration file written\n");
    printf("\nSleeping for 10 seconds ...");

    sleep(10);
    printf("\nThis program will generate a new gyro calibration file\n");
    printf("keep your board very still for this procedure.\n");
    if(rc_mpu_calibrate_gyro_routine(config)<0){
    	printf("Failed to complete gyro calibration\n");
        return -1;
    }
    printf("\ngyro calibration file written\n");
    printf("\nSleeping for 10 seconds ...");
    sleep(10);
    printf("\n");
    printf("This will sample the magnetometer for the next 15 seconds\n");
    printf("Rotate the board around in the air through as many orientations\n");
    printf("as possible to collect sufficient data for calibration\n");
    printf("spin spin spin!!!\n\n");
    // wait for the user to actually start
    sleep(5);
	if(rc_mpu_calibrate_mag_routine(config)<0){
		   printf("Failed to complete magnetometer calibration\n");
		   return -1;
	}

    printf("\nmagnetometer calibration file written\n");
    return 0;
}
