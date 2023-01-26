/*
 - Electronic Boat Compass
 - RK Whitehouse December 2022
 
*/

/* Functionality
* The CMPS14 sensor module provides a tilt compensated
 * compass bearing. This is output as an NMEA "HDM" message over WiFi
 * The ESP32 provides a WiFi Access Point.
 * This has two Telnet servers;
 * On port 23- the HDM messages are transmitted 5 times per second
 * On port 1024 - There is a configuration and calibration server
 *
 */

/* Software design
 *  
 *  There are two sets of methods 
 *  
 *  1. Foreground tasks - i.e. user interface tasks
 *  2. Background tasks - run at specific intervals
 *
 *  The background tasks are run by the "TaskScheduler" library without any
 *  direct user interaction
 *  This is a co-operative (non-preemptive) scheduler so needsto be called often (when not responding to user input)
 *  The scheduler dispatcher is called in the main "loop()" method
 *  
 *  The foreground tasks are called from the main loop()
 *  
 *
 * Communication between background and foreground tasks is via a set of global static
 * objects and variables
 * 
 */

/*
 * Hardware design
 * 
 * Runs on an ESP-32 devkit with a CMPS14 compass sensor module attached to the standard
 * I2C GPIO bus pins (22 & 21) 
 * 
 */


/* Imported libraries */
#include <SPI.h>
#include <Wire.h>
#include "Cmps14.h"
#include <Preferences.h>
#include <cppQueue.h>
#include <TaskScheduler.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>
#include <DNSServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

/* Local libs */
#include "NMEA.hpp"
#include "calibration.h"



#define TELNET_PORT 23       //Compass heading is output on this port
#define CONFIG_PORT 1024
#define MAX_TELNET_CLIENTS 4

#define DISPLAY_I2C_ADDRESS 0x3c //initialize with the I2C addr 0x3C Typically eBay OLED's
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET -1   //   QT-PY / XIAO

#define CMPS14_SAMPLERATE_DELAY_MS 100

Adafruit_SH1106G display = Adafruit_SH1106G(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Pre-Declare background task methods
void output();
void updateHeading();
void turnOff();
void checkWiFiClients();
void displayHeadings();


/* Declare Global Singleton Objects */

// Non-volatile settings - will be restored on power-up
Preferences settings;

// Background tasks
Task outputTask(200, TASK_FOREVER, &output);              // do output every 200 milliseconds
Task updateHeadingTask(200, TASK_FOREVER, &updateHeading); //Read sensor twice per second
Task checkWiFiClientsTask(2000, TASK_FOREVER, &checkWiFiClients); //Check WiFi client connections every 2 secs
Task updateOLEDTask(300, TASK_FOREVER, &displayHeadings); //Update OLED display

//Background task scheduler
Scheduler runner;

unsigned short  boatHeading = 0; //Heading seen on boat compass, calculated from sensorHeading + boatCompassOffset
unsigned short sensorHeading = 0; //Heading read from CMPS14
byte calibration = 0; //CMPS14 calibration level

//Set up some storage for the NMEA output messages
HSCmessage hsc;
HDMmessage hdm;

const char *ssid = "NavSource";
WiFiServer *telnetServer = NULL;
WiFiServer *configServer = NULL;
WiFiClient **telnetClients = {NULL};
WiFiClient configClient;

extern int16_t compassCard[]; //declared in calibration.h has compass card offsets for each degree


void setup() {
 
  Serial.begin(115200);
  delay(1000);
  Wire.begin();
  
  settings.begin("compass",false); //Open (or create) settings namespace "compass" in read-write mode
  if ( settings.isKey("compassCard") ) {
    Serial.println("Loading settings from flash memory");
    settings.getBytes("compassCard",&compassCard,sizeof(compassCard));
  } else Serial.println("No settings found in flash");
  
  calibrationBegin();

  //Init OLED display
  display.begin(DISPLAY_I2C_ADDRESS, true); // Address 0x3C default
  //Display splash screen on OLED
  displayOLEDSplash();

  // -- We should now have a working sensor so set up the task schedule
  runner.init();
  runner.addTask(outputTask);
  runner.addTask(updateHeadingTask);
  runner.addTask(checkWiFiClientsTask);
  runner.addTask(updateOLEDTask);

  //Startup the Wifi access point
  WiFi.softAP(ssid);
  telnetClients = new WiFiClient*[4];
  for(int i = 0; i < 4; i++)
  {
    telnetClients[i] = NULL;
  }
  //This server outputs the NMEA messages
  telnetServer = new WiFiServer(TELNET_PORT);
  telnetServer->begin();

  //This server is used for config, calibration  & debug
  configServer = new WiFiServer(CONFIG_PORT);
  configServer->begin();

  IPAddress myAddr = WiFi.softAPIP();
  Serial.print("IP Address =");
  Serial.println(myAddr);

  delay(2000);

  //Real time tasks start from now
  //Start the background tasks
  outputTask.enable();
  updateHeadingTask.enable();
  checkWiFiClientsTask.enable();
  updateOLEDTask.enable();
   
}

unsigned loopCounter;
long loopStart, totalLoopTime=0;

void loop() {
  
  loopStart = micros();

  //Run the background tasks
  runner.execute();

  if (configClient) {
    Serial.println("Config Client detected.");
    outputTask.disable();
    updateHeadingTask.disable();
    calibrationMenu();
    outputTask.enable();
    updateHeadingTask.enable();   
  }
  
  totalLoopTime += micros() - loopStart;
  loopCounter++;

}

//Definition of background tasks

//Output the heading as an NMEA message over WiFi
void output() {
  char buff[128];
  
  sprintf(buff, "Current sensor heading: %03d deg.\n", sensorHeading);
  Serial.print(buff);
  sprintf(buff, "Current boat heading: %03d deg.\n", boatHeading);
  Serial.print(buff);  
  
  
  hdm.update(boatHeading);
    
  for ( int i=0; i<MAX_TELNET_CLIENTS; i++ ) {
    if ( telnetClients[i] != NULL ) 
      telnetClients[i]->println(hdm.msgString);
  }
}


//Update the heading from the CMPS14
void updateHeading() {         
    
    //get the raw CMPS14 output
    sensorHeading = getBearing();

    //Apply compass card offset
    boatHeading = MOD360(sensorHeading - compassCard[sensorHeading]);

    calibration = getCalibration();
}  


//Look for new WiFi clients
void checkWiFiClients() {
  
  WiFiClient tempClient = telnetServer->available();   // listen for incoming clients

  if (tempClient) {                             // if you get a client,
    Serial.println("New NMEA client.");           // print a message out the serial port
     for (int i=0; i<MAX_TELNET_CLIENTS; i++ ) {
      if ( telnetClients[i] == NULL ) {
        WiFiClient* client = new WiFiClient(tempClient);
        telnetClients[i] = client;
        break;
      }
    }
  }
  configClient = configServer->available();
  if (configClient.connected()) {
    Serial.println("New Config client");
  } 
}

void displayOLEDSplash()
{
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(20,0);
  display.setTextColor(SH110X_WHITE);
  display.println("E.A.S.T.");
  display.setCursor(25,25);
  display.setTextSize(1);
  display.println("Audio Compass");
  display.setCursor(25,40);
  display.println("Prototype 0.C");
  display.drawLine(0, 59, 127, 59, SH110X_WHITE);
  display.drawCircle(63, 59, 4, SH110X_WHITE);
  display.display();
}

void displayHeadings()
{
  char buff[64];
  long diff;
  
  buff[0] = '\0';
  
  //Set up run mode OLED display  - display fixed items
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(3,0);
  display.println("Sensor Hdg  Boat Hdg");
  display.drawLine(66, 0, 66, 34, SH110X_WHITE);
  display.drawLine(0, 34, 127, 34, SH110X_WHITE);
  display.setCursor(3,38);
  display.println("Calibration Status;");

  display.setTextSize(2);

  //Display sensor heading
  display.setCursor(12,12);
  sprintf(buff,"%03d", sensorHeading);
  display.print(buff);

  //Display boat heading
  display.setCursor(83,12);
  sprintf(buff,"%03d", boatHeading);
  display.print(buff);
  
  //Sensor calibration status
  display.setCursor(3,50); 
  display.setTextSize(1);
   sprintf(buff,"   S:%1d G:%1d A:%1d M:%1d",
      (calibration & 0b11000000) >> 6, (calibration & 0b00110000) >> 4, (calibration & 0b00001100) >> 2, calibration & 0b00000011);
  display.print(buff);
  
  display.display();
}
