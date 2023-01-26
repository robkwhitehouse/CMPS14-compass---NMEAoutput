#include "Cmps14.h"

#define MOD360(x) (((x)%360 + 360) % 360)

#define _i2cAddress         0x60
#define calibrationQuality  0x1E

// https://stackoverflow.com/questions/111928 (nice trick)
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"

#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

// Timer
unsigned long mytime;

// Character array
char Message[256];

// Compass card - array holds offsets to boat compass
// Allows CMPS14 to be mounted in any orientation

int16_t compassCard[360];

extern WiFiClient configClient;
extern Preferences settings;

//local function prototypes
byte getVersion();
void CalibrationQuality();
void writeToCMPS14(byte n);
void printMenu();
void Countdown(unsigned);
void printTerm(char *);
void printTerm(byte);
void endTransmission();
void createCompassCard();
void resetCompassCard();
void displayCompassCard();
void saveCompassCard();
byte getCalibration();

void calibrationBegin() {
  printTerm("----------------------\n");
  printTerm("   Calibrate CMPS14\n"); 
  printTerm("----------------------\n");

  printTerm("CMPS 14 software version v");
  printTerm(getVersion());
  printTerm("\n");
  CalibrationQuality();

}

void calibrationMenu() {
  byte a, junk;

  printMenu();
  do {
    if (!(Serial || configClient)) {
      Serial.println("Error: CalibrationMenu() - no client ");
      break; //No user terminal present
    }
    else { //We have a config user
      a = 0;
      if (Serial.available() > 0 || configClient.available() > 0) {//We have some user input 
        // Read User input
        if (Serial.available() > 0) {
          a = Serial.read();
          while(Serial.available() > 0) junk = Serial.read(); //ditch any trailing EOL chars
        } else {
          a = configClient.read();
          while(configClient.available() > 0) junk = configClient.read();
        }
      } 

      // Do we start calibration?
      if (a == 'm' || a == 'a' || a == 'g' || a == 'p'  || a == 'x'){

      // Configuation byte
        writeToCMPS14(byte(0x98));
        writeToCMPS14(byte(0x95));
        writeToCMPS14(byte(0x99));

        // Begin communication with CMPS14
        Wire.beginTransmission(_i2cAddress);

        // Want the Command Register
        Wire.write(byte(0x00));

        // Send some data
        switch (a){

          case 'm':
            printTerm("Magnetometer...\n");
            Wire.write(byte(B10000001));
            endTransmission();
            printTerm("Rotate the CMPS14 randomly around for 40 seconds\n");
            Countdown(40);
            CalibrationQuality();
            break;

          case 'a':
            printTerm("Accelerometer...\n");
            Wire.write(byte(B10000010));
            endTransmission();
            printTerm("Rotate in differnt 90 degrees and keep steady for a while\n");
            Countdown(40);
            CalibrationQuality();
            break;

          case 'g':
            printTerm("Gyro... Keep the CMPS14 stationary\n");
            Wire.write(byte(B10000100));
            endTransmission();
            Countdown(20);
            CalibrationQuality();
            break;

          case 'p':
            printTerm("Enable periodic automatic save of calibration data\n");
            Wire.write(byte(B10010000));
            endTransmission();
            CalibrationQuality();
            break;

          case 'x':
            printTerm("Stop auto calibration\n");
            Wire.write(byte(B10000000));
            endTransmission();
            CalibrationQuality();
            break;
        }
        printMenu();
      }

      // Show calibration quality
      if (a == 'c'){
        CalibrationQuality();
      }

      // Store the calibration
      if (a == 's'){

        writeToCMPS14(byte(0xF0));
        writeToCMPS14(byte(0xF5));
        writeToCMPS14(byte(0xF6));      

        // Update the User
        printTerm("Calibration profile saved\n");
      }

      // Reset the calibration
      if (a == 'e'){
      
        writeToCMPS14(byte(0xE0));
        writeToCMPS14(byte(0xE5));
        writeToCMPS14(byte(0xE2));      

        // Update the User
        printTerm("Saved calibration erased, factory defaults apply\n");
        delay(500);
      }

      //End of I2C activities. release the bus
      endTransmission();

      //Other (non-I2C commands)

      switch(a) {
        //Print the command menu
        case'h':
        case '?':
         printMenu(); break;
        //Align with boat compass (create a compass card)
        case 'b': createCompassCard(); break;
        case 'd': displayCompassCard(); break;
        //Zero (reset) the compassCard
        case 'z': resetCompassCard(); break;
        //Save the current compassCard to NV memory
        case'n': saveCompassCard(); break;
        //Restart the ESP32
        case 'r': ESP.restart(); 
        //Will never get here but just in case ...
                break;
      }
    }
  } while( a != 'q'); 
}

void writeToCMPS14(byte n){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
    
  // Want the Command Register
  Wire.write(byte(0x00));

  // Send some data    
  Wire.write(n);
  
  endTransmission();
}

void CalibrationQuality(){

  byte calibration = getCalibration();
  sprintf(Message,"Calibration " BYTE_TO_BINARY_PATTERN "\n", BYTE_TO_BINARY(calibration));
  printTerm(Message);
}

byte getCalibration() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
  delay(20);
  // Tell register you want some data
  Wire.write(calibrationQuality);
  endTransmission();

  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(_i2cAddress, 1);

  // Timed out so return
  if (nReceived != 1) {
    sprintf(Message,"calibrationQuality - invalid return from Wire.requestFrom() == %d\n", nReceived);
    printTerm(Message);
    return(0);
  }
  
  // Read the values
  return(Wire.read());

}


void endTransmission() {
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0)
    printTerm("communication error\n");
  // Wait 100ms
  delay(100);
}

byte getVersion(){

  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
    
  // Want the Command Register
  Wire.write(byte(0x00));

  // Send some data    
  Wire.write(byte(0x11));
  
  endTransmission();

  // Request 1 byte from CMPS14
  int nReceived = Wire.requestFrom(_i2cAddress, 1);

  // Read the values
  byte ver = Wire.read();

  return ver;
}

void Countdown(unsigned period){

  int i;
  for (i=period; i>0; i--){

    printTerm(i);
    printTerm(" ");
    delay(1000);
  }

  printTerm("OK\n");

}

void printTerm(char *mesg) {
  if (Serial) Serial.print(mesg);
  if (configClient) configClient.print(mesg);
}

void printTerm(byte mesg) {
  if (Serial) Serial.print(mesg);
  if (configClient) configClient.print(mesg);
}

void printMenu() {
    
  printTerm("\n");
  printTerm("Enter command;\n");
  printTerm(" - 'h' to print this menu\n");
  printTerm(" - 'c' to see current calibration levels\n");
  printTerm(" - 'g' to calibrate the gyroscope\n");
  printTerm(" - 'a' to calibrate accelerometer\n");
  printTerm(" - 'm' to calibrate magnetometer\n");
  printTerm(" - 's' to save current CMPS calibration\n");
  printTerm(" - 'e' to erase the saved CMPS calibration\n");
  printTerm(" - 'p' to enable periodic auto-save\n");
  printTerm(" - 'x' to disable periodic auto-save\n");
  printTerm(" - 'b' to generate a boat compass card\n");
  printTerm(" - 'd' to display the compass card\n");
  printTerm(" - 'z' to zero (erase) the compass card\n");
  printTerm(" - 'n' to save compass card to ESP32 Non-volatile memory\n");
  printTerm(" - 'r' to reboot the system\n");
  printTerm(" - 'q' to quit settings mode\n");
  printTerm("->? ");

}

/*
 * generate a "compass card" for the CMPS14
 * Allows the CMPS to be mounted in any orientation
 * Generates a mapping table, mapping readings from the CMPS into 
 * real world magnetic compass bearings
 */

//procedure to create the compass card
void createCompassCard() {

  int junk, north, east, south, west;
  float delta;
  char buff[128];  
  unsigned index;

  //Clear input buffers

  //Get sensor heading for north
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due North. Hit enter when the boat compass reads 000 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  north = getBearing();
  sprintf(buff,"CMPS reading for North is %03d degrees\n\n",north);
  printTerm(buff);

  //Get sensor heading for east
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due East. Hit enter when the boat compass reads 090 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  east = getBearing();
  sprintf(buff,"CMPS reading for East is %03d degrees\n\n", east);
  printTerm(buff);

  //Get sensor heading for south
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due South. Hit enter when the boat compass reads 180 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  south = getBearing();
  sprintf(buff,"CMPS reading for South is %03d degrees\n\n",south);
  printTerm(buff);

  //Get sensor heading for west
  while (Serial.available()) junk = Serial.read();
  while (configClient.available()) junk = configClient.read();
  printTerm("Steer the boat due West. Hit enter when the boat compass reads 270 degrees.\n");
  while (Serial.available() == 0 && configClient.available() == 0)  ; //wait 
  west = getBearing();
  sprintf(buff,"CMPS reading for West is %03d degrees\n\n", west);
  printTerm(buff);
  
//now calculate the mapping for every possible degree

//first quandrant
  //Calculate the average difference per degree. This will likely vary for each quadrant
  unsigned Qsize = MOD360(east-north); //Might not be 90 due to sensor eccentricities etc.
  sprintf(buff,"Q1 size is %d\n",Qsize);
  printTerm(buff);
  delta = (Qsize / 90.0)-1; //force float arithmetic
  sprintf(buff,"Q1 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(north+i);
    compassCard[index] = MOD360((int)(north + (int)round(i*delta)));
  }

//second quandrant
  //Calculate the average difference per degree. This will likely vary for each quadrant
  Qsize = MOD360(south-east); //Might not be 90 
  delta = (Qsize / 90.0)-1;
  sprintf(buff,"Q2 size is %d\n",Qsize);
  printTerm(buff);
  sprintf(buff,"Q2 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(east+i);
    compassCard[index] = MOD360((int)(east-90 + (int)round(i*delta)));
  }

  //third quadrant
  //Calculate the average difference per degree. This will vary for each quadrant
  Qsize = MOD360(west-south); //Might not be 90 
  delta = (Qsize / 90.0)-1; //force float arithmetic
  sprintf(buff,"Q3 size is %d\n",Qsize);
  printTerm(buff);
  sprintf(buff,"Q3 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(south+i);
    compassCard[index] = MOD360((int)(south-180 + (int)round(i*delta)));
  }

  //Calculate the average difference per degree. This will vary for each quadrant
  Qsize = MOD360(north-west); //Might not be 90 
  sprintf(buff,"Q4 size is %d\n",Qsize);
  printTerm(buff);
  delta = (Qsize / 90.0)-1; //force float arithmetic
  sprintf(buff,"Q4 delta is %f\n",delta);
  printTerm(buff);
  //Now populate the compassCard array
  for (int i=0;i<Qsize;i++) {
    int index = MOD360(west+i);
    compassCard[index] = MOD360((int)(west-270 + (int)round(i*delta)));
  }
}

//Procedure to zero the compass card
void resetCompassCard() {
  for(int i=0; i<360; i++) compassCard[i] = 0;
}

//Procedure to save the compass card to ESP32 NVRAM - this will be automatically restored
//on power up
void saveCompassCard() {
  settings.putBytes("compassCard", compassCard, sizeof(compassCard));
  printTerm("compassCard saved\n");
}

//Display the compass  card
void displayCompassCard() {
  char buff[128];
  printTerm("compassCard;\n");
  for (int i = 0; i<360; i++) {
    sprintf(buff,"compassCard[%d] = %d\n",i,compassCard[i]);
    printTerm(buff);
  }  
}