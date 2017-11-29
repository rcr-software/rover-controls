/*

    /\\\\\\\\\           /\\\\\       /\\\        /\\\  /\\\\\\\\\\\\\\\    /\\\\\\\\\                /\\\\\\\\\\\\\       /\\\\\\\\\     /\\\        /\\\  /\\\                   /\\\\\          /\\\\\\\\\     /\\\\\\\\\\\\
   /\\\///////\\\       /\\\///\\\    \/\\\       \/\\\ \/\\\///////////   /\\\///////\\\             \/\\\/////////\\\   /\\\\\\\\\\\\\  \///\\\    /\\\/  \/\\\                 /\\\///\\\      /\\\\\\\\\\\\\  \/\\\////////\\\
   \/\\\     \/\\\     /\\\/  \///\\\  \//\\\      /\\\  \/\\\             \/\\\     \/\\\             \/\\\       \/\\\  /\\\/////////\\\   \///\\\/\\\/    \/\\\               /\\\/  \///\\\   /\\\/////////\\\ \/\\\      \//\\\
    \/\\\\\\\\\\\/     /\\\      \//\\\  \//\\\    /\\\   \/\\\\\\\\\\\     \/\\\\\\\\\\\/              \/\\\\\\\\\\\\\/  \/\\\       \/\\\     \///\\\/      \/\\\              /\\\      \//\\\ \/\\\       \/\\\ \/\\\       \/\\\
     \/\\\//////\\\    \/\\\       \/\\\   \//\\\  /\\\    \/\\\///////      \/\\\//////\\\              \/\\\/////////    \/\\\\\\\\\\\\\\\       \/\\\       \/\\\             \/\\\       \/\\\ \/\\\\\\\\\\\\\\\ \/\\\       \/\\\
      \/\\\    \//\\\   \//\\\      /\\\     \//\\\/\\\     \/\\\             \/\\\    \//\\\             \/\\\             \/\\\/////////\\\       \/\\\       \/\\\             \//\\\      /\\\  \/\\\/////////\\\ \/\\\       \/\\\
       \/\\\     \//\\\   \///\\\  /\\\        \//\\\\\      \/\\\             \/\\\     \//\\\            \/\\\             \/\\\       \/\\\       \/\\\       \/\\\              \///\\\  /\\\    \/\\\       \/\\\ \/\\\       /\\\
        \/\\\      \//\\\    \///\\\\\/          \//\\\       \/\\\\\\\\\\\\\\\ \/\\\      \//\\\           \/\\\             \/\\\       \/\\\       \/\\\       \/\\\\\\\\\\\\\\\    \///\\\\\/     \/\\\       \/\\\ \/\\\\\\\\\\\\/
         \///        \///       \/////             \///        \///////////////  \///        \///            \///              \///        \///        \///        \///////////////       \/////       \///        \///  \////////////


           Authors:   Kristian Jens Meyer
  Date of Creation:   November 6th, 2017
  Date of Revision:   Novermber 6th, 2017
    Version Number:   1.0
      Organization:   River City Rocketry

       Description:   This software is designed to control the AUTONOMOUS ROVER with FOLDABLE SOLAR ARRAY to act as the payload of the River City Rocketry ___________ Launch Vehicle.
*/

/****************************************Include Libraries*******************************************************************/
/* Adafruit gits:
    https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library.git
    https://github.com/adafruit/Adafruit_Sensor.git
    https://github.com/adafruit/Adafruit_BluefruitLE_nRF51.git

   Arducam git:
    https://github.com/ArduCAM/Arduino.git
    (might have to copy "ArduCAM" directory out "Arduino" so it can be found)
*/
#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "BluefruitConfig.h"
#include "utility/Adafruit_MS_PWMServoDriver.h"
//#include "RTClib.h"
#include <RTClib.h>

#include "RoverCAM.h" //local file


#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

/****************************************Initialize Defines*******************************************************************/
#define FACTORYRESET_ENABLE         1                                                                                                             //Factory reset recommended prior to loading sketch for the first time
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"                                                 //If problems persist, check warning in nRF51 Controller Example
#define MODE_LED_BEHAVIOUR          "MODE"
#define MOTOR_SPEED   100                                                                   //Power delivered to motors, 0-255
#define BATTPIN A7

/****************************************Bluefruit Object*******************************************************************/
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);       //Create a Bluefruit Object for Hardware SPI
Adafruit_MotorShield AFMS = Adafruit_MotorShield();                                         //Create Motor Shield object
Adafruit_DCMotor *motor1 = AFMS.getMotor(1);                                                //Select which H-Bridge you want to drive
Adafruit_DCMotor *motor2 = AFMS.getMotor(2);
Adafruit_DCMotor *motor3 = AFMS.getMotor(3);
Adafruit_DCMotor *motor4 = AFMS.getMotor(4);

/****************************************Initialize Globals*******************************************************************/
uint8_t readPacket(Adafruit_BLE*ble, uint16_t timeout);                                     //Functional prototypes in packetparser.cpp
float parsefloat(uint8_t*buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);
extern uint8_t packetbuffer[];
boolean ble_connected;

/****************************************Adalogger Initializations*******************************************************************/
File controllerData;
String controllerDataLogName = "TEMP";                                                      //This variable is used to create a new unique data file later
RTC_PCF8523 rtc;                                                                                                                                  //Real time clock object
int timeSince = 0;
const int adaloggerCS = 10;

/*=================================================================================================================================================================================================================================================================

                                                                                                            Error Function
            -Used to pass any errors along the way
  =================================================================================================================================================================================================================================================================*/

void error(const __FlashStringHelper*err)                                                                                                          //Error print
{
  Serial.println(err);
  while (1);
}

/*=================================================================================================================================================================================================================================================================

                                                                                                                Setup
            -Sets up the HW and the BLE module
  =================================================================================================================================================================================================================================================================*/
void setup()
{
  /****************************************Initialize Serial*******************************************************************/
  delay(500);                                                                                                                                       //There was a line here while(!Serial) mentioned to be only used for Flora and Micro
  Serial.begin(115200);
  Serial.println(F("River City Rocketry"));
  Serial.println(F("-----------------------------------------"));

  /****************************************Check Initialization of BLE Module*******************************************************************/
  Serial.print(F("Initialising Bluefruit LE Module: "));

  if (!ble.begin(VERBOSE_MODE))                                                                                                                     //Check initialization of BLE module
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));                                                            //Pass error if BLE not initialized

  Serial.println(F("Successful"));                                                                                                                  //If initialized, move on

  /****************************************Factory Reset*******************************************************************/
  if (FACTORYRESET_ENABLE)                                                                                                                          //Perform a factory reset based on value set above to return board to a known state
  {
    Serial.println(F("Performing Factory Reset: "));

    if (!ble.factoryReset())                                                                                                                        //Check if factory reset was successful
      error(F("Couldn't factory reset"));                                                                                                           //If unsuccessful, pass error

    Serial.println(F("Reset: Successful"));
    Serial.println();
  }

  /****************************************Disable Echo*******************************************************************/
  ble.echo(false);

  /****************************************Request BLE Module Info*******************************************************************/
  Serial.println("Requesting Bluefruit info:");
  ble.info();                                                                                                                                        //Display BLE Module information
  Serial.println();

  /****************************************Menu Instructions*******************************************************************/
  Serial.println(F("Attempt to Pair with Feather"));
  Serial.println();

  /****************************************Debug Settings*******************************************************************/
  ble.verbose(false);                                                                                                                                //Turn debug info off

  /****************************************BLE Connect*******************************************************************/
  Serial.print(F("Pairing: "));

  //loop until ble connects or retry limit of 6 is reached
  int retries = 0;
  while (true)                //Attempt to connect to BLE device
  {
    //try to get ble
    if (ble.isConnected()) {
      ble_connected = true;
      Serial.println(F("Successful ble connection"));
      break;
    }
    //limit number of retries
    if (retries++ > 6) {
      ble_connected = false;
      Serial.println(F("Failed ble connection"));
      break;
    }

    Serial.println("failed, retry in 0.5 seconds");
    delay(500);
  }
  if (ble_connected) {
    /****************************************Check Firmware*******************************************************************/
    if (ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )                                                                                               //LED command supported from 0.6.6
    {
      Serial.println(F("Changing LED to Mode: " MODE_LED_BEHAVIOUR));                                                                                  //Change mode LED activity
      ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);                                                                                      //Send test command
    }
  }
  /****************************************File Creation*******************************************************************/
  if (!rtc.begin())                                                                                                                                  //Check intialization of the real time clock
    error(F("Couldn't find RTC"));

  if (!SD.begin(adaloggerCS))                                                                                                                        //Check initialization of the SD card
    error(F("Couldn't find SD card"));
  else
    Serial.println("SD card initializing: Success");                                                                                                 //Confirm the SD card is initialized properly

  DateTime now = rtc.now();                                                                                                                          //Query the RTC
  timeSince = now.unixtime() - 1510471301;                                                                                                           //Get time since 1/1/1970, convert to time since 11/10/2017
  controllerDataLogName = String(timeSince);                                                                                                                      //Create a string of the time since 11/10/2017, this guarantees unique file names and no overwriting

  File controllerData = SD.open(controllerDataLogName + ".txt", FILE_WRITE);                                                                                      //Can only be 7 letters or less

  if (controllerData)                                                                                                                                //Check if the controllerData log file has been created properly
  {
    Serial.print("File created: ");
    Serial.println(controllerDataLogName + ".txt");
    controllerData.println("Time\t\tButton\tPressed\tAction");                                                                                       //Print the header line in the file
    controllerData.close();                                                                                                                          //Close the file
  }
  else
    error(F("Controller Data log file NOT created"));

  /****************************************Enter Data Mode*******************************************************************/
  if (ble_connected)
  {
    Serial.print( F("Switching to DATA Mode: ") );
    ble.setMode(BLUEFRUIT_MODE_DATA);                                                                                                                  //Put Bluefruit in Data Mode
    Serial.println(F("Successful"));
    Serial.println();
    Serial.println("Controller Active");
  }
  /****************************************Initialize Motors*******************************************************************/
  AFMS.begin();

  motor1->setSpeed(MOTOR_SPEED);                                                                                                                     //Set default motor speed
  motor1->run(FORWARD);
  motor1->run(RELEASE);                                                                                                                              //Release allows the motor to coast to a stop

  motor2->setSpeed(MOTOR_SPEED);
  motor2->run(FORWARD);
  motor2->run(RELEASE);

  motor3->setSpeed(MOTOR_SPEED);
  motor3->run(FORWARD);
  motor3->run(RELEASE);

  motor4->setSpeed(MOTOR_SPEED);
  motor4->run(FORWARD);
  motor4->run(RELEASE);
  /****************************************Setup RoverCAM*******************************************************************/
  setup_rovercam(); //perform setup in RoverCAM.h
}

/*=================================================================================================================================================================================================================================================================

                                                                                                                Loop
            -Main loop function where the primary controls will happen
  =================================================================================================================================================================================================================================================================*/

void loop()
{
  int serial_command = Serial.parseInt();

  if (serial_command != 0)
  {
    Serial.print("Serial received: ");
    Serial.println(serial_command);
    rover_command(serial_command, true);
  }

  /****************************************Check for Input Data*******************************************************************/
  if (ble_connected)
  {
    uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);    //Read incoming data
    if (len == 0)
      return;

    /****************************************Controlling the Motors*******************************************************************/
    if (packetbuffer[1] == 'B' || serial_command != 0)
    {
      uint8_t buttNum = packetbuffer[2] - '0';     //Incoming button data is in the following package: [!] [B] [buttNum] [pressed] [CRC]
      boolean pressed = packetbuffer[3] - '0';
      rover_command(buttNum, pressed);


    }
  }
}

//Call rover command with a function for re-usability
void rover_command(int buttNum, boolean pressed) {
  String action = "";
  switch (buttNum)
  {
    case 1:
      if (pressed)
      {
        motor1->run(FORWARD);
        Serial.println("Motor 1 Engaged...");
        action = "Motor 1 Engaged";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor1->run(RELEASE);
        Serial.println("Motor 1 Disengaged...");
        action = "Motor 1 Disengaged";
        logData(buttNum, pressed, action);
      }
      break;
    case 2:
      if (pressed)
      {
        motor2->run(FORWARD);
        Serial.println("Motor 2 Engaged...");
        action = "Motor 2 Engaged";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor2->run(RELEASE);
        Serial.println("Motor 2 Disengaged...");
        action = "Motor 2 Disengaged";
        logData(buttNum, pressed, action);
      }
      break;
    case 3://image capture
      if (pressed)
      {
        Serial.println("Taking picture");
        DateTime now = rtc.now();
        int k = now.unixtime() - 1510471301;
        bool success = capture(k); //from ArduCAM
        if (success) {
          Serial.print("Took picture #");
          Serial.println(k);
          action = "Took picture #" + String(k, DEC);
        }
        else {
          Serial.print("failed");
          action = "Failed to take picture";
        }
        logData(buttNum, pressed, action);
      }
      else
      {
        Serial.println("Camera button released");
        action = "Camera button released";
        logData(buttNum, pressed, action);
      }
      break;
    case 4:
      if (pressed)
      {
        motor4->run(FORWARD);
        Serial.println("Motor 4 Engaged...");
        action = "Motor 4 Engaged";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor4->run(RELEASE);
        Serial.println("Motor 4 Disengaged...");
        action = "Motor 4 Disengaged";
        logData(buttNum, pressed, action);
      }
      break;
    case 5:
      if (pressed)
      {
        motor3->run(FORWARD);
        motor4->run(FORWARD);
        Serial.println("Drive Forward...");
        action = "Drive Forward";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor3->run(RELEASE);
        motor4->run(RELEASE);
        Serial.println("Disengage...");
        action = "Disengage";
        logData(buttNum, pressed, action);
      }
      break;
    case 6:
      if (pressed)
      {
        motor3->run(BACKWARD);
        motor4->run(BACKWARD);
        Serial.println("Drive Backward...");
        action = "Drive Backward";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor3->run(RELEASE);
        motor4->run(RELEASE);
        Serial.println("Disengage...");
        action = "Disengage";
        logData(buttNum, pressed, action);
      }
      break;
    case 7:
      if (pressed)
      {
        motor3->run(BACKWARD);
        motor4->run(FORWARD);
        Serial.println("Turn Left...");
        action = "Turn Left";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor3->run(RELEASE);
        motor4->run(RELEASE);
        Serial.println("Disengage...");
        action = "Disengage";
        logData(buttNum, pressed, action);
      }
      break;
    case 8:
      if (pressed)
      {
        motor3->run(FORWARD);
        motor4->run(BACKWARD);
        Serial.println("Turn Right...");
        action = "Turn Right";
        logData(buttNum, pressed, action);
      }
      else
      {
        motor3->run(RELEASE);
        motor4->run(RELEASE);
        Serial.println("Disengage...");
        action = "Disengage";
        logData(buttNum, pressed, action);
      }
      break;
    default:
      Serial.println("DEFAULT...");
      motor1->run(RELEASE);
      motor2->run(RELEASE);
      motor3->run(RELEASE);
      motor4->run(RELEASE);
      break;
  }
  float measuredvbat = analogRead(BATTPIN);
  measuredvbat *= 2;                                                                                                                              //Divided by 2, so multiply back
  measuredvbat *= 3.7;                                                                                                                            //Multiply by 3.3V, the reference voltage
  measuredvbat /= 1024;                                                                                                                           //Convert to voltage
  Serial.print("VBat: " );
  Serial.println(measuredvbat);
  Serial.println();
}

/*=================================================================================================================================================================================================================================================================

                                                                                                                Log Data
            -Log data passed to this function to the SD card
  =================================================================================================================================================================================================================================================================*/

void logData(int buttonNumber, int pressedState, String actionTaken)
{
  int tsHour = 0;
  int tsMinute = 0;
  int tsSecond = 0;
  String timeStamp = "";
  String data = "";

  DateTime now = rtc.now();
  tsHour = now.hour();
  tsMinute = now.minute();
  tsSecond = now.second();
  timeStamp = String(tsHour) + ":" + String(tsMinute) + ":" + String(tsSecond);

  data = String(tsHour) + ":" + String(tsMinute) + ":" + String(tsSecond) + "\t\t" + String(buttonNumber) + "\t" + String(pressedState) + "\t" + actionTaken;

  File controllerData = SD.open(controllerDataLogName + ".txt", FILE_WRITE);
  if (controllerData)
  {
    controllerData.println(data);
    controllerData.close();
  }
}




