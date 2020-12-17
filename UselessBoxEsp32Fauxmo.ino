/******************************************************************************
 
  Useless Box with Alexa
  Tony Weil tonylweil@gmail.com
  12/17/2020

  DOIT ESP32 DEVKIT V1 https://www.amazon.com/gp/product/B086MJGFVV/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1
  SparkFun SparkFun Qwiic Motor Driver  https://www.sparkfun.com/products/15451
  DFRobot DFPlayerMini https://wiki.dfrobot.com/DFPlayer_Mini_SKU_DFR0299
  Adafruit Mini Lipo w/Mini-B USB Jack - USB LiIon/LiPoly charger - v1 https://www.adafruit.com/product/1905
  Adafruit Lithium Ion Polymer Battery - 3.7v 1200mAh https://www.adafruit.com/product/258


  
******************************************************************************/

#include <SCMD.h>        //https://github.com/sparkfun/SparkFun_Serial_Controlled_Motor_Driver_Arduino_Library
#include <SCMD_config.h> //Contains #defines for common SCMD register names and values
#include <Wire.h>

#include <WiFi.h>
#include <fauxmoESP.h>            //https://github.com/vintlabs/fauxmoESP
#include "credentials.h"          // Enter your Wifi credentials
#include "DFRobotDFPlayerMini.h"  //https://github.com/DFRobot/DFRobotDFPlayerMini

SCMD myMotorDriver; //This creates the main object of one motor driver and connected slaves.
#define motor 0
#define forward 0  //top goes clockwise (Dagu facing you)
#define backward 1 //top goes counterclockwise

DFRobotDFPlayerMini myDFPlayer;

HardwareSerial SerialPort2(2);  // ESP32 second serial port for DFPlayer communcation

fauxmoESP fauxmo;
#define RED_LED 4  // indicator of startup and when triggered by Alexa
bool triggerFlag = false;

void wifiSetup() {

    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

    // Connect
    Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Wait
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Connected!
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

}

void fauxmoSetup() {

    // Set fauxmoESP to not create an internal TCP server and redirect requests to the server on the defined port
    // The TCP port must be 80 for gen3 devices (default is 1901)
    // This has to be done before the call to enable()
   fauxmo.createServer(true);
   fauxmo.setPort(80); // This is required for gen3 devices

    // You have to call enable(true) once you have a WiFi connection
    // You can enable or disable the library at any moment
    // Disabling it will prevent the devices from being discovered and switched
   fauxmo.enable(true);

    // "Alexa, turn Useless Box on" 
    // Add virtual devices
  fauxmo.addDevice("Useless Box");

  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
        
        // Callback when a command from Alexa is received. 
        // You can use device_id or device_name to choose the element to perform an action onto (relay, LED,...)
        // State is a boolean (ON/OFF) and value a number from 0 to 255 (if you say "set kitchen light to 50%" you will receive a 128 here).
        // Just remember not to delay too much here, this is a callback, exit as soon as possible.
        // If you have to do something more involved here set a flag and process it in your main loop.

        if (strcmp(device_name, "Useless Box")==0) 
            Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
            // digitalWrite(RED_LED, state ? HIGH : LOW);
            if(state) triggerFlag = true;
    }); 
}

void motorSetup() {

  //***** Configure the Motor Driver's Settings *****//
  //  .commInter face can be I2C_MODE or SPI_MODE
  myMotorDriver.settings.commInterface = I2C_MODE;
  //myMotorDriver.settings.commInterface = SPI_MODE;

  //  set address if I2C configuration selected with the config jumpers
  //myMotorDriver.settings.I2CAddress = 0x5D; Default might be 0x5A, check
  myMotorDriver.settings.I2CAddress = 0x5D;

  //  set chip select if SPI selected with the config jumpers
  myMotorDriver.settings.chipSelectPin = 10;  // not needed
 
  //*****initialize the driver get wait for idle*****//
  while ( myMotorDriver.begin() != 0xA9 ) //Wait until a valid ID word is returned
  {
    Serial.println( "ID mismatch, trying again" );
    delay(500);
  }
  Serial.println( "ID matches 0xA9" );

  //  Check to make sure the driver is done looking for slaves before beginning
  Serial.print("Waiting for enumeration...");
  while ( myMotorDriver.ready() == false );
  Serial.println("Done.");
  Serial.println();

  //*****Set application settings and enable driver*****//

  while ( myMotorDriver.busy() );
  myMotorDriver.enable(); //Enables the output driver hardware

  //pass setDrive() a motor number, direction as 0(call 0 forward) or 1, and level from 0 to 255
  myMotorDriver.setDrive( motor, 0, 0); //Stop motor
  
}

void runMotor() {

  Serial.println("Motor Moving");
  myMotorDriver.setDrive( motor, forward, 100);
  delay(1000);
  myMotorDriver.setDrive( motor, 0, 0);
  delay(1000);
  myMotorDriver.setDrive( motor, forward, 100);
  delay(250);
  myMotorDriver.setDrive( motor, 0, 0);
  delay(1000);
  myMotorDriver.setDrive( motor, forward, 255);
  delay(500);
  myMotorDriver.setDrive( motor, backward, 255);
  delay(2000);
  myMotorDriver.setDrive( motor, 0, 0);
  
  myDFPlayer.play(1);  //Play the first mp3 on SD card
}

void dfPlayerSetup() {
  SerialPort2.begin(9600, SERIAL_8N1,16, 17); //Rx, Tx
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));
  myDFPlayer.begin(SerialPort2);
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30
   
}

void setup()
{

  Serial.begin(115200);
  Serial.println("Starting sketch.");

  wifiSetup();
  fauxmoSetup();
  motorSetup();
  dfPlayerSetup();
  
  Serial.println("Ready ...");
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(1000);
  digitalWrite(RED_LED, LOW);
}


void loop()
{
  fauxmo.handle(); // fauxmoESP uses an async TCP server but a sync UDP server, Therefore, we have to manually poll for UDP packets

  if(triggerFlag==true) {
     triggerFlag = false;
     digitalWrite(RED_LED, HIGH);
     runMotor();
     fauxmo.setState("Useless Box", false, 0);  // Set Useless Box state back to off
     digitalWrite(RED_LED, LOW);
  }  
}
