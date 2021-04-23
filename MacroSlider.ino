// 192.168.0.111
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include "Adafruit_MCP23017.h"
#include <LiquidCrystal_I2C.h>

// Instance of MCP23017 library
Adafruit_MCP23017 mcp;

/* ----- DEFINICE PINU ESP8266 ----- */
#define D0 16 // FREE - DO NOT HOLD LOW DURING BOOT (HIGH DURING BOOT)
#define D1 5  // STEP PIN SCL - MCP
#define D2 4  // SDA - MCP
#define D3 0  // FREE - DO NOT HOLD LOW DURING BOOT - WITHOUT PWM A I2C
#define D4 2  // FREE - DO NOT HOLD LOW DURING BOOT (HIGH DURING BOOT)
#define D5 14 // DIR PIN
#define D6 12 // STEP PIN
#define D7 13 // MUP SWITCH
#define D8 15 // IR DIODE - DO NOT HOLD HIGH DURING BOOT

#define RX 3 // FREE (HIGH DURING BOOT)
#define TX 1 // FREE - DO NOT HOLD LOW DURING BOOT (HIGH DURING BOOT)

#define DIR_PIN D5
#define STEP_PIN D6
#define IR_DIODE D8
#define ANALOG_PIN A0
#define MUP_SWITCH D7
#define SCREEN_WIDTH 20

/*
#define BUTTON_BACKWARD //MCP 0
#define BUTTON_FORWARD //MCP 1
#define BUTTON_END //MCP 2
#define BUTTON_START //MCP 3
#define DIODE_START //MCP 6
#define DIODE_END //MCP 7
*/

int analogPinValue = 0; // value read from the A0
int lastAnalogPinValue = 1024;

/* ----- DEFAULT VALUES ----- */
int defaultStepVal = 40;  // 0.1mm
int stepVal = defaultStepVal;
int defaultDelayBetweenStep = 50;
int defaultDelayMotorFrequency = 1000;
int buttonHoldTime = 2000;
int quickStepVal = 8000;

int cameraStabilizeWait = 5000;
int cameraShutterTime = 10000;
bool mirrorLockUpActive = false;
/* ----- / DEFAULT VALUES ----- */

/* ----- MOVE BUTTONS ----- */
int buttonStateBackward = 0;
int buttonStateForward = 0;
int lastButtonStateForward = 0;  // previous state of the button
int lastButtonStateBackward = 0; // previous state of the button
/* ----- / MOVE BUTTONS ----- */

/* ----- PHOTO BUTTONS AND DIODE ----- */
int buttonEnd = 0;
int buttonStart = 0;
int lastButtonEnd = 0;   // previous state of the button
int lastButtonStart = 0; // previous state of the button
int diodeEnd = 0;
int diodeStart = 0;
unsigned long startPressed = 0; // the moment the button was pressed (millis)
unsigned long endPressed = 0;   // the moment the button was released (millis)
unsigned long holdTime = 0;     // how long the button was hold (millis)
unsigned long idleTime = 0;     // how long the button was idle (millis)
/* ----- / PHOTO BUTTONS AND DIODE ----- */

/* ----- STEPPER MOTOR ----- */
bool startPositionIsSet = false; // default no set -1
bool endPositionIsSet = false;   // default no set -1
int autoMoveNumOfSteps = 0;
int delayBetweenStep = defaultDelayBetweenStep;
int delayMotorFrequency = defaultDelayMotorFrequency;
bool isRunInProgress = false;
int numberOfAlreadyTakenPhotos = 0;
/* ----- / STEPPER MOTOR ----- */

/* ----- LED DIODE ----- */
unsigned long previousMillisLed = 0; // will store last time LED was updated
const int numberOfLedBlink = 4;
const int intervalLed = 1000; // interval at which to blink (milliseconds)
int ledState = LOW;
/* ----- / LED DIODE ----- */

/* ----- LCD SCREEN ----- */
unsigned long currentMillisLCD = 0; // will store last time LED was updated
unsigned long previousMillisLCD = 0;        // will store last time LED was updated
unsigned long intervalLCD = 250;        // will store last time LED was updated
/* ----- / LCD SCREEN ----- */

/* ----- OTHERS ----- */
unsigned long currentMillisMUP= 0; // will store last time LED was updated
unsigned long previousMillisMUP = 0;        // will store last time LED was updated
unsigned long intervalMUP = 250;        // will store last time LED was updated
/* ----- / OTHERS ----- */

// Create an instance of the server
// specify the port to listen on as an argument
WiFiServer server(80);
const char *ssid = "TP-LINK_E11D";
const char *password = "76163760";

// Variable to store the HTTP request
String header;
String currentLine = ""; // make a String to hold incoming data from the client

LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup()
{

  Wire.begin();
  lcd.init(); // initialize lcd
  lcd.backlight();

  // initialize the IR digital pin as an output:
  pinMode(IR_DIODE, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);
  pinMode(MUP_SWITCH, INPUT);

  /* -------- MCP -------- */
  // Initialize the MCP23017
  mcp.begin();
  // ----------------------
  mcp.pinMode(0, INPUT); // BACKWARD
  mcp.digitalWrite(0, LOW);
  mcp.pullUp(0, HIGH); // turn on a 100K pullup internally
  // ----------------------
  mcp.pinMode(1, INPUT); // FORWARD
  mcp.digitalWrite(1, LOW);
  mcp.pullUp(1, HIGH); // turn on a 100K pullup internally
  // ----------------------
  mcp.pinMode(2, INPUT); // END
  mcp.digitalWrite(2, LOW);
  mcp.pullUp(2, HIGH); // turn on a 100K pullup internally
  // ----------------------
  mcp.pinMode(3, INPUT); // START
  mcp.digitalWrite(3, LOW);
  mcp.pullUp(3, HIGH); // turn on a 100K pullup internally
  // ----------------------
  mcp.pinMode(6, OUTPUT); // DIODE START
  mcp.digitalWrite(6, diodeStart);
  // ----------------------
  mcp.pinMode(7, OUTPUT); // DIODE END
  mcp.digitalWrite(7, diodeEnd);
 // ----------------------

  digitalWrite(MUP_SWITCH, LOW);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(ANALOG_PIN, LOW);

  Serial.begin(9600);

  delay(10);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  printOnLCD(1,-1,"PRIPOJUJI K WIFI");
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    //Serial.println("Connection Failed! Rebooting...");
    lcd.clear();
    printOnLCD(1,-1,"WIFI PRIPOJENI");
    printOnLCD(2,-1,"NEUSPESNE");
    delay(2000);
    //delay(5000);
    //ESP.restart();
  }

  // Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname("Wemos D1 - Makro posuv");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  });
  ArduinoOTA.begin();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(50);
  }
  Serial.println("WiFi connected");
  lcd.clear();
  printOnLCD(1,-1,"WIFI PRIPOJENI");
  printOnLCD(2,-1,"USPESNE");

  // Start the server
  server.begin();
  Serial.println("Server started");

  // Print the IP address
  Serial.println(WiFi.localIP());
  lcd.clear();
}

void loop()
{
  ArduinoOTA.handle();
  //i2cScanner();

  // Is there change of number of steps by potentiometer?
  if (abs(analogPinValue - lastAnalogPinValue) > 10) // Compare analog value with last analog value with tolerance
  {
    stepVal = setStepLength();
  }

  getMirrorLockUpActiveBySwitch();

  if (isAutoRunInProgress() == false)
  {
    statusOnLCD();
    // BUTTON BACKWARD PRESS OR HOLD
    checkButtonBackward();

    // BUTTON FORWARD PRESS OR HOLD
    checkButtonForward();

    // BUTTON END PRESS
    checkButtonEnd();

    // BUTTON START PRESS
    checkButtonStart();
  }

  WiFiClient client = server.available(); // Listen for incoming clients

  if (client)
  { // If a new client connects,
    //Serial.println("New Client.");          // print a message out in the serial port

    while (client.connected())
    { // loop while the client's connected
      if (client.available())
      {                         // if there's bytes to read from the client,
        char c = client.read(); // read a byte, then
        //Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n')
        { // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0)
          {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            //client.println("\nHTTP/1.1 200 OK\nContent-type:text/html\nAccess-Control-Allow-Origin: *\nConnection: close\n");

            // CHECK POTENTIOMETER SETTINGS NUMBER OF STEPS
            if (header.indexOf("GET /?getSteps") >= 0)
            {
              client.print(lastAnalogPinValue);
              client.print("|");
              client.print(stepVal);
              Serial.println(lastAnalogPinValue);
              Serial.println(stepVal);
            }

             if (header.indexOf("GET /?getMUP") >= 0)
            {
              client.print(mirrorLockUpActive);
              client.print("|");
              client.print(digitalRead(MUP_SWITCH));
            }

            // GET NUMBER OF STEPS TO AUTO TAKE PHOTO
            if (header.indexOf("GET /?getAutoSteps") >= 0)
            {
              client.print(autoMoveNumOfSteps);
              client.print(startPositionIsSet);
              client.print(endPositionIsSet);
            }

            // TAKE PHOTO BY SEND NIKON IR CODE
            if (header.indexOf("GET /?takePhoto") >= 0)
            {
              // Send the response to the client
              takePhoto();
              Serial.println("Taking photo");
            }

            // GO FORWARD
            if (header.indexOf("GET /?stepForward") >= 0)
            {
              // Send the response to the client
              if (header.indexOf("&n=") > 0)
              {
                String number = header.substring(20, 25);
                int n = number.toInt();
                client.print(n);
                if (n < 0)
                {
                  n = 0;
                }
                goForward(n);
              }
              Serial.println("Go forward!");
            }

            // GO BACKWARD
            if (header.indexOf("GET /?stepBackward") >= 0)
            {
              // Send the response to the client
              if (header.indexOf("&n=") > 0)
              {
                String number = header.substring(21, 26);
                int n = number.toInt();
                client.print(n);
                if (n < 0)
                {
                  n = 0;
                }
                goBackward(n);
              }
              Serial.println("Go backward!");
            }

            // SET START POSITION
            if (header.indexOf("GET /?toggleStartPosition") >= 0)
            {
              // Send the response to the client
              toggleStartPosition();
              Serial.println("Toggle start position");
            }

            // SET END POSITION AND RUN AUTO CYCLE
            if (header.indexOf("GET /?toggleEndPosition") >= 0)
            {
              // Send the response to the client
              toggleEndPosition();
              Serial.println("Toggle end position and run auto cycle");
            }

            // The HTTP response ends with another blank line
            //client.println();
            // Break out of the while loop
            break;
          }
          else
          { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        }
        else if (c != '\r')
        {                   // if you got anything else but a carriage return character,
          currentLine += c; // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    currentLine = "";
    // Close the connection
    client.stop();
    delay(1);
  }
  lastButtonStateForward = buttonStateForward;   // save state for next loop
  lastButtonStateBackward = buttonStateBackward; // save state for next loop
  lastButtonStart = buttonStart;                 // save state for next loop
  lastButtonEnd = buttonEnd;                     // save state for next loop
  delay(5);
  lastAnalogPinValue = analogRead(ANALOG_PIN);
  delay(5);
}

/* SET LENGTH OF STEP BY VALUE FROM POTENTIOMETER */
unsigned short int setStepLength()
{
  analogPinValue = analogRead(ANALOG_PIN);
  lastAnalogPinValue = analogPinValue;

  switch (analogPinValue)
  {
case 0 ... 85:
    return 16000; // 20mm
    break;
 case 86 ... 171:
    return 8000; // 10mm
    break;
  case 172 ... 257:
    return 3200; // 4mm
    break;
  case 258 ... 343:
    return 2000; // 2,5mm
    break;
  case 344 ... 429:
    return 1600; // 2mm
    break;
  case 430 ... 515:
    return 1200; // 1.5mm
    break;
  case 516 ... 601:
    return 800; // 1mm
    break;
  case 602 ... 687:
    return 640; // 0.8mm
    break;
  case 688 ... 773:
    return 320; // 0.4mm
    break;
  case 774 ... 859:
    return 160; // 0.20mm
    break;
  case 860 ... 945:
    return 80; // 0.1mm
    break;
  case 946 ... 1024:
    return defaultStepVal; // 0.05mm
    break;
  default:
    return defaultStepVal; // 0.05mm
    break;
  }
}

float getRealStepLength()
{
  return (float)stepVal / 800;
}


/* NIKON IR TAKE PHOTO */
// This procedure sends a 38KHz pulse to the IRledPin
// for a certain # of microseconds. We'll use this whenever we need to send codes
void pulseIR(long microsecs)
{
  // we'll count down from the number of microseconds we are told to wait

  cli(); // this turns off any background interrupts

  while (microsecs > 0)
  {
    // 38 kHz is about 13 microseconds high and 13 microseconds low
    digitalWrite(IR_DIODE, HIGH); // this takes about 3 microseconds to happen
    delayMicroseconds(10);        // hang out for 10 microseconds, you can also change this to 9 if its not working
    digitalWrite(IR_DIODE, LOW);  // this also takes about 3 microseconds
    delayMicroseconds(10);        // hang out for 10 microseconds, you can also change this to 9 if its not working

    // so 26 microseconds altogether
    microsecs -= 26;
  }

  sei(); // this turns them back on
}

void takePhoto()
{
  sendNikonCode();
}

void stabilizeDelayWithCountdown(bool mirrorDown){

  for (int count = cameraStabilizeWait/1000; count > 0; count--)
  {
    // clearRowOnLCD(3);
    int charCount = 13 + 1 + getNumberOfDigitsInInt(count) + 1;
    int textPosition = (SCREEN_WIDTH - charCount)/2;
    printOnLCD(3,textPosition,"STABILIZACE:         ");
    printOnLCD(3,textPosition+13,count);
    printOnLCD(3,textPosition+13+getNumberOfDigitsInInt(count),"s");
    delay(1000);
  }
   clearRowOnLCD(3);
   if(mirrorDown == true){
     printOnLCD(3,-1,"PREDSKLAPIM ZRCATKO");
     delay(1000);
     clearRowOnLCD(3);
   }else{
     printOnLCD(3,-1,"FOTOGRAFUJI");
     delay(1000);
   }
   delay(500);
}

void takePhotoDelayWithCountdown(){
  for (int count = cameraShutterTime/1000; count > 0; count--)
  {
    // clearRowOnLCD(3);
    int charCount = 12 + 1 + getNumberOfDigitsInInt(count) + 1;
    int textPosition = (SCREEN_WIDTH - charCount)/2;
    printOnLCD(3,textPosition,"ZPRACOVANI:          ");
    printOnLCD(3,textPosition+12,count);
    printOnLCD(3,textPosition + 12 + getNumberOfDigitsInInt(count),"s");
    delay(1000);
  }
   clearRowOnLCD(3);
   printOnLCD(3,-1,"ZPRACOVANO");
}

void takePhotoAuto(bool mirrorLockUpActive)
{
  if (mirrorLockUpActive == true)
  {
    // delay(cameraStabilizeWait); // wait for camera stabilize
    stabilizeDelayWithCountdown(true);
    sendNikonCode();            // LockUpMirror - first IR signal
  }
  // delay(cameraStabilizeWait); // wait for camera stabilize
  stabilizeDelayWithCountdown(false);
  sendNikonCode(); // final signal - take photo
  takePhotoDelayWithCountdown();
  // delay(cameraShutterTime); // wait for camera take photo
  numberOfAlreadyTakenPhotos += 1;
}

void sendNikonCode()
{
  pulseIR(2080);
  delay(27);
  pulseIR(440);
  delayMicroseconds(1500);
  pulseIR(460);
  delayMicroseconds(3440);
  pulseIR(480);

  delay(65); // wait 65 milliseconds before sending it again

  pulseIR(2000);
  delay(27);
  pulseIR(440);
  delayMicroseconds(1500);
  pulseIR(460);
  delayMicroseconds(3440);
  pulseIR(480);
}


void setDelayMotorFrequency(int numOfSteps) {
  if(numOfSteps > 640){
    delayMotorFrequency = 500;
  }
  if(numOfSteps > 1200){
    delayMotorFrequency = 250;
  }
  if(numOfSteps > 1600){
    delayMotorFrequency = 150;
  }
   if(numOfSteps > 3200){
    delayMotorFrequency = 50;
  }
   if(numOfSteps > 6400){
    delayMotorFrequency = 25;
  }
}

/* STEPPER MOTOR */
//BACKWARD
void goBackward(int numOfSteps)
{
setDelayMotorFrequency(numOfSteps);
  digitalWrite(DIR_PIN, LOW); //Changes the rotations direction
  for (int x = 0; x < numOfSteps; x++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayMotorFrequency);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayMotorFrequency);
  }
  countNumberOfSteps(numOfSteps, false);
}

//FORWARD
void goForward(int numOfSteps)
{
 setDelayMotorFrequency(numOfSteps);

  digitalWrite(DIR_PIN, HIGH); // Enables the motor to move in a particular direction
  for (int x = 0; x < numOfSteps; x++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayMotorFrequency);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayMotorFrequency);
  }
  countNumberOfSteps(numOfSteps, true);
}

// START BUTTON
void checkButtonStart()
{
  buttonStart = mcp.digitalRead(3);
  if (buttonStart == HIGH)
  {
    toggleStartPosition();
    delay(250);
  }
  setDiodeStart(diodeStart);
}

// END BUTTON
void checkButtonEnd()
{
  buttonEnd = mcp.digitalRead(2);

  if (buttonEnd == HIGH)
  {
    toggleEndPosition();
    delay(250);
  }
  setDiodeEnd(diodeEnd);
}

void setDiodeStart(bool statue)
{
  mcp.digitalWrite(6, statue);
}

void setDiodeEnd(bool statue)
{
  mcp.digitalWrite(7, statue);
}

// FORWARD BUTTON
void checkButtonForward()
{
  buttonStateForward = mcp.digitalRead(1);
  if (buttonStateForward == HIGH)
  {
    goForward(stepVal);
    delay(delayBetweenStep);
  }

  if (buttonStateForward != lastButtonStateForward)
  {
    if (buttonStateForward == HIGH)
    {
      startPressed = millis();
      idleTime = startPressed - endPressed;
    }
    else
    {
      setToDefaultValues();
    }
  }
  else
  {
    if (buttonStateForward == HIGH)
    {
      holdTime = millis() - startPressed;

      if (holdTime >= buttonHoldTime)
      {
        // setStepToQuickMove();
      }

      // the button is still released
    }
    else
    {
      idleTime = millis() - endPressed;
    }
  }
}

// BACKWARD BUTTON
void checkButtonBackward()
{
  buttonStateBackward = mcp.digitalRead(0);
  if (buttonStateBackward == HIGH)
  {
    goBackward(stepVal);
    delay(delayBetweenStep);
  }

  if (buttonStateBackward != lastButtonStateBackward)
  {
    if (buttonStateBackward == HIGH)
    {
      startPressed = millis();
      idleTime = startPressed - endPressed;
    }
    else
    {
      setToDefaultValues();
    }
  }
  else
  {
    if (buttonStateBackward == HIGH)
    {
      holdTime = millis() - startPressed;

      if (holdTime >= buttonHoldTime)
      {
        // setStepToQuickMove();
      }

      // the button is still released
    }
    else
    {
      idleTime = millis() - endPressed;
    }
  }
}

void setToDefaultValues()
{
  endPressed = millis();
  holdTime = endPressed - startPressed;
  stepVal = setStepLength();
  delayBetweenStep = defaultDelayBetweenStep;
  delayMotorFrequency = defaultDelayMotorFrequency;
}

void setStepToQuickMove()
{
  stepVal = quickStepVal;
  delayBetweenStep = 0;
}

void toggleStartPosition()
{
  if (startPositionIsSet == false)
  {
    startPositionIsSet = true;
    diodeStart = 1;
  }
  else
  {
    startPositionIsSet = false;
    diodeStart = 0;
    autoMoveNumOfSteps = 0;
    eraseStartInfoOnLCD();
  }
}

void toggleEndPosition()
{
  if (startPositionIsSet == true && endPositionIsSet == false && autoMoveNumOfSteps != 0 && stepVal > 0)
  {
    endPositionIsSet = true;
    runAutoTakingPhoto();
  }
  else
  {
    endPositionIsSet = false;
    diodeEnd = 0;
  }
}

void countNumberOfSteps(int numOfSteps, bool forward)
{
  if (startPositionIsSet == true)
  {
    if (forward == true)
    {
      autoMoveNumOfSteps += numOfSteps;
    }
    else
    {
      autoMoveNumOfSteps -= numOfSteps;
    }
  }
  //Serial.println(autoMoveNumOfSteps);
}

bool isAutoRunInProgress()
{
  return isRunInProgress;
}

int getNumberOfPhotography()
{
    return abs(autoMoveNumOfSteps) / stepVal;
}

void runAutoTakingPhoto()
{

  if (startPositionIsSet == true && endPositionIsSet == true && autoMoveNumOfSteps != 0 && stepVal > 0)
  {
    numberOfAlreadyTakenPhotos = 0;
    isRunInProgress = true;
    diodeStart = 1;
    diodeEnd = 1;
    setDiodeEnd(1);
    Serial.println("start auto photo");
    blinkDiode(6); // START DIODE
    setDiodeStart(1);
    int numOfPhotos = abs(autoMoveNumOfSteps) / stepVal;
    int moveInOneStep = abs(autoMoveNumOfSteps) / numOfPhotos;

    Serial.println("Total steps:");
    Serial.println(autoMoveNumOfSteps);
    Serial.println("Number of photography:");
    Serial.println(numOfPhotos);
    Serial.println("Size of one step (40 = 0,1mm):");
    Serial.println(moveInOneStep);

    lcd.clear();
    printOnLCD(0,-1,"FOTOGRAFOVANI");

    if (autoMoveNumOfSteps > 0)
    { // GO BACKWARD
      for (int x = 0; x < numOfPhotos; x++)
      {
        showAutoProgressOnLcd(numOfPhotos);
        goBackward(moveInOneStep);
        delay(delayBetweenStep);
        Serial.println("krok");
        takePhotoAuto(mirrorLockUpActive); // is Mirror LockUp active?
        Serial.println("delam fotku");
        showAutoProgressOnLcd(numOfPhotos);
      }
    }
    else
    { // GO FORWARD
      for (int x = 0; x < numOfPhotos; x++)
      {
        showAutoProgressOnLcd(numOfPhotos);
        goForward(moveInOneStep);
        delay(delayBetweenStep);
        Serial.println("krok");
        takePhotoAuto(mirrorLockUpActive); // is Mirror LockUp active?
        Serial.println("delam fotku");
        showAutoProgressOnLcd(numOfPhotos);
      }
    }

    blinkDiode(7); // END DIODE
    diodeStart = 0;
    diodeEnd = 0;
    isRunInProgress = false;
    numberOfAlreadyTakenPhotos = 0;
    autoMoveNumOfSteps = 0;
    startPositionIsSet = false;
    lcd.clear();
  }
}

void blinkDiode(int pin) // 6 - START, 7 - END
{
  for (int x = 0; x < numberOfLedBlink; x++)
  {
    mcp.digitalWrite(pin, x % 2 == 0);
    delay(intervalLed);
  }
}

template<class TYPE>
void showOnLCD(int row, int col, int textPosition, TYPE msg) {
  if(col == -1){ // -1 == center text
    col = textPosition;
  }
  lcd.setCursor(col, row);
  lcd.print(msg);
}

void printOnLCD(int row, int col, char* msg) {
  showOnLCD(row, col, (SCREEN_WIDTH - strlen(msg))/2, msg);
}

void printOnLCD(int row, int col, float msg) {
  showOnLCD(row, col, (SCREEN_WIDTH - getNumberOfDigitsInInt(msg))/2, msg);
}

void printOnLCD(int row, int col, int msg) {
  showOnLCD(row, col, (SCREEN_WIDTH - getNumberOfDigitsInInt(msg))/2, msg);
}

void statusOnLCD(){
   unsigned long currentMillisLCD = millis();

  if (currentMillisLCD - previousMillisLCD >= intervalLCD) {

    previousMillisLCD = currentMillisLCD;
    float realStep = getRealStepLength();
    /* FIRST ROW */
    printOnLCD(0,0,"DELKA KROKU:");
    printOnLCD(0,13, realStep);
    printOnLCD(0,17, "mm");

    /* SECOND ROW */
    printOnLCD(1,0,"REZIM:");
    if(mirrorLockUpActive == true){
      printOnLCD(1,7,"MUP          ");
    }else{
      printOnLCD(1,7,"NORMALNI     ");
    }
    /* THIRD ROW + FOURTH ROW */
    if(startPositionIsSet == true){
      printOnLCD(2,0,"START NASTAVEN");
      printOnLCD(3,0,"FOTOGRAFII:");
      int numberOfPhotos = getNumberOfPhotography();
      printOnLCD(3,12, numberOfPhotos);
      for (int i = 19; i >= 12 + getNumberOfDigitsInInt(numberOfPhotos); i--)
      {
        printOnLCD(3,i," ");
      }
      
    }else{
     eraseStartInfoOnLCD();
    }
  }
}

void eraseStartInfoOnLCD(){
  clearRowOnLCD(2);
  clearRowOnLCD(3);
}

void showAutoProgressOnLcd(int numOfPhotos) {
      int charCount = getNumberOfDigitsInInt(numberOfAlreadyTakenPhotos) + 1 + getNumberOfDigitsInInt(numOfPhotos);
      int textPosition = (SCREEN_WIDTH - charCount)/2;
      printOnLCD(1,textPosition, numberOfAlreadyTakenPhotos);
      printOnLCD(1,textPosition+getNumberOfDigitsInInt(numberOfAlreadyTakenPhotos),"/");
      printOnLCD(1,textPosition+getNumberOfDigitsInInt(numberOfAlreadyTakenPhotos)+1,numOfPhotos);
}

int getNumberOfDigitsInInt(int x) {
    x = abs(x);  
    return (x < 10 ? 1 :
        (x < 100 ? 2 :
        (x < 1000 ? 3 :
        (x < 10000 ? 4 :
        (x < 100000 ? 5 :
        (x < 1000000 ? 6 :
        (x < 10000000 ? 7 :
        (x < 100000000 ? 8 :
        (x < 1000000000 ? 9 :
        10)))))))));  
}

int getPercentualProgress(int totalPhoto, int actualPhoto){
  int result = 100 / (totalPhoto / actualPhoto);
  return result;
}

bool getMirrorLockUpActiveBySwitch() {
    if(digitalRead(MUP_SWITCH) == 1){
      mirrorLockUpActive = true;
    }else{
      mirrorLockUpActive = false;
    }
}

void clearRowOnLCD(int row) {
   for (int i = 0; i < 20; i++)
      {
      printOnLCD(row,i," ");
      }
}
