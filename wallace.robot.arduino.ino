#include <SoftwareSerial.h>
//#include <stdlib.h>
#include "RoboClaw.h"


#define MYDEBUG 0



#define ARDUINO_USB_LOWEST_BAUD 9600
//#define ARDUINO_USB_BAUD 19200
//#define ARDUINO_USB_BAUD 57600
//#define ARDUINO_USB_BAUD 74880

#define ARDUINO_USB_MIDDLE_BAUD 115200

//#define ARDUINO_USB_BAUD 230400
//#define ARDUINO_USB_BAUD 250000
//#define ARDUINO_USB_BAUD 500000
//#define ARDUINO_USB_BAUD 1000000
#define ARDUINO_USB_HIGHEST_BAUD 2000000

#define ARDUINO_SERIAL_TO_ROBOCLAW_BAUD 38400
#define rcRxPin 10
#define rcTxPin 11
#define address 0x80
#define C2F_FACTOR 9/5
#define C2F(x) (x * C2F_FACTOR) + 32

#define MAX_USB_RX_BUF 64
#define MAX_PARM_BUF 6


#define DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST 100
#define TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME 100

enum  COMMAND {
  MENU = 0,
  STATUSSTOP = 2,
  STATUSSTART = 3,
  CLRAllERRORS = 4,
  RSTNUMUSBCMDS = 5,
  MINSPD2AMPS = 6,
  //MAXAMPS = 7,
  VERSION = 20,
  STATUS = 24,
  STOP = 28,
  FORWARD = 29,
  BACKWARD = 32,
  LEFT = 33,
  RIGHT = 34,
  FWDRESP = 35,
  BCKRESP = 36,
  LFTRESP = 37,
  RITRESP = 38
};




/////////////// this is the main serial input buffer to temporarily store whatever came in from USB serial /////////
char receivedChars[MAX_USB_RX_BUF];   // an array to store the received data

/////////////// these small buffers store whatever was parsed from the above input USB serial buffer ////////////
char numParms[MAX_PARM_BUF] = {'\0'};
char command[MAX_PARM_BUF] = {'\0'};
char randNum[MAX_PARM_BUF] = {'\0'};
char chksum[MAX_PARM_BUF] = {'\0'};
char param1[MAX_PARM_BUF]  = {'\0'};
char param2[MAX_PARM_BUF]  = {'\0'};

////////////// these flags control the program flow.  first we need new data. then we need to parse it. then we execute /////
bool thereIsRoboclawError = false;
bool thereIsUsbError = true;  //initialized to true, to force client program (Raspberry?  Browser?) to clear the flag
bool thereIsMovementError = false;
bool newData = false;
bool newCommandIsReady = false;
bool respond = false;


/////////// stuff related to auto-sending status back to host /////////////////////////////////////
bool continueToAutoSendStatusToHost = true;
unsigned long prevMillisLastTimeAutoSentStatus = millis();
//unsigned long motorCurrentsHighAtThisTime = millis();
unsigned long motorSpeedLowAtThisTime = millis();
//unsigned int highAmpsCount = 0;
unsigned int tooLowMotorSpeedCount = 0;
unsigned long autoSendStatusTimeout = DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST;
unsigned long lastTimeReadRoboclawStatus = millis();

//////////// these are roboclaw values that are set as a result of calling roboclaw library functions.
bool rcValid;
uint8_t rcStatus;

//////////// these are the other global values related to the roboclaw or arduino, of interest
char version[32];
uint16_t volts = 0;
int16_t amps1 = 0, amps2 = 0;
uint16_t temp = 0;
int32_t speedM1 = 0,
        speedM2 = 0;
String lastError;
long numCmdsRxdFromUSB = 0;
int lastCmdRxdFromUSB = -1;
long numDroppedUsbPackets = 0;
float minSpeedToAmps = 0;
long minSpeedToAmpsMillis = 1000;
float currSpeedToAmps = 0;
unsigned long prevMillisLastCommand = millis();

////////////  we need to know if motors were commanded to turn, as a safety feature, so we can shut them down.
bool motorM1Rotating = false;
bool motorM2Rotating = false;
bool motorSpeedMarkedAsLow = false;


SoftwareSerial serial(rcRxPin, rcTxPin);
RoboClaw roboclaw(&serial, 10000);                  //38400

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//  Typical Arduino setup
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////
//////////////////////////////////////////
void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(ARDUINO_USB_HIGHEST_BAUD);
  roboclaw.begin(ARDUINO_SERIAL_TO_ROBOCLAW_BAUD);  //38400
}

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // safety and / or status related functions
  getRoboclawStatus();
  stopMotorsIfBeenTooLongSinceLastCommand();
  //stopMotorsIfHighAmpsTooLong();
  stopMotorsIfSpeedTooLowForAmps();
  readStatusIfBeenTooLongSinceLastTime();

  //these functions will do (or not) something, based on the above global program-flow flags
  //Example:
  //  newData
  //  usbDataReadyToParse
  //  newCommandIsReady
  recvIncomingUsbSerialWithEndMarker();
  parseIncomingUsbSerial();
  commandHandler();

}



///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// Start of Functions
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////




void commandHandler() {

  if (!newCommandIsReady) return;

  prevMillisLastCommand = millis();

  long int cmd = strtol(command, NULL, 10);

  if (cmd != STATUS) {
    lastCmdRxdFromUSB = -1; lastCmdRxdFromUSB = cmd;
    numCmdsRxdFromUSB++;
  }


  switch (cmd) {

    //////////////////// Arduino user - instructions /////////////////////////
    case MENU: //help - send list of available commands
      showMenu();
      break;
    case CLRAllERRORS:
      thereIsUsbError = false;
      thereIsRoboclawError = false;
      thereIsMovementError = false;
      tooLowMotorSpeedCount = 0;

      break;
    case RSTNUMUSBCMDS:
      numCmdsRxdFromUSB = 0;
      numDroppedUsbPackets = 0;
      break;
    case STATUSSTOP:
      stopAutoSendingStatusToHost();
      break;
    case STATUSSTART:
      startAutoSendingStatusToHost();
      break;
    case MINSPD2AMPS:
      setMinSpeedToAmps();
      break;
    //case MAXAMPS:
      //setMaxAmps();
      //break;
    //////////////////// Roboclaw instructions /////////////////////////
    case VERSION: //version
      readRcVersion();
      break;
    case STATUS: //read volts, amps, temp
      //readRcVoltsAmpsTempSpeedsNumCmds();
      readStatus();
      break;
    case STOP: //stop both motors
      stopMotors();
      break;
    case FWDRESP:
      stopAutoSendingStatusToHost();
      respond = true;
    case FORWARD:
      if (thereIsUsbError || thereIsRoboclawError || thereIsMovementError) {
        if (respond) readStatus(); respond = false;
        return;
      }
      moveForward();
      if (respond) readStatus();
      respond = false;
      break;
    case BCKRESP:
      stopAutoSendingStatusToHost();
      respond = true;
    case BACKWARD:
      if (thereIsUsbError || thereIsRoboclawError || thereIsMovementError) {
        if (respond) readStatus(); respond = false;
        return;
      }
      moveBackward();
      if (respond) readStatus();
      respond = false;
      break;
    case LFTRESP:
      stopAutoSendingStatusToHost();
      respond = true;
    case LEFT:
      if (thereIsUsbError || thereIsRoboclawError || thereIsMovementError) {
        if (respond) readStatus(); respond = false;
        return;
      }
      rotateLeft();
      if (respond) readStatus();
      respond = false;
      break;
    case RITRESP:
      stopAutoSendingStatusToHost();
      respond = true;
    case RIGHT:
      if (thereIsUsbError || thereIsRoboclawError || thereIsMovementError) {
        if (respond) readStatus(); respond = false;
        return;
      }
      rotateRight();
      if (respond) readStatus();
      respond = false;
      break;

    default:
      lastError = "\"UNKCMD [";
      lastError.concat(lastCmdRxdFromUSB);
      lastError.concat("]\"");
      String usbError = "{\"error\":";
      usbError.concat(lastError);
      usbError.concat("}");
      Serial.println(usbError);
      thereIsUsbError = true;
      stopMotors();
  }

  //this may be optional.. but the Roboclaw takes a certain amount of time to do stuff
  //and I didnt want to be stepping on it by having yet another command (or other function)
  // interfering during that time.
  delay(10);

  newCommandIsReady = false;
  newData = false;
}

//////////////////// Arduino user - instructions functions /////////////////////////
//////////////////// Arduino user - instructions functions /////////////////////////
//////////////////// Arduino user - instructions functions /////////////////////////
void showMenu() {
  Serial.println("{\"msg\":\"20 - version\"}");
  Serial.println("{\"msg\":\"24 - one-time status\"}");
  Serial.println("{\"msg\":\"28 - stop motors\"}");
  Serial.println("{\"msg\":\"29 - forward <spd>\"}");
  Serial.println("{\"msg\":\"32 - back <spd>\"}");
  Serial.println("{\"msg\":\"33 - left <spd>\"}");
  Serial.println("{\"msg\":\"34 - right <spd>\"}");
  Serial.println("{\"msg\":\"35 - fwdresp <spd>\"}");
  Serial.println("{\"msg\":\"36 - backresp <spd>\"}");
  Serial.println("{\"msg\":\"37 - leftresp <spd>\"}");
  Serial.println("{\"msg\":\"38 - rightresp <spd>\"}");
  Serial.println("{\"msg\":\"0 - help\"}");
  Serial.println("{\"msg\":\"1 - num cmds rxd\"}");
  Serial.println("{\"msg\":\"2 - stop auto send status\"}");
  Serial.println("{\"msg\":\"3 - start auto send status <timeout>\"}");
  Serial.println("{\"msg\":\"4 - clear USB error\"}");
  Serial.println("{\"msg\":\"5 - reset num USB cmds rxd\"}");
  //Serial.println("{\"msg\":\"6 - chg USB Baud <1=9600, 2=115200, 3=2000000\"}");
  //Serial.println("{\"msg\":\"7 - set Max Amps <amps>\"}");
}


//void setMaxAmps() {
  //maxAmps = atof(param1);
  //maxAmpsMillis = strtol(param2, NULL, 10);
//}

void setMinSpeedToAmps() {
  minSpeedToAmps = atof(param1);
  minSpeedToAmpsMillis = strtol(param2, NULL, 10);
}

void getNumCmdsRxdFromUSB() {
  String message = "{\"numCmds\":";
  message.concat(numCmdsRxdFromUSB);
  message.concat("}");
  Serial.println(message);
}


void stopAutoSendingStatusToHost() {
  continueToAutoSendStatusToHost = false;
}

void startAutoSendingStatusToHost() {
  continueToAutoSendStatusToHost = true;
  autoSendStatusTimeout = strtol(param1, NULL, 10);
}

void returnCommandResponse(long int cmd) {
  String results = "{\"ackcmd\":";
  results.concat(cmd);
  results.concat("}");
  Serial.println(results);
  respond = false;
}

//////////////////// Roboclaw instructions ////////////////////////////////////////////
//////////////////// Roboclaw instructions ////////////////////////////////////////////
//////////////////// Roboclaw instructions ////////////////////////////////////////////
void readRcVersion() {
  for (byte i = 0; i < sizeof(version); i++) {
    version[i] = 0;
  }
  if (roboclaw.ReadVersion(address, version)) {
    String message = "{\"version\":\"";
    message.concat(version);
    message.concat("\"}");
    Serial.println(message);
  } else {
    lastError = "\"ERRVER\"";
    Serial.println(lastError);
    thereIsRoboclawError = true;
  }
}



void readRcVoltage() {
  rcValid = false;
  volts = roboclaw.ReadMainBatteryVoltage(address, &rcValid);
  if (rcValid) {
  } else {
    volts = -1;
    lastError = "\"VOLTS\"";
    thereIsRoboclawError = true;
  }
}


void readRcCurrents() {
  if (roboclaw.ReadCurrents(address, amps1, amps2)) {
  } else {
    amps1 = -1;
    amps2 = -1;
    lastError = "\"AMPS\"";
    thereIsRoboclawError = true;
  }
}


void readRcTemperature() {
  if (roboclaw.ReadTemp(address, temp)) {
  } else {
    temp = -1;
    lastError = "\"TEMP\"";
    thereIsRoboclawError = true;
  }
}

void getRoboclawStatus() {

  if (millis() - lastTimeReadRoboclawStatus > 30) {
    readRcVoltage();
    readRcCurrents();
    readRcTemperature();
    readRcMotorSpeeds();
    lastTimeReadRoboclawStatus = millis();
  }
}

void readStatus() {
  String results = "{\"v\":";
  results.concat(volts / 10.0f);
  results.concat(",\"a1\":");
  results.concat(amps1 / 100.0f);
  results.concat(",\"a2\":");
  results.concat(amps2 / 100.0f);
  results.concat(",\"t\":");
  results.concat(C2F(temp / 10.0f));
  results.concat(",\"s1\":");
  results.concat(speedM1);
  results.concat(",\"s2\":");
  results.concat(speedM2);
  results.concat(",\"cmds\":");
  results.concat(numCmdsRxdFromUSB);
  results.concat(",\"drop\":");
  results.concat(numDroppedUsbPackets);
  results.concat(",\"last\":");
  results.concat(lastCmdRxdFromUSB);
  if (thereIsUsbError || thereIsRoboclawError || thereIsMovementError) {
    results.concat(",\"error\":");
    results.concat(lastError);
    //results.concat(",\"hiAmpsCnt\":");
    //results.concat(highAmpsCount);
    results.concat(",\"loSpdCnt\":");
    results.concat(tooLowMotorSpeedCount);
    results.concat(",\"sp2amp\":");
    results.concat(currSpeedToAmps);
  }
  results.concat("}");
  Serial.println(results);

}

void readRcMotorSpeeds() {
  rcStatus = 0; rcValid = false;
  speedM1 = roboclaw.ReadSpeedM1(address, &rcStatus, &rcValid);
  if (!rcValid) {
    speedM1 = -1;
    lastError = "\"SPDM1\"";
    thereIsRoboclawError = true;
  }
  rcStatus = 0; rcValid = false;
  speedM2 = roboclaw.ReadSpeedM2(address, &rcStatus, &rcValid);
  if (!rcValid) {
    speedM2 = -1;
    lastError = "\"SPDM2\"";
    thereIsRoboclawError = true;
  }
}

void moveForward() {
  rotateLeftSideForward(param1);
  rotateRightSideForward(param1);
}

void moveBackward() {
  rotateLeftSideBackward(param1);
  rotateRightSideBackward(param1);
}

void rotateLeft() {
  rotateLeftSideBackward(param1);
  rotateRightSideForward(param1);
}

void rotateRight() {
  rotateLeftSideForward(param1);
  rotateRightSideBackward(param1);
}

void rotateLeftSideForward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.ForwardM1(address, speed)) {
    motorM1Rotating = true;
  } else {
    lastError = "\"ERRROTM1F\"";
    thereIsRoboclawError = true;
  }
}

void rotateRightSideForward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.ForwardM2(address, speed)) {
    motorM2Rotating = true;
  } else {
    lastError = "\"ERRROTM2F\"";
    thereIsRoboclawError = true;
  }
}

void stopMotors() {
  uint8_t speed = 0;
  if (roboclaw.ForwardM1(address, speed)) {
    motorM1Rotating = false;
  } else {
    lastError = "\"ERRSTOPM1\"";
    thereIsRoboclawError = true;
  }
  if (roboclaw.ForwardM2(address, speed)) {

    motorM2Rotating = false;
  } else {
    lastError = "\"ERRROTM1F\"";
    thereIsRoboclawError = true;
  }
}

void rotateLeftSideBackward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.BackwardM1(address, speed)) {
    motorM1Rotating = true;
  } else {
    lastError = "\"ERRROTM1B\"";
    thereIsRoboclawError = true;
  }
}

void rotateRightSideBackward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.BackwardM2(address, speed)) {
    motorM2Rotating = true;
  } else {
    lastError = "\"ERRROTM2B\"";
    thereIsRoboclawError = true;
  }
}


//////////////////// private functions /////////////////////////
//////////////////// private functions /////////////////////////
//////////////////// private functions /////////////////////////

void recvIncomingUsbSerialWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= MAX_USB_RX_BUF) {
        ndx = MAX_USB_RX_BUF - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }

}


void parseIncomingUsbSerial() {

  if (newData != true) return;

  clearBuffer(numParms, MAX_PARM_BUF);
  clearBuffer(command, MAX_PARM_BUF);
  clearBuffer(randNum, MAX_PARM_BUF);
  clearBuffer(chksum, MAX_PARM_BUF);
  clearBuffer(param1, MAX_PARM_BUF);
  clearBuffer(param2, MAX_PARM_BUF);

  //Serial.println("..parsing..");


  byte ndx = 0;
  byte pidx = 0;
  byte sidx = 0;

  /////   extract numParms out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    numParms[ndx] = receivedChars[ndx];
    ndx++;
  }
  /*
        Serial.print(F("numParms["));
        Serial.print(numParms);
        Serial.print(F("] "));
        Serial.print(pidx);
        Serial.print(F(" "));
        Serial.println(ndx);
  */
  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract  command out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_RX_BUF && ndx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    command[pidx] = receivedChars[ndx];
    pidx++;
    ndx++;
  }
  /*
        Serial.print(F("command["));
        Serial.print(command);
        Serial.print(F("]"));
        Serial.println(ndx);
  */

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract randNum out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    randNum[pidx] = receivedChars[ndx];
    ndx++;
    pidx++;
  }
  /*
        Serial.print(F("randNum["));
        Serial.print(randNum);
        Serial.print(F("] "));
        Serial.print(pidx);
        Serial.print(F(" "));
        Serial.println(ndx);
  */
  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract chksum out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    chksum[pidx] = receivedChars[ndx];
    ndx++;
    pidx++;
  }
  /*
        Serial.print(F("chksum["));
        Serial.print(chksum);
        Serial.print(F("] "));
        Serial.print(pidx);
        Serial.print(F(" "));
        Serial.println(ndx);
  */
  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    ndx++;
  }



  /////   extract param1 out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    param1[pidx] = receivedChars[ndx];
    ndx++;
    pidx++;
  }

  /*
        Serial.print(F("param1["));
        Serial.print(param1);
        Serial.print(F("] "));
        Serial.print(pidx);
        Serial.print(F(" "));
        Serial.println(ndx);
  */

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    ndx++;
  }



  /////   extract param2 out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
    param2[pidx] = receivedChars[ndx];
    ndx++;
    pidx++;
  }
  /*
        Serial.print(F("param2["));
        Serial.print(param2);
        Serial.print(F("] "));
        Serial.print(pidx);
        Serial.print(F(" "));
        Serial.println(ndx);
  */

  clearBuffer(receivedChars, MAX_USB_RX_BUF);

  if (verifyChecksum()) {
    newCommandIsReady = true;
    //Serial.println(F("command verified"));
  } else {
    numDroppedUsbPackets++;
    //Serial.println(F("command NOT verified"));
    newData = false; // whatever was rxd from USB was bad, so we're just ignoring it and are now ready to receive more from USB
  }


}

bool verifyChecksum() {

  int numberOfUsbParameters = 0;
  if (strlen(numParms) < 1) return false;
  numberOfUsbParameters++;
  if (strlen(command) < 1) return false;
  numberOfUsbParameters++;
  if (strlen(randNum) < 1) return false;
  numberOfUsbParameters++;
  if (strlen(chksum) < 1) return false;
  numberOfUsbParameters++;
  if (strlen(param1) > 0) numberOfUsbParameters++;
  if (strlen(param2) > 0) numberOfUsbParameters++;

  //Serial.println(F("...still verifying.."));
  long int nParms = atoi(numParms);

  //Serial.print(numberOfUsbParameters);Serial.print(' ');Serial.println(nParms);
  if (numberOfUsbParameters != nParms) return false;

  long int cmd = atoi(command);
  long int rnd = atoi(randNum);
  long int chksm = atoi(chksum);
  long int p1 = atoi(param1);
  long int p2 = atoi(param2);

  long int mySum = nParms + cmd + rnd + p1 + p2;

  //Serial.print(cmd);Serial.print(' ');Serial.print(rnd);Serial.print(' ');Serial.print(chksum);Serial.print(' ');Serial.print(p1);Serial.print(' ');Serial.println(mySum);
  if (mySum == chksm) {
    return true;
  } else {
    return false;
  }
}

void stopMotorsIfSpeedTooLowForAmps() {
  
  //this starts tracking of how long motor speed is too low for the amps
  if (!thereIsMovementError && !motorSpeedMarkedAsLow && motorSpeedTooLowForAmps()) {
    motorSpeedMarkedAsLow = true;
    motorSpeedLowAtThisTime = millis();
  }

  //this marks the end of the tracking of the above period
  //AND starts the tracking of how long to give motor speed a chance to be good long enough.
  if (!thereIsMovementError && motorSpeedMarkedAsLow && motorSpeedTooLowForAmps() && motorSpeedTooLowForAmpsTooLong()) {
    lastError = "\"MOTORSPD2LOW\"";
    thereIsMovementError = true;
  }

  if (thereIsMovementError && motorSpeedMarkedAsLow && !motorSpeedTooLowForAmps()) {
    motorSpeedMarkedAsLow = false;
    motorSpeedLowAtThisTime = millis();
  }

  if (thereIsMovementError && !motorSpeedMarkedAsLow && !motorSpeedTooLowForAmps() && motorSpeedIsGoodLongEnough()) {
    if (tooLowMotorSpeedCount > 2) return;
    tooLowMotorSpeedCount++;
    thereIsMovementError = false;
  }
}

bool motorSpeedTooLowForAmps() {
  float m1s2a = speedM1/(amps1/100.0f);
  float m2s2a = speedM2/(amps2/100.0f);
  m1s2a = m1s2a<0? -m1s2a : m1s2a;
  m2s2a = m2s2a<0? -m2s2a : m2s2a;
  currSpeedToAmps = m1s2a < m2s2a ? m1s2a : m2s2a;
  return (amps1/100.0f > 0.5 || amps2/100.0f > 0.5) && (m1s2a < minSpeedToAmps || m2s2a < minSpeedToAmps);
}
bool motorSpeedTooLowForAmpsTooLong() {
  return (millis() - motorSpeedLowAtThisTime > minSpeedToAmpsMillis);
}
bool motorSpeedIsGoodLongEnough() {
  return (millis() - motorSpeedLowAtThisTime > 1000);
}

void stopMotorsIfBeenTooLongSinceLastCommand() {
  if (motorM1Rotating || motorM2Rotating) {
    if (millis() - prevMillisLastCommand > TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME) {
      stopMotors();
    }
  }
}

void readStatusIfBeenTooLongSinceLastTime() {
  if (continueToAutoSendStatusToHost && ((millis() - prevMillisLastTimeAutoSentStatus) > autoSendStatusTimeout)) {
    prevMillisLastTimeAutoSentStatus = millis();
    //readRcVoltsAmpsTempSpeedsNumCmds();
    readStatus();
  }
}

void clearBuffer(char* pBuf, byte len) {
  for (byte i = 0; i < len; i++) {
    pBuf[i] = '\0';
  }
}
