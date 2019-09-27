#include <SoftwareSerial.h>
#include "RoboClaw.h"


#define MYDEBUG 0



#define ARDUINO_USB_BAUD 115200
#define ARDUINO_SERIAL_TO_ROBOCLAW_BAUD 38400
#define rcRxPin 10
#define rcTxPin 11
#define address 0x80
#define C2F_FACTOR 9/5
#define C2F(x) (x * C2F_FACTOR) + 32

#define MAX_USB_RX_BUF 64
#define MAX_CMD_BUF 4
#define MAX_PARM_BUF 6


#define DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST 500
#define TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME 200

enum  COMMAND {
  MENU = 0,
  STATUSSTOP = 2,
  STATUSSTART =3,
  VERSION = 20,
//  VOLTS,
//  AMPS,
//  TEMP, 
//  SPEED,
  STATUS = 24,
//  FWDM1,
//  FWDM2,
  STOP = 28,
  FORWARD = 29,
//  ROTLEFTBACK,
//  ROTRIGHTBACK,
  BACKWARD = 32,
  LEFT = 33,
  RIGHT = 34
};




/////////////// this is the main serial input buffer to temporarily store whatever came in from USB serial /////////
char receivedChars[MAX_USB_RX_BUF];   // an array to store the received data

/////////////// these small buffers store whatever was parsed from the above input USB serial buffer ////////////
char command[MAX_CMD_BUF] = {'\0'};
char param1[MAX_PARM_BUF]  = {'\0'};
char param2[MAX_PARM_BUF]  = {'\0'};

////////////// these flags control the program flow.  first we need new data. then we need to parse it. then we execute /////
bool thereIsAnError = false;
bool newData = false;
bool usbDataReadyToParse = false;
bool newCommandIsReady = false;


///////////// for every command we incr this.  the host can request this value. if arduino had reset, this value would
///////////// start at 0 again, indicating a problem.
long numCmdsRxdFromUSB = 0;
String lastCmdRxdFromUSB = "none";

/////////// stuff related to auto-sending status back to host /////////////////////////////////////
bool continueToAutoSendStatusToHost = true;
unsigned long prevMillisLastTimeAutoSentStatus = millis();
unsigned long autoSendStatusTimeout = DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST;


//////////// these are roboclaw values that are set as a result of calling roboclaw library functions.
bool rcValid;
uint8_t rcStatus;

//////////// these are the other global values related to the roboclaw, of interest
char version[32];
uint16_t volts = 0;
int16_t amps1 = 0, amps2 = 0;
uint16_t temp = 0;
int32_t prevSpeedM1 = 0, prevSpeedM2 = 0;
int32_t speedM1 = 0, speedM2 = 0;

unsigned long prevMillisLastCommand = millis();
unsigned long nowMillis = millis();

////////////  we need to know if motors were commanded to turn, as a safety feature, so we can shut them down.
bool motorM1Rotating = false;
bool motorM2Rotating = false;



SoftwareSerial serial(rcRxPin, rcTxPin);
RoboClaw roboclaw(&serial, 10000);

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//  Typical Arduino setup
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(ARDUINO_USB_BAUD);
  roboclaw.begin(ARDUINO_SERIAL_TO_ROBOCLAW_BAUD);
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
  stopMotorsIfBeenTooLongSinceLastCommand();
  readRcVoltsAmpsTempIfBeenTooLongSinceLastTime();

  //these functions will do (or not) something, based on the above global program-flow flags
  //Example:
  //  newData
  //  usbDataReadyToParse
  //  newCommandIsReady
  recvIncomingUsbSerialWithEndMarker();
  showIncomingUsbSerial();
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
  numCmdsRxdFromUSB++;

  long int cmd = strtol(command, NULL, 10);

  if (cmd != STATUS) {
    lastCmdRxdFromUSB = ""; lastCmdRxdFromUSB.concat(cmd);
  }
  
  switch (cmd) {

    //////////////////// Arduino user - instructions /////////////////////////
    case MENU: //help - send list of available commands
      getHelp();
      break;
/*
    case 1: //num cmds rxd from USB
      getNumCmdsRxdFromUSB();
      break;
*/
    case STATUSSTOP:
        stopAutoSendingStatusToHost();
        break;
    case STATUSSTART:
        startAutoSendingStatusToHost();
        break;
    //////////////////// Roboclaw instructions /////////////////////////
    case VERSION: //version
      readRcVersion();
      break;
    /*
    case VOLTS: //voltage
      readRcVoltage(false);
      break;
    case AMPS: //amps
      readRcCurrents(false);
      break;
    case TEMP: //temperature
      readRcTemperature(false);
      break;
    */
    case STATUS: //read volts, amps, temp
      readRcVoltsAmpsTempSpeedsNumCmds();
      break;
    /*
    case SPEED: //read both motors speed
      readRcMotorSpeeds(false);
      break;
    */
    /*
    // eliminating option to only move one side of robot
    case FWDM1: //move motor 1 forward
      rotateLeftSideForward(param1, true);
      break;
    case FWDM2: //move motor 2 forward
      rotateRightSideForward(param1, true);
      break;
*/
    case STOP: //stop both motors
      stopMotors();
      break;
    case FORWARD:
      moveForward();
      break;
    /*
    // eliminating option to only move one side of robot
    case ROTLEFTBACK:
      rotateLeftSideBackward(param1, true);
      break;
    case ROTRIGHTBACK:
      rotateRightSideBackward(param1, true);
      break;
      */
    case BACKWARD:
      moveBackward();
      break;
    case LEFT:
      rotateLeft();
      break;
    case RIGHT:
      rotateRight();
      break;
    default:
      String error = "{\"error\":\"UNKCMD [";
      error.concat(lastCmdRxdFromUSB);
      error.concat("]\"}");
      Serial.println(error);
  }

  //this may be optional.. but the Roboclaw takes a certain amount of time to do stuff
  //and I didnt want to be stepping on it by having yet another command (or other function)
  // interfering during that time.
  delay(10);

  usbDataReadyToParse = false;
  newCommandIsReady = false;
}

//////////////////// Arduino user - instructions functions /////////////////////////
//////////////////// Arduino user - instructions functions /////////////////////////
//////////////////// Arduino user - instructions functions /////////////////////////
void getHelp() {
  Serial.println("{\"msg\":\"20 - version\"}");
  //Serial.println("{\"msg\":\"21 - volts\"}");
  //Serial.println("{\"msg\":\"22 - amps\"}");
  //Serial.println("{\"msg\":\"23 - temp\"}");
  Serial.println("{\"msg\":\"24 - volts,amps,temp,speeds\"}");
  //Serial.println("{\"msg\":\"25 - read speeds\"}");
  //Serial.println("{\"msg\":\"26 - rot M1 fwd\"}");
  //Serial.println("{\"msg\":\"27 - rot M2 fwd\"}");
  Serial.println("{\"msg\":\"28 - stop motors\"}");
  Serial.println("{\"msg\":\"29 - move forward <spd> <spd>\"}");
  //Serial.println("{\"msg\":\"30 - rot M1 bck\"}");
  //Serial.println("{\"msg\":\"31 - rot M2 bck\"}");
  Serial.println("{\"msg\":\"32 - move back <spd> <spd>\"}");
  Serial.println("{\"msg\":\"33 - rot left <spd> <spd>\"}");
  Serial.println("{\"msg\":\"34 - rot right <spd> <spd>\"}");
  Serial.println("{\"msg\":\"0 - help\"}");
  Serial.println("{\"msg\":\"1 - num cmds rxd\"}");
  Serial.println("{\"msg\":\"2 - stop auto send status\"}");
  Serial.println("{\"msg\":\"3 - start auto send status <timeout>\"}");
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
    Serial.println("{\"error\":\"ERRVER\"}");
    thereIsAnError = true;
  }
}


/*
   Read Main Battery Voltage Level
   Read the main battery voltage level connected to B+ and B- terminals.
   The voltage is returned in 10ths of a volt(eg 300 = 30v).
*/
void readRcVoltage() {

#if MYDEBUG
  Serial.println("reading voltage");
#endif

  rcValid = false;
  volts = roboclaw.ReadMainBatteryVoltage(address, &rcValid);
  if (rcValid) {
#if MYDEBUG
    Serial.println(volts / 10.0f);
#endif
  } else {
#if MYDEBUG
    Serial.println("ERVOLT");
#endif
    volts = -1;
    thereIsAnError = true;
  }
}

/*
    Read Motor Currents
   Read the current draw from each motor in 10ma increments.
   The amps value is calculated by dividing the value by 100.
*/
void readRcCurrents() {
#if MYDEBUG
  Serial.println("reading currents");
#endif

  if (roboclaw.ReadCurrents(address, amps1, amps2)) {
#if MYDEBUG
    Serial.print(amps1 / 100.0f);
    Serial.print(" , ");
    Serial.println(amps2 / 100.0f);
#endif
  } else {
#if MYDEBUG
    Serial.println("ERAMPS");
#endif
    amps1 = -1;
    amps2 = -1;
    thereIsAnError = true;
  }

}

/*
   Read Temperature
   Read the board temperature. Value returned is in 10ths of degrees.
*/
void readRcTemperature() {
  Serial.println("reading temp");

  if (roboclaw.ReadTemp(address, temp)) {
#if MYDEBUG
    Serial.println(C2F(temp / 10.0f));
#endif
  } else {
#if MYDEBUG
    Serial.println("ERTEMP");
#endif
    temp = -1;
    thereIsAnError = true;
  }
}

void readRcVoltsAmpsTempSpeedsNumCmds() {
  readRcVoltage();
  readRcCurrents();
  readRcTemperature();
  readRcMotorSpeeds();
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
  results.concat(",\"last\":");
  results.concat(lastCmdRxdFromUSB);
  results.concat("}");
  Serial.println(results);
}

void readRcMotorSpeeds() {

#if MYDEBUG
  Serial.println("reading motor speeds");
#endif

  rcStatus = 0; rcValid = false;
  speedM1 = roboclaw.ReadSpeedM1(address, &rcStatus, &rcValid);
  if (!rcValid) {
#if MYDEBUG
    Serial.print("ERRDSPM1 : "); Serial.print(rcStatus, HEX); Serial.print(" "); Serial.println(rcValid);
#endif
    speedM1 = -1;
    thereIsAnError = true;
  }

  rcStatus = 0; rcValid = false;
  speedM2 = roboclaw.ReadSpeedM2(address, &rcStatus, &rcValid);
  if (!rcValid) {
#if MYDEBUG
    Serial.print("ERRDSPM2 : "); Serial.print(rcStatus, HEX); Serial.print(" "); Serial.println(rcValid);
#endif
    speedM2 = -1;
    thereIsAnError = true;
  }

#if MYDEBUG
    String results = "{\"s1\":";
    results.concat(speedM1);
    results.concat(",\"s2\":");
    results.concat(speedM2);
    results.concat("}");
    Serial.println(results);
#endif
}

void moveForward() {
  rotateLeftSideForward(param1);
  rotateRightSideForward(param2);
}

void moveBackward() {
  rotateLeftSideBackward(param1);
  rotateRightSideBackward(param2);
}

void rotateLeft() {
  rotateLeftSideBackward(param1);
  rotateRightSideForward(param2);
}

void rotateRight() {
  rotateLeftSideForward(param1);
  rotateRightSideBackward(param2);
}

void rotateLeftSideForward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.ForwardM1(address, speed)) {
    motorM1Rotating = true;
  } else {
    Serial.println("{\"error\":\"ERRROTM1F\"}");
    thereIsAnError = true;
  }
}

void rotateRightSideForward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.ForwardM2(address, speed)) {
    motorM2Rotating = true;
  } else {
    Serial.println("{\"error\":\"ERRROTM2F\"}");
    thereIsAnError = true;
  }
}

void stopMotors() {
  uint8_t speed = 0;
  if (roboclaw.ForwardM1(address, speed)) {
#if MYDEBUG
    Serial.println("M1 stopped");
#endif
    motorM1Rotating = false;
  } else {
    Serial.println("{\"error\":\"ERRSTOPM1\"}");
    thereIsAnError = true;
  }
  if (roboclaw.ForwardM2(address, speed)) {
#if MYDEBUG
    Serial.println("M2 stopped");
#endif
    motorM2Rotating = false;
  } else {
    Serial.println("{\"error\":\"ERRROTM1F\"}");
    thereIsAnError = true;
  }
}

void rotateLeftSideBackward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.BackwardM1(address, speed)) {
    motorM1Rotating = true;
  } else {
    Serial.println("{\"error\":\"ERRROTM1B\"}");
    thereIsAnError = true;
  }
}

void rotateRightSideBackward(char* param) {
  uint8_t speed = (uint8_t)strtol(param, NULL, 10);
  if (roboclaw.BackwardM2(address, speed)) {
    motorM2Rotating = true;
  } else {
    Serial.println("{\"error\":\"ERRROTM2B\"}");
    thereIsAnError = true;
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
#if MYDEBUG
      Serial.println("{\"msg\":\"Got end marker.\"}");
#endif
    }
  }

}

void showIncomingUsbSerial() {
  if (newData == true) {
    if (receivedChars[0] != '\n' && receivedChars[0] != ' ' && receivedChars[0] != '\t' && receivedChars[0] != '\r' && receivedChars[0] != 0) {
#if MYDEBUG      
      Serial.print("This just in ... ["); Serial.print(receivedChars); Serial.println("]"); }
#endif
      usbDataReadyToParse = true;
    }
    newData = false;
  }
}

void parseIncomingUsbSerial() {

  if (usbDataReadyToParse == true) {
    clearBuffer(command, MAX_CMD_BUF);
    clearBuffer(param1, MAX_PARM_BUF);
    clearBuffer(param2, MAX_PARM_BUF);
#if MYDEBUG
    Serial.println("..parsing..");
#endif
    
    byte ndx = 0;

    /////   extract  command out of the receive buffer
    while (ndx < MAX_USB_RX_BUF && ndx < MAX_CMD_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      command[ndx] = receivedChars[ndx];
      ndx++;
    }
#if MYDEBUG
      Serial.print("command[");
      Serial.print(command);
      Serial.print("] ");
      Serial.println(ndx);
#endif

    /////   need to get past whitespace (except newline)
    byte sidx = 0;
    while (ndx < MAX_USB_RX_BUF && (receivedChars[ndx] == ' ' || receivedChars[ndx] == '\t')) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      ndx++;
    }

    /////   extract param1 out of the receive buffer
    byte pidx = 0;
    while (ndx < MAX_USB_RX_BUF && pidx < MAX_PARM_BUF - 1 && receivedChars[ndx] != '\n' && receivedChars[ndx] != ' ' && receivedChars[ndx] != '\t' && receivedChars[ndx] != '\r' && receivedChars[ndx] != 0) {
      //Serial.print(ndx); Serial.print(", ["); Serial.print(receivedChars[ndx]); Serial.println("]");
      param1[pidx] = receivedChars[ndx];
      ndx++;
      pidx++;
    }
#if MYDEBUG
      Serial.print("param1[");
      Serial.print(param1);
      Serial.print("] ");
      Serial.print(pidx);
      Serial.print(" ");
      Serial.println(ndx);
#endif

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
#if MYDEBUG
      Serial.print("param2[");
      Serial.print(param2);
      Serial.print("] ");
      Serial.print(pidx);
      Serial.print(" ");
      Serial.println(ndx);
#endif

    clearBuffer(receivedChars, MAX_USB_RX_BUF);
    newCommandIsReady = true;
  }

}


void stopMotorsIfBeenTooLongSinceLastCommand() {
  if (motorM1Rotating || motorM2Rotating) {
    nowMillis = millis();
    if (nowMillis - prevMillisLastCommand > TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME) {
      stopMotors();
    }
  }
}

void readRcVoltsAmpsTempIfBeenTooLongSinceLastTime() {
    nowMillis = millis();
    if (continueToAutoSendStatusToHost && ((nowMillis - prevMillisLastTimeAutoSentStatus) > autoSendStatusTimeout)) {
      prevMillisLastTimeAutoSentStatus = millis();
      readRcVoltsAmpsTempSpeedsNumCmds();      
    }
}

void clearBuffer(char* pBuf, byte len) {
  for (byte i = 0; i < len; i++) {
    pBuf[i] = '\0';
  }
}
