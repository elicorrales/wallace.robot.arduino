#include <SoftwareSerial.h>
#include "RoboClaw.h"


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  GLOBAL DEFINES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

#define MAX_USB_TX_RX_BUF 256
#define MAX_RX_PARM_BUF 10
#define MAX_TX_BUF 64

#define DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST 50
#define TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME 100


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE COMMANDS THE ARDUINO UNDERSTANDS RECEIVED VIA USB SERIAL
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
enum COMMAND {
  //MENU = 0,
  ACKCMDS = 1,
  STATUSSTOP = 2,
  STATUSSTART = 3,
  CLRAllERRORS = 4,
  RSTNUMUSBCMDS = 5,
  //MINSPD2CMD = 6,
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

const char* bJSON = "{";
const char* eJSON = "}";
const char* sep = ",";
const char* v = "\"v\":";
const char* a1= "\"a1\":";
const char* a2= "\"a2\":";
const char* t = "\"t\":";
const char* s1= "\"s1\":";
const char* s2= "\"s2\":";
const char* c = "\"c\":";
const char* d = "\"d\":";
const char* l = "\"l\":";
const char* e = "\"e\":";
const char* quote = "\"";


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE SERIAL COMM BUFERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////// this is the main serial input buffer to temporarily store whatever came in from USB serial /////////
char mainTxRxBuffer[MAX_USB_TX_RX_BUF];   // an array to store the received data
char tempHoldingBuf[MAX_RX_PARM_BUF] = {'\0'};

/////////////// these small buffers store whatever was parsed from the above input USB serial buffer ////////////
char numParms[MAX_RX_PARM_BUF] = {'\0'};
char command[MAX_RX_PARM_BUF] = {'\0'};
char randNum[MAX_RX_PARM_BUF] = {'\0'};
char chksum[MAX_RX_PARM_BUF] = {'\0'};
char param1[MAX_RX_PARM_BUF]  = {'\0'};
char param2[MAX_RX_PARM_BUF]  = {'\0'};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE PROGRAM FLOW-CONTROL FLAGS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool userHasInitiatedArduino = false;
bool thereIsRoboclawError = false;
bool thereIsUsbError = true;  //initialized to true, to force client program (Raspberry?  Browser?) to clear the flag
bool newData = false;
bool newCommandIsReady = false;
bool respond = false;
bool ackCommands = false;

/////////// stuff related to auto-sending status back to host /////////////////////////////////////
bool continueToAutoSendStatusToHost = true;
unsigned long prevMillisLastTimeAutoSentStatus = millis();
unsigned long autoSendStatusTimeout = DEFAULT_TIMEOUT_SEND_STATUS_TO_HOST;
unsigned long lastTimeReadRoboclawStatus = millis();

//////////// these are roboclaw values that are set as a result of calling roboclaw library functions.
bool rcValid;
uint8_t rcStatus;

//////////// these are the other global values related to the roboclaw or arduino, of interest
int nParms = -1;
int cmd = -1;
int rnd = -1;
int chksm = -1;
int p1  = -1;
int p2  = -2;
float p1f = -1.0;
float p2f = -1.0;

char version[32];
uint16_t volts = 0;
int16_t amps1 = 0, amps2 = 0;
uint16_t temp = 0;
int32_t speedM1 = 0, speedM2 = 0;
char lastError[MAX_TX_BUF] = {'\0'};
char msg[MAX_TX_BUF] = {'\0'};
long numCmdsRxdFromUSB = 0;
int lastCmdRxdFromUSB = -1;
long numDroppedUsbPackets = 0;
byte currSpeedCommanded = 0;
unsigned long prevMillisLastCommand = millis();

////////////  we need to know if motors were commanded to turn, as a safety feature, so we can shut them down.
bool motorM1Rotating = false;
bool motorM2Rotating = false;


SoftwareSerial serial(rcRxPin, rcTxPin);
RoboClaw roboclaw(&serial, 10000);                  //38400

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  Typical Arduino setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(ARDUINO_USB_HIGHEST_BAUD);
  while (!Serial);
  Serial.println(F("{\"msg\":\"Arduino Roboclaw Driver Is Up\"}"));
  roboclaw.begin(ARDUINO_SERIAL_TO_ROBOCLAW_BAUD);  //38400
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  // safety and / or status related functions
  getRoboclawStatus();
  stopMotorsIfBeenTooLongSinceLastCommand();
  respondWithStatusIfBeenTooLongSinceLastTime();

  recvIncomingUsbSerialWithEndMarker();
  parseIncomingUsbSerial();
  commandHandler();

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Start of Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void commandHandler() {

  if (!newCommandIsReady) return;

  prevMillisLastCommand = millis();

  if (cmd != STATUS) {
    lastCmdRxdFromUSB = -1; lastCmdRxdFromUSB = cmd;
    numCmdsRxdFromUSB++;
  }


  switch (cmd) {

    //////////////////// Arduino user - instructions /////////////////////////
    //case MENU: //help - send list of available commands
      //showMenu();
      //break;
    case ACKCMDS:
      ackCommands = true;
      break;
    case CLRAllERRORS:
      userHasInitiatedArduino = true;
      thereIsUsbError = false;
      thereIsRoboclawError = false;
      ackCommands = false;
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

    //////////////////// Roboclaw instructions /////////////////////////
    case VERSION: //version
      readRcVersion();
      break;
    case STATUS: //read volts, amps, temp
      readStatus();
      break;
    case STOP: //stop both motors
      stopMotors();
      break;
    case FORWARD:
      if (!userHasInitiatedArduino || thereIsUsbError || thereIsRoboclawError) { break; }
      moveForward();
      break;
    case BACKWARD:
      if (!userHasInitiatedArduino || thereIsUsbError || thereIsRoboclawError) { break; }
      moveBackward();
      break;
    case LEFT:
      if (!userHasInitiatedArduino || thereIsUsbError || thereIsRoboclawError) { break; }
      rotateLeft();
      break;
    case RIGHT:
      if (!userHasInitiatedArduino || thereIsUsbError || thereIsRoboclawError) { break; }
      rotateRight();
      break;

    default:
      memset(lastError,0x00,MAX_TX_BUF);
      sprintf(lastError,"\"UNKCMD [%d]\"", lastCmdRxdFromUSB);
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
    memset(msg,0x00,MAX_TX_BUF);
    sprintf(msg,"{\"version\":\"%s\"}", version);
    Serial.println(msg);
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRVER\"");
    thereIsRoboclawError = true;
  }
}



void readRcVoltage() {
  rcValid = false;
  volts = roboclaw.ReadMainBatteryVoltage(address, &rcValid);
  if (rcValid) {
  } else {
    volts = -1;
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRVOLTS\"");
    thereIsRoboclawError = true;
  }
}


void readRcCurrents() {
  if (roboclaw.ReadCurrents(address, amps1, amps2)) {
  } else {
    amps1 = -1;
    amps2 = -1;
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRAMPS\"");
    thereIsRoboclawError = true;
  }
}


void readRcTemperature() {
  if (roboclaw.ReadTemp(address, temp)) {
  } else {
    temp = -1;
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRTEMP\"");
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

  memset(mainTxRxBuffer,0x00, MAX_USB_TX_RX_BUF);

  strncpy(mainTxRxBuffer,bJSON,strlen(bJSON));

  strncat(mainTxRxBuffer, v, strlen(v));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(volts,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, a1, strlen(a1));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(amps1,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, a2, strlen(a2));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(amps2,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, t, strlen(t));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(temp,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, s1, strlen(s1));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(speedM1,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, s2, strlen(s2));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(speedM2,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, c, strlen(c));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(numCmdsRxdFromUSB,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, d, strlen(d));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(numDroppedUsbPackets,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));
  strncat(mainTxRxBuffer,sep,strlen(sep));    

  strncat(mainTxRxBuffer, l, strlen(l));
  memset(tempHoldingBuf,0x00,MAX_RX_PARM_BUF);
  itoa(lastCmdRxdFromUSB,tempHoldingBuf,10);
  strncat(mainTxRxBuffer,tempHoldingBuf,strlen(tempHoldingBuf));

  int lenOfError = strlen(lastError);
  int lenOfTxRxBuf = strlen(mainTxRxBuffer);

  if (lenOfError > 0) {
    strncat(mainTxRxBuffer,sep,strlen(sep));    
    strncat(mainTxRxBuffer,e,strlen(e));    
    strncat(mainTxRxBuffer,lastError,strlen(lastError));
  }
/*
  

  int lenOfError = strlen(lastError);
  int lenOfTxRxBuf = strlen(mainTxRxBuffer);

  if (lenOfError > 0) {
    strncat(mainTxRxBuffer,sep,strlen(sep));    
    strncat(mainTxRxBuffer,e,strlen(e));    
    strncat(mainTxRxBuffer,lastError,strlen(lastError));
    strncat(mainTxRxBuffer,quote,strlen(quote));    
  }
  */
  strncat(mainTxRxBuffer,eJSON,strlen(eJSON));

  Serial.println(mainTxRxBuffer);

}

void readRcMotorSpeeds() {
  rcStatus = 0; rcValid = false;
  speedM1 = roboclaw.ReadSpeedM1(address, &rcStatus, &rcValid);
  if (!rcValid) {
    speedM1 = -1;
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRSPDM1\"");
    thereIsRoboclawError = true;
  }
  rcStatus = 0; rcValid = false;
  speedM2 = roboclaw.ReadSpeedM2(address, &rcStatus, &rcValid);
  if (!rcValid) {
    speedM2 = -1;
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRSPDM2\"");
    thereIsRoboclawError = true;
  }
}

void moveForward() {
  rotateLeftSideForward(p1);
  rotateRightSideForward(p1);
}

void moveBackward() {
  rotateLeftSideBackward(p1);
  rotateRightSideBackward(p1);
}

void rotateLeft() {
  rotateLeftSideBackward(p1);
  rotateRightSideForward(p1);
}

void rotateRight() {
  rotateLeftSideForward(p1);
  rotateRightSideBackward(p1);
}

void rotateLeftSideForward(int speed) {
  if (roboclaw.ForwardM1(address, speed)) {
    motorM1Rotating = true;
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRROTM1F\"");
    thereIsRoboclawError = true;
  }
}

void rotateRightSideForward(int speed) {
  if (roboclaw.ForwardM2(address, speed)) {
    motorM2Rotating = true;
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRROTM2F\"");
    thereIsRoboclawError = true;
  }
}

void stopMotors() {
  uint8_t speed = 0;
  currSpeedCommanded = 0;
  if (roboclaw.ForwardM1(address, speed)) {
    motorM1Rotating = false;
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRSTOPM1\"");
    thereIsRoboclawError = true;
  }
  if (roboclaw.ForwardM2(address, speed)) {
    motorM2Rotating = false;
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRSTOPM2\"");
    thereIsRoboclawError = true;
  }
}

void rotateLeftSideBackward(int speed) {
  if (roboclaw.BackwardM1(address, speed)) {
    motorM1Rotating = true;
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRROTM1B\"");
    thereIsRoboclawError = true;
  }
}

void rotateRightSideBackward(int speed) {
  if (roboclaw.BackwardM2(address, speed)) {
    motorM2Rotating = true;
  } else {
    memset(lastError,0x00,MAX_TX_BUF);
    sprintf(lastError,"\"ERRROTM2B\"");
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
      mainTxRxBuffer[ndx] = rc;
      ndx++;
      if (ndx >= MAX_USB_TX_RX_BUF) {
        ndx = MAX_USB_TX_RX_BUF - 1;
      }
    }
    else {
      mainTxRxBuffer[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }

}


void parseIncomingUsbSerial() {

  if (newData != true) return;

  memset(numParms,0x00,MAX_RX_PARM_BUF);
  memset(command,0x00,MAX_RX_PARM_BUF);
  memset(randNum,0x00,MAX_RX_PARM_BUF);
  memset(chksum,0x00,MAX_RX_PARM_BUF);
  memset(param1,0x00,MAX_RX_PARM_BUF);
  memset(param2,0x00,MAX_RX_PARM_BUF);

  //Serial.println("..parsing..");


  byte ndx = 0;
  byte pidx = 0;
  byte sidx = 0;

  /////   extract numParms out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    numParms[ndx] = mainTxRxBuffer[ndx];
    ndx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract  command out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && ndx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    command[pidx] = mainTxRxBuffer[ndx];
    pidx++;
    ndx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract randNum out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    randNum[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }


  /////   extract chksum out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    chksum[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }



  /////   extract param1 out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    param1[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  /////   need to get past whitespace (except newline)
  sidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && (mainTxRxBuffer[ndx] == ' ' || mainTxRxBuffer[ndx] == '\t')) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    ndx++;
  }



  /////   extract param2 out of the receive buffer
  pidx = 0;
  while (ndx < MAX_USB_TX_RX_BUF && pidx < MAX_RX_PARM_BUF - 1 && mainTxRxBuffer[ndx] != '\n' && mainTxRxBuffer[ndx] != ' ' && mainTxRxBuffer[ndx] != '\t' && mainTxRxBuffer[ndx] != '\r' && mainTxRxBuffer[ndx] != 0) {
    //Serial.print(ndx); Serial.print(", ["); Serial.print(mainTxRxBuffer[ndx]); Serial.println("]");
    param2[pidx] = mainTxRxBuffer[ndx];
    ndx++;
    pidx++;
  }

  memset(mainTxRxBuffer,0x00,MAX_USB_TX_RX_BUF);

  if (verifyChecksum()) {
    newCommandIsReady = true;
    //Serial.println(F("command verified"));
    thereIsUsbError = false;
    memset(lastError,0x00,MAX_TX_BUF);

    
  } else {
    numDroppedUsbPackets++;
    //Serial.println(F("command NOT verified"));
    newData = false; // whatever was rxd from USB was bad, so we're just ignoring it and are now ready to receive more from USB
  }


}

bool verifyRequiredIncomingStringToken(char* bufName, char* buf) {
  if (strlen(buf) < 1) {

    if (ackCommands) {
      memset(lastError,0x00,MAX_TX_BUF);
      sprintf(lastError,"\"%s is empty.\"",bufName);
      readStatus();
    }
    return false;
  }
  return true;
}

bool verifyChecksum() {

  int numberOfUsbParameters = 0;

  if (!verifyRequiredIncomingStringToken("numParms",numParms)) { return false; }

  numberOfUsbParameters++;

  if (!verifyRequiredIncomingStringToken("command",command)) { return false; }

  numberOfUsbParameters++;

  if (!verifyRequiredIncomingStringToken("randNum",randNum)) { return false; }

  numberOfUsbParameters++;

  if (!verifyRequiredIncomingStringToken("chksum",chksum)) { return false; }

  numberOfUsbParameters++;

  if (strlen(param1) > 0) numberOfUsbParameters++;
  if (strlen(param2) > 0) numberOfUsbParameters++;

  nParms = atoi(numParms);

  if (numberOfUsbParameters != nParms) {
    if (ackCommands) {
      memset(lastError,0x00,MAX_TX_BUF);
      sprintf(lastError,"\"nParms [%d] NOT correct.\"",nParms);
      readStatus();
    }
    return false;
  }

  cmd = atoi(command);
  rnd = atoi(randNum);
  chksm = atoi(chksum);
  p1 = 0;
  p1f = 0;
  p2 = 0;
  p2f = 0;
  if (isDecimalValue(param1)) {
    p1f =atof(param1);
  } else {
    p1 = atoi(param1);
  }
  if (isDecimalValue(param2)) {
    p2f =atof(param2);
  } else {
    p2 = atoi(param2);
  }
  
  
  long int mySum = nParms + cmd + rnd + (int)(p1f!=0?p1f*1000:p1) + (int)(p2f!=0?p2f*1000:p2);

//  Serial.print(" nParms:");Serial.print(nParms);
//  Serial.print(" cmd:");Serial.print(cmd);
//  Serial.print(" rnd:");Serial.print(rnd);
//  Serial.print(" p1:");Serial.print((int)(p1f!=0?p1f*1000:p1));
//  Serial.print(" p2:");Serial.print((int)(p2f!=0?p2f*1000:p2));
//  Serial.print(" mySum:");Serial.print(mySum);
//  Serial.print(" chksm:");Serial.println(chksm);

 
  if (mySum == chksm) {
    if (ackCommands) {
      memset(msg,0x00,MAX_TX_BUF);
      sprintf(msg,"{\"msg\":\"Arduino Rxd command [%d].\"}", cmd);
      Serial.println(msg);
    }
    return true;
  } else {
    if (ackCommands) {
      memset(lastError,0x00,MAX_TX_BUF);
      sprintf(lastError,"\"Bad chksum\"");
      readStatus();
    }
    return false;
  }
}

bool isDecimalValue(char* paramBuffer) {
  byte len = strlen(paramBuffer);
  for (byte i=0; i<len; i++) {
    if (paramBuffer[i] == '.') {
      return true;
    }
  }
  return false;
}


void stopMotorsIfBeenTooLongSinceLastCommand() {
  if (motorM1Rotating || motorM2Rotating) {
    if (millis() - prevMillisLastCommand > TIMEOUT_TO_STOP_MOTORS_LAST_HOST_CMD_BEEN_LONG_TIME) {
      stopMotors();
    }
  }
}

void respondWithStatusIfBeenTooLongSinceLastTime() {
  if (continueToAutoSendStatusToHost && ((millis() - prevMillisLastTimeAutoSentStatus) > autoSendStatusTimeout)) {
    prevMillisLastTimeAutoSentStatus = millis();
    readStatus();
  }
}
