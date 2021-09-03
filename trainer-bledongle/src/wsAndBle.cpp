#include <Arduino.h>

// wifi stuff
#include <WiFi.h>
#include <WiFiMulti.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h> // https://github.com/Links2004/arduinoWebSockets (WebSockets by Markus Sattler)

#include <ArduinoJson.h> // parsing incoming commands
StaticJsonBuffer<256> jsonBuffer; // parsing incoming json docs;

WiFiMulti wiFiMulti;
WebSocketsServer webSocket = WebSocketsServer(81);
// TCP server at port 80 will respond to HTTP requests
WiFiServer server(80);

const char OTAName[] = "esp32dongleOTA";     // A name and a password for the OTA service
const char OTAPassword[] = "esp8266";
const char mdnsName[] = "esp32dongle";

#define WS_SEND_INTERVAL 1000
uint8_t wsClientNum; // we sturen maar data naar 1 client; de laatste die connecteerde
bool wsConnected = false;
uint32_t wsMillis;


// BLE stuff
#include "BLEDevice.h"
//#include "BLEScan.h"

// BLE server we wish to connect to
BLEAddress DiretoAddress("e5:66:56:7a:0c:9b");
BLEClient *bleClient;

#define BLE_NOT_CONNECTED   0
#define BLE_SCANNING        1
#define BLE_SCANNED         2
#define BLE_CONNECTED       3

uint8_t bleStatus = BLE_NOT_CONNECTED;

BLERemoteService *pSvcCyclingSpeedCadence; // UUID 0x1816
BLERemoteService *pSvcCyclingPower; // UUID 0x1818
BLERemoteService *pSvcFitnessMachine; // UUID 0x1826

BLERemoteCharacteristic *pCharCyclingSpeedCadenceMeasurement; // UUID 0x2A5B
BLERemoteCharacteristic *pCharCyclingPowerMeasurement; // UUID 0x2A63
BLERemoteCharacteristic *pCharIndoorBikeData; // UUID 0x2AD2
BLERemoteCharacteristic *pCharTrainingStatus; // UUID 0x2AD3
BLERemoteCharacteristic *pCharFitnessMachineStatus; // UUID 0x2ADA
BLERemoteCharacteristic *pCharFitnessMachineControlPoint; // UUID 0x2AD9

uint32_t bleCumulativeWheelRevsCSC; // zitten ook in CyclingPower characteristic
uint16_t bleLastWheelEventTimeCSC; // zitten ook in CyclingPower characteristic
uint16_t bleCumulativeCrankRevsCSC;// zitten ook in CyclingPower characteristic
uint16_t bleLastCrankEventTimeCSC; // zitten ook in CyclingPower characteristic
uint16_t bleInstantaneousPower; 
uint8_t blePedalPowerBalance;
uint32_t bleCumulativeWheelRevs;
uint16_t bleLastWheelEventTime;
uint16_t bleCumulativeCrankRevs;
uint16_t  bleLastCrankEventTime;
float     bleInstantaneousSpeed; // 16-bit field in indoor bike data / 100
uint16_t bleInstantaneousCadence;
uint32_t bleCumulativeDistance;
uint16_t bleInstantaneousPowerIBD; 
uint16_t bleElapsedTime;
uint16_t bleResistanceLevelIBD; // reports back the setting by FTMS Control Point

#define FTMS_CMD_REQUEST_CONTROL              0x00
#define FTMS_CMD_RESET                        0x01
//#define FTMS_CMD_SET_TARGET_SPEED             0x02
//#define FTMS_CMD_SET_TARGET_INCLINATION       0x03
#define FTMS_CMD_SET_TARGET_RESISTANCE_LEVEL  0x04
#define FTMS_CMD_SET_TARGET_POWER             0x05
#define FTMS_CMD_START_OR_RESUME              0x07
#define FTMS_CMD_STOP_OR_PAUSE                0x08
#define FTMS_CMD_SET_SIMULATION_PARAMETERS    0x11
#define FTMS_CMD_SET_WHEEL_CIRCUMFERENCE      0x12

#define FTMS_CMD_RESPONSE                     0x80
#define FTMS_RESULT_SUCCESS                   0x01
#define FTMS_RESULT_NOT_SUPPORTED             0x02
#define FTMS_RESULT_INVALID_PARAMETER         0x03
#define FTMS_RESULT_OPERATION_FAILED          0x04
#define FTMS_RESULT_CONTROL_NOT_PERMITTED     0x05

// status of the FTMS state machine

#define FTMSSTATUS_INIT_REQUEST_CONTROL 0
#define FTMSSTATUS_INIT_RESET           1
#define FTMSSTATUS_REQUEST_CONTROL      2
#define FTMSSTATUS_DO_RESET             3
#define FTMSSTATUS_READY                4
#define FTMSSTATUS_COMMAND_BUSY         5

// status for individual FTMS commands
#define CMDLOOPSTATUS_IDLE      0
#define CMDLOOPSTATUS_WAITREPLY 2
#define CMDLOOPSTATUS_GOTREPLY  3

uint8_t cmdLoopStatus  = CMDLOOPSTATUS_IDLE;
uint8_t ftmsStatus = FTMSSTATUS_INIT_REQUEST_CONTROL;

uint8_t ftmsCmd[20];
uint8_t ftmsReply[20]; // max reply size according the spec
uint8_t ftmsCmdLen, ftmsReplyLen;

void printHexData(uint8_t *data, int len)
{
  for (int i=0;i<len;i++) {
    Serial.print(data[i],HEX);
    Serial.print(" -");
  }
  Serial.println();
} // printHexData

static void callbackCyclingSpeedAndCadenceMeasurement (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  uint8_t flags;
  uint8_t idx;
  
  Serial.print("Notify callback for CSC Measurement : ");
  Serial.print(pChar->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  printHexData(pData,length);
  // parse the pData according to the spec
  flags = pData[0];
  idx = 1;
  if (flags & 0x1) {
    bleCumulativeWheelRevsCSC =  *(uint32_t*)(pData+idx);
    idx += 4;
    bleLastWheelEventTimeCSC =  *(uint16_t*)(pData+idx);
    idx += 2;
  }
  if (flags & 0x2) {
    bleCumulativeCrankRevsCSC =  *(uint32_t*)(pData+idx);
    idx += 2;
    bleLastCrankEventTimeCSC =  *(uint16_t*)(pData+idx);
    idx += 2;
  }
  Serial.printf("CSC - WheelRevs = %d, WhEvt = %d, CrankRevs = %d, CrEvt = %d\n",
              bleCumulativeWheelRevsCSC, bleLastWheelEventTimeCSC,
              bleCumulativeCrankRevsCSC,bleLastCrankEventTimeCSC);
    
} // callbackCyclingSpeedAndCadenceMeasurement


static void callbackCyclingPowerMeasurement (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  uint16_t flags;
  uint8_t idx;
  
  Serial.print("Notify callback for Cycling Power Measurement : ");
  Serial.print(pChar->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  printHexData(pData,length);
  // parse the pData according to the spec
  flags = *(uint16_t*)(pData);
  idx = 2;
  
  bleInstantaneousPower =  *(uint16_t*)(pData+idx);
  idx += 2;
  
  if (flags & 0x1) {
    blePedalPowerBalance = pData[idx] >> 1; // percentage with resolution 1/2
    idx += 1;
  }  
  if (flags & 0x2) {
    // accumulated torque - skip
    idx += 2;
  }  
  if (flags & 0x10) {
    // wheel rev data
    bleCumulativeWheelRevs =  *(uint32_t*)(pData+idx);
    idx+= 4;
    bleLastWheelEventTime =  *(uint16_t*)(pData+idx);
    idx += 2;
  }  
  if (flags & 0x20) {
    // crank rev data
    bleCumulativeCrankRevs =  *(uint16_t*)(pData+idx);
    idx+= 2;
    bleLastCrankEventTime =  *(uint16_t*)(pData+idx);
    idx += 2;
  }  
  
  Serial.printf("CP - Power = %d, Balance = %d, WheelRevs = %d, WhEvt = %d, CrankRevs = %d, CrEvt = %d\n",
                                 bleInstantaneousPower, blePedalPowerBalance, bleCumulativeWheelRevs,
                                 bleLastWheelEventTime,bleCumulativeCrankRevs,bleLastCrankEventTime);
    
} // callbackCyclingPowerMeasurement

static void callbackIndoorBikeData (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  uint16_t flags;
  uint8_t idx;

  Serial.print("Notify callback for Indoor Bike Data : ");
  Serial.print(pChar->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  printHexData(pData,length);
  // parse the pData according to the spec
  flags = *(uint16_t*)(pData);
  idx = 2;
  if ((flags & 0x1) == 0) {
    // C1 contains instantaneous speed
    // 16-bit field is km/h with 0.01 resolution
    bleInstantaneousSpeed =  ((float)(*(uint16_t*)(pData+idx))) / 100.0;
    idx += 2;
  }
  if (flags & 0x2){
    // C2 average speed - skip
    idx += 2;
  }
  if (flags & 0x4) {
    // C3 instantaneous cadence
    // 16-bit field is cadence with 0.5 resolution
    bleInstantaneousCadence =  (*(uint16_t*)(pData+idx)) >> 1;
    idx += 2;
  }
  if (flags & 0x8) {
    // C4 average cadence -- skip
    idx += 2;
  }
  if (flags & 0x10) {
    // C5 total distance - 3 bytes
    bleCumulativeDistance =  (*(uint32_t*)(pData+idx))&0xFFFFFF; // 24-bits field
    idx += 3;
  }
  if (flags & 0x20) {
    // C6 resistance level
    bleResistanceLevelIBD = (*(uint16_t*)(pData+idx));
    idx += 2;
  }
  if (flags & 0x40) {
    // C7 instantaneous power
    bleInstantaneousPowerIBD = (*(uint16_t*)(pData+idx));
    idx += 2;
  }
  if (flags & 0x80) {
    // C8 average power - skip
    idx += 2;
  }
  if (flags & 0x100) {
    // C9 expended energy - skip
    idx += 5;
  }
  if (flags & 0x200) {
    // C10 hear rate - skip
    idx += 1;
  }
  if (flags & 0x400) {
    // C11 metabolic equivalent - skip
    idx += 1;
  }
  if (flags & 0x800) {
    // C12 elapsed time
    bleElapsedTime =  *(uint16_t*)(pData+idx);
  }

  Serial.printf("IBD - Speed = %f, Cadence = %d, Distance  = %d, Resistance  = %d, Power = %d, ElapsedTime = %d\n",
                                 bleInstantaneousSpeed, bleInstantaneousCadence, bleCumulativeDistance, 
                                 bleResistanceLevelIBD, bleInstantaneousPowerIBD,bleElapsedTime);
    
} // callbackIndoorBikeData

static void callbackFitnessMachineStatus (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Notify callback for FTMS Status : ");
  Serial.print(pChar->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  printHexData(pData,length);
  // parse the pData according to the spec
    
} // callbackFitnessMachineStatus

static void callbackTrainingStatus (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Notify callback for Training Status : ");
  Serial.print(pChar->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  printHexData(pData,length);
  // parse the pData according to the spec
    
} // callbackTrainingStatus

static void callbackFitnessMachineControlPoint (BLERemoteCharacteristic* pChar, uint8_t* pData, size_t length, bool isNotify) {
  Serial.print("Notify callback for FTMS CP - ");
  Serial.print(pChar->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  printHexData(pData,length);
  
  if (cmdLoopStatus == CMDLOOPSTATUS_WAITREPLY) {
    // misschien de reply matchen met het command?
    //pData[1] == ftmsCmd[0] ?
    memcpy((void*)ftmsReply,(void*)pData,length);
    ftmsReplyLen = length;
    cmdLoopStatus = CMDLOOPSTATUS_GOTREPLY;
    printHexData(ftmsReply,ftmsReplyLen);
  }
} // callbackFitnessMachineControlPoint

void ftms_SetResistance (uint8_t targetResistance) {
  if (ftmsStatus == FTMSSTATUS_READY) {
    ftmsCmd[0] = FTMS_CMD_SET_TARGET_RESISTANCE_LEVEL;
    ftmsCmd[1] = targetResistance;
    ftmsCmdLen = 2;
  }
  
} // ftms_SetResistance

void ftms_Start () {
  if (ftmsStatus == FTMSSTATUS_READY) {
    ftmsCmd[0] = FTMS_CMD_START_OR_RESUME;
    ftmsCmdLen = 1;
  }
} // ftms_Start

void ftms_Stop () {
  if (ftmsStatus == FTMSSTATUS_READY) {
    ftmsCmd[0] = FTMS_CMD_STOP_OR_PAUSE;
    ftmsCmd[1] = 0x1; // STOP
    ftmsCmdLen = 2;
  }
} // ftms_Stop


void ftms_Pause () {
  if (ftmsStatus == FTMSSTATUS_READY) {
    ftmsCmd[0] = FTMS_CMD_STOP_OR_PAUSE;
    ftmsCmd[1] = 0x2; // PAUSE
    ftmsCmdLen = 2;
  }
} // ftms_Stop

void ftms_Reset () {
  cmdLoopStatus = CMDLOOPSTATUS_IDLE;
  ftmsStatus = FTMSSTATUS_DO_RESET;
} // ftms_Reset

void ftms_SetWheelCircumference (uint16_t wheelCircumference) {
  uint16_t *pParam;
  if (ftmsStatus == FTMSSTATUS_READY) {
    ftmsCmd[0] = FTMS_CMD_SET_WHEEL_CIRCUMFERENCE;
    pParam = (uint16_t*) &ftmsCmd[1];
    *pParam = wheelCircumference;
    ftmsCmdLen = 3;
  }
} // ftms_SetWheelCircumference

uint16_t bikeSimWindSpeed = 0;
int16_t bikeSimGradient = 0;
uint8_t bikeSimCrr = 0;
uint8_t bikeSimCw = 0;

void ftms_SetBikeSimParameters (uint16_t windSpeed, int16_t gradient, uint8_t crr, uint8_t cw) {
  uint16_t *pParam;
  if (ftmsStatus == FTMSSTATUS_READY) {
    ftmsCmd[0] = FTMS_CMD_SET_SIMULATION_PARAMETERS;
    pParam = (uint16_t*) &ftmsCmd[1];
    *pParam = windSpeed;
    pParam = (uint16_t*) &ftmsCmd[3];
    *pParam = gradient;
    ftmsCmd[5] = crr;
    ftmsCmd[6] = cw;
    ftmsCmdLen = 7;
  }
} // ftms_SetWheelCircumference

void ftmsLoop() {
  bool sendCommand = false;
  
  if (cmdLoopStatus == CMDLOOPSTATUS_IDLE) {
    switch(ftmsStatus) {
      case FTMSSTATUS_INIT_REQUEST_CONTROL :
      case FTMSSTATUS_REQUEST_CONTROL :
        // send request control
        ftmsCmd[0] = FTMS_CMD_REQUEST_CONTROL;
        ftmsCmdLen = 1;
        sendCommand = true;
        break;
      case FTMSSTATUS_INIT_RESET : 
      case FTMSSTATUS_DO_RESET : 
        // send reset :
        ftmsCmd[0] = FTMS_CMD_RESET;
        ftmsCmdLen = 1;
        sendCommand = true;      
        break;
      case FTMSSTATUS_READY :
        // we are ready to accept commands from the interface
        if (ftmsCmdLen) {
          sendCommand = true;
          ftmsStatus = FTMSSTATUS_COMMAND_BUSY;
        }
    } 
  }
  
  else if (cmdLoopStatus == CMDLOOPSTATUS_GOTREPLY) {
    
    // check reply code
    // reply has form 0x80 - request OpCode - result code - response parameters
    if (ftmsReply[1] == ftmsCmd[0]) {
      switch (ftmsReply[2]) {
        case FTMS_RESULT_SUCCESS :
          cmdLoopStatus = CMDLOOPSTATUS_IDLE;
          // and move ftmsStatus to the next step REQUEST_CONTROL ->DO_RESET -> READY
          if (ftmsStatus == FTMSSTATUS_INIT_REQUEST_CONTROL) ftmsStatus = FTMSSTATUS_INIT_RESET;
          else if (ftmsStatus == FTMSSTATUS_INIT_RESET) ftmsStatus = FTMSSTATUS_REQUEST_CONTROL;
          else if (ftmsStatus == FTMSSTATUS_REQUEST_CONTROL) ftmsStatus = FTMSSTATUS_READY;
          else if (ftmsStatus == FTMSSTATUS_DO_RESET) ftmsStatus = FTMSSTATUS_REQUEST_CONTROL;
          else if (ftmsStatus == FTMSSTATUS_COMMAND_BUSY) {
            ftmsStatus = FTMSSTATUS_READY;
            // and send a notify to the caller with the reply/results?
          } 
          ftmsCmdLen = 0;
          break;
        case FTMS_RESULT_NOT_SUPPORTED :
          Serial.println("FTMS command is not supported!");
          cmdLoopStatus = CMDLOOPSTATUS_IDLE;
          break;
        case FTMS_RESULT_INVALID_PARAMETER :
          Serial.println("FTMS wrong format! CHECK CODE!");
          cmdLoopStatus = CMDLOOPSTATUS_IDLE;
          break;
        case FTMS_RESULT_OPERATION_FAILED :
          Serial.println("FTMS operation failed -> retrying");
          // check hoe dit moet, want ftmsCmdLen is hier 0, en dan kan je hetzelfde command niet meer overdoen
          // voorlopig ftmStatus = READY zodat we manueel hetzelfde commando kunnen sturen
          // beter zou zijn ftmsStatus hier niet te wijzigen, maar dat lukt voorlopig niet met deze implementatie van ftmsCmdLen
          ftmsStatus = FTMSSTATUS_READY;
          cmdLoopStatus = CMDLOOPSTATUS_IDLE;
          break;
        case FTMS_RESULT_CONTROL_NOT_PERMITTED :
          Serial.println("FTMS control not permitted -> requesting control");
          ftmsStatus = FTMSSTATUS_REQUEST_CONTROL;
          cmdLoopStatus = CMDLOOPSTATUS_IDLE;
          break;
      }
    }
    else {
      // we received a reply for another command
      // resend or wait longer ?
      Serial.printf("received wrong opCode : %d for request %d",ftmsReply[1],ftmsCmd[0]);
      // cmdLoopStatus = CMDLOOPSTATUS_IDLE;
    }
  }
  
  if (sendCommand){
    Serial.printf("sending ftms command, status = %d\n",ftmsStatus);
    printHexData(ftmsCmd,ftmsCmdLen);
    cmdLoopStatus = CMDLOOPSTATUS_WAITREPLY;
    pCharFitnessMachineControlPoint->writeValue(ftmsCmd,ftmsCmdLen,true); // true = need response -> response comes in the callback
    sendCommand = false;
  }

} // ftmsLoop


/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    if (advertisedDevice.getAddress().equals(DiretoAddress)) {
      Serial.println("found Direto");
      advertisedDevice.getScan()->stop();
      bleStatus = BLE_SCANNED;
    }

  } // onResult
}; // MyAdvertisedDeviceCallbacks

class MyBLEClientCallbacks : public BLEClientCallbacks {
public:
 ~MyBLEClientCallbacks() {};
  void onConnect(BLEClient *pClient){
    Serial.println("BLE client connected!");
  }
  void onDisconnect(BLEClient *pClient) {
    Serial.println("BLE client disconnected!!!");
    // TODO : de connectie restoren!
    
  }
}; // MyBLEClientCallbacks

/*
json format : 
{ event : "data", "newConnection", "lostConnection", "cmd"
  data : {
    power : powerValue,
    cadence : cadenceValue,
    pedalBalance : balanceValue,
    elapsedTime : elapsedTimeValue
  }
  newConnection : {
    bleAddress :  "xx.xx.xx.xx.xx.xx",
    bleName : "DI"
  }
  lostConnection : {
    bleAddress :  "xx.xx.xx.xx.xx.xx",
    bleName : "DI"
  }
  cmd : {
    "RESET" : 1,
    "START" : 1,
    "STOP" : 1,
    "RESISTANCE" : 200
  }
}

*/
char wsDataBuffer[200];

void buildBikeDataJSON() {
  snprintf(wsDataBuffer, 200, 
           "{\"event\": \"data\"," 
             "\"data\" : {"
               "\"pow\": %d,"
               "\"cad\": %d,"
               "\"bal\": %d,"
               "\"elTim\": %d}}",bleInstantaneousPower,bleInstantaneousCadence,blePedalPowerBalance,bleElapsedTime);
  
} // buildBikeDataJSON

void parseWsCommand (uint8_t *data, size_t length ) {
  // parse the payload
  jsonBuffer.clear(); // start new parse operation
  JsonObject& root = jsonBuffer.parseObject(data);
  if (!root.success()) {
    Serial.printf("json parsing error on incoming command\n");
    // parsing failed
    return;
  }

  const char *eventType = root["event"];
  if (eventType) {
    if (0 == strcmp(eventType, "cmd")) {
      // command from client
      if (root["cmd"]["START"]) {
        Serial.println("cmd from ws : start");
        ftms_Start();
      }
      else if (root["cmd"]["STOP"]) {
        Serial.println("cmd from ws : stop");
        ftms_Stop();
      }
      else if (root["cmd"]["RESET"]) {
        Serial.println("cmd from ws : reset");
        ftms_Reset();
      }
      // we hebben de .success() hier nodig omdat resistance = 0 een geldige waarde is
      // https://arduinojson.org/api/jsonobject/containskey/
      else if ((root["cmd"]["RESISTANCE"]).success()) {
        uint8_t targetResistance = (uint8_t) root["cmd"]["RESISTANCE"];
        Serial.printf("cmd from ws : resistance to %d\n",targetResistance);
        ftms_SetResistance (targetResistance);
      }
      else {
        Serial.println("cmd from ws : no valid cmd in JSON document");
      }
    }
    else {
      Serial.printf("ws : Unknown event type received : %s\n",eventType);
    }
  }
  else {
    Serial.println("ws: json contains no event key");
  }
} // parseWsCommand

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {

  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      
      // maybe we should keep track of all connected clients
      // and only reset the flag when all clients are disconnected
      // be we assume only 1 will connect anyway
      wsConnected = false; 
      
      break;
    case WStype_CONNECTED:
    {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      wsClientNum = num;
      wsConnected = true;
      // should we send an ack message here? the main loop will send ble data updates from now on anyway
    }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      
      // parse commands from client here!!
      if (num != wsClientNum) {
        Serial.println("INFO! I get data from an unknown client");
      }
      else {
        Serial.println("parsing command");
        parseWsCommand (payload,length);
      }

      break;
    case WStype_BIN:
      Serial.println("receive binary data -- OOPS");
      break;
    default :
      break;
  }

} // webSocketEvent

void setup_wifi() {
  bool retval;
  
  if (WiFi.getMode() != WIFI_STA)
  {
      WiFi.mode(WIFI_STA); // we want this permanently in flash
  }
  WiFi.begin("blabla","bloblo",0,NULL,false); // this is needed for setHostname after this to work correctly. haha, arduino!

  retval = WiFi.setHostname(mdnsName); // --> http://mdnsName/ (als de router mee wil) met mdns ook : http://mdnsName.local/ (na startMDNS)
  if (!retval) {
    Serial.println("setHostname failed");
  }
  Serial.printf("hostname = %s\n",WiFi.getHostname());

  // turns out this does nothing for the moment :  
  WiFi.persistent(false);  
  
  wiFiMulti.addAP("TP-LINK", "7EB9E33C8C");

  Serial.println("Connecting");
  while (wiFiMulti.run() != WL_CONNECTED) {  // Wait for the Wi-Fi to connect
      delay(250);
      Serial.print('.');
  }

  Serial.println("\r\n");
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());             // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.print(WiFi.localIP());            // Send the IP address of the ESP8266 to the computer
  Serial.println("\r\n");

} // setup_wifi

void setup_OTA() { // Start the OTA service
  ArduinoOTA.setHostname(OTAName);
  ArduinoOTA.setPassword(OTAPassword);
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });  
  
  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\r\nEnd");
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
} // setup_OTA

void setup_wifi2() {
    // Connect to WiFi network
    WiFi.begin("TP-LINK", "7EB9E33C8C");
    Serial.println("");

    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(WiFi.SSID());
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
} // setup_wifi2


void setup_mdns() {
  if (!MDNS.begin(mdnsName)) {
      Serial.println("Error setting up MDNS responder!");
      return;
  }

  // Start TCP (HTTP) server
  server.begin();
  Serial.println("TCP server started");
  
  MDNS.addService("http", "tcp", 80); // deze heeft geen zin want we hebben toch geen webserver hier
  MDNS.addService("ws", "tcp", 81);
  Serial.print("mDNS responder started: http://");
  Serial.print(mdnsName);
  Serial.println(".local");  

} // setup_mdns


void setup_websocket() {
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("webSocket ready!");
} // setup_websocket





void activateNotify(BLERemoteCharacteristic *pChar) {

  uint16_t val_2902;
  BLEUUID uuid2902((uint16_t) 0x2902);
  uuid2902.to128();

  BLERemoteDescriptor* pDescriptor2902 = pChar->getDescriptor(uuid2902);
  if (pDescriptor2902 == nullptr) { 
      Serial.println("ERROR met pRemoteDescriptor");
  }
  else {
    uint8_t data[2] = {0x3,0x0};
    pDescriptor2902->writeValue(data, 2, false);
    val_2902 = pDescriptor2902->readUInt16();
    if (val_2902 != 3)
      Serial.printf("writing val=3 to descriptor 2902 failed: %d",val_2902);
  }    
  // map niet nodig
/*  
  std::map<std::string, BLERemoteDescriptor *> *descriptorsMap;
  descriptorsMap = pChar->getDescriptors();
  for (auto &myPair : *descriptorsMap){
    BLERemoteDescriptor* pRemoteDescriptor = myPair.second;
  }
*/
  
} // activateNotify

void initServicesAndCharacteristics() {
  //02/10/2018
  Serial.println(ESP.getFreeHeap());
  
  // use BLEUUID instances to find the services / characteristics pointers from the maps
  BLEUUID uuidSvcCyclingSpeedCadence((uint16_t)0x1816);
  BLEUUID uuidSvcCyclingPower((uint16_t)0x1818);
  BLEUUID uuidSvcFitnessMachine((uint16_t)0x1826);
  BLEUUID uuidCharCyclingSpeedCadenceMeasurement((uint16_t)0x2A5B); // UUID 0x2A5B
  BLEUUID uuidCharCyclingPowerMeasurement((uint16_t)0x2A63); // UUID 0x2A63
  BLEUUID uuidCharIndoorBikeData((uint16_t)0x2AD2); // UUID 0x2AD2
  BLEUUID uuidCharTrainingStatus((uint16_t)0x2AD3); // UUID 0x2AD3
  BLEUUID uuidCharFitnessMachineStatus((uint16_t)0x2ADA); // UUID 0x2ADA
  BLEUUID uuidCharFitnessMachineControlPoint((uint16_t)0x2AD9); // UUID 0x2AD9
  uuidSvcCyclingSpeedCadence.to128();
  uuidSvcCyclingPower.to128();
  uuidSvcFitnessMachine.to128();
  uuidCharCyclingSpeedCadenceMeasurement.to128();
  uuidCharCyclingPowerMeasurement.to128();
  uuidCharIndoorBikeData.to128();
  uuidCharTrainingStatus.to128();
  uuidCharFitnessMachineStatus.to128();
  uuidCharFitnessMachineControlPoint.to128();

  bleClient = BLEDevice::createClient();
  if (!bleClient)
    return;
  Serial.println(" - Created client");

  // Connect to the remote BLE Server.
  bool retval =  bleClient->connect(DiretoAddress);
  if (!retval)
  {
    Serial.println("failed to connect to direto");
    return;
  }
  Serial.println(" - Connected to server"); // the direto
  // 1. find services we are interested in
  std::map<std::string, BLERemoteService*> *servicesMap;
  BLERemoteService *pSvc;
  servicesMap = bleClient->getServices();
  for (auto &myPair : *servicesMap){
    pSvc = myPair.second;
    if (pSvc->getUUID().equals(uuidSvcCyclingSpeedCadence)) {
      pSvcCyclingSpeedCadence = pSvc;
      Serial.println("found Cycling Speed & Cadence service");
    }
    else if (pSvc->getUUID().equals(uuidSvcCyclingPower)) {
      pSvcCyclingPower = pSvc;
      Serial.println("found Cycling Power service");
    }
    else if (pSvc->getUUID().equals(uuidSvcFitnessMachine)) {
      pSvcFitnessMachine = pSvc;
      Serial.println("found Fitness Machine service");
    }
  }
  
  // 2. find characteristics we are interested in 
  std::map<std::string, BLERemoteCharacteristic *> *characteristicsMap;
  BLERemoteCharacteristic *pChar;

  // Cycling Speed & Cadence
  pSvc = pSvcCyclingSpeedCadence;
  if (pSvc) {
    characteristicsMap = pSvc->getCharacteristics(); // this will call retrieveCharacteristics
    //Serial.printf("listing all characteristics for service %s\n",pSvc->getUUID());
    Serial.println(pSvc->toString().c_str());

    for (auto &myPair : *characteristicsMap){
      pChar = myPair.second;
      if (pChar->getUUID().equals(uuidCharCyclingSpeedCadenceMeasurement)) {
        pCharCyclingSpeedCadenceMeasurement = pChar;
        Serial.println("found Cycling Speed & Cadence Measurement Characteristic");
      }
    }
   }
   
  // Cycling Power
  pSvc = pSvcCyclingPower;
  if (pSvc) {
    characteristicsMap = pSvc->getCharacteristics(); // this will call retrieveCharacteristics
    //Serial.printf("listing all characteristics for service %s\n",pSvc->getUUID());
    Serial.println(pSvc->toString().c_str());

    for (auto &myPair : *characteristicsMap){
      pChar = myPair.second;
      if (pChar->getUUID().equals(uuidCharCyclingPowerMeasurement)) {
        pCharCyclingPowerMeasurement = pChar;
        Serial.println("found Cycling Power Measurement Characteristic");
      }
    }
   }
   
  // Fitness Machine
  pSvc = pSvcFitnessMachine;
  if (pSvc) {
    characteristicsMap = pSvc->getCharacteristics(); // this will call retrieveCharacteristics
    //Serial.printf("listing all characteristics for service %s\n",pSvc->getUUID());
    Serial.println(pSvc->toString().c_str());

    for (auto &myPair : *characteristicsMap){
      pChar = myPair.second;
      if (pChar->getUUID().equals(uuidCharIndoorBikeData)) {
        pCharIndoorBikeData = pChar;
        Serial.println("found Indoor Bike Data Characteristic");
      }
      else if (pChar->getUUID().equals(uuidCharTrainingStatus)) {
        pCharTrainingStatus = pChar;
        Serial.println("found Training Status Characteristic");
      }
      else if (pChar->getUUID().equals(uuidCharFitnessMachineStatus)) {
        pCharFitnessMachineStatus = pChar;
        Serial.println("found Fitness Machine Status Characteristic");
      }
      else if (pChar->getUUID().equals(uuidCharFitnessMachineControlPoint)) {
        pCharFitnessMachineControlPoint = pChar;
        Serial.println("found  Fitness Machine Control Point Characteristic");
      }
    }
   }
   
   // 3. descriptors gebruiken om de notifies te activeren
  
   if (pCharCyclingSpeedCadenceMeasurement) {
     Serial.println("setup ntf for csc");
     activateNotify(pCharCyclingSpeedCadenceMeasurement);
     pCharCyclingSpeedCadenceMeasurement->registerForNotify(callbackCyclingSpeedAndCadenceMeasurement);
   }
   
   if (pCharCyclingPowerMeasurement) {
     Serial.println("setup ntf for cpm");
     activateNotify(pCharCyclingPowerMeasurement);
     pCharCyclingPowerMeasurement->registerForNotify(callbackCyclingPowerMeasurement);
   }
  
   if (pCharIndoorBikeData) {
     Serial.println("setup ntf for ibd");
     activateNotify(pCharIndoorBikeData);
     pCharIndoorBikeData->registerForNotify(callbackIndoorBikeData);
   }
   
   if (pCharTrainingStatus) {
     Serial.println("setup ntf for training status");
     activateNotify(pCharTrainingStatus);
     pCharTrainingStatus->registerForNotify(callbackTrainingStatus);
   }
   if (pCharFitnessMachineStatus) {
     Serial.println("setup ntf for ftms status");
     activateNotify(pCharFitnessMachineStatus);
     pCharFitnessMachineStatus->registerForNotify(callbackFitnessMachineStatus);
   }
      
   if (pCharFitnessMachineControlPoint) {
     Serial.println("setup ntf for ftms cp");
     activateNotify(pCharFitnessMachineControlPoint);
     pCharFitnessMachineControlPoint->registerForNotify(callbackFitnessMachineControlPoint);
   }

   Serial.println("initServicesAndCharacteristics completed!");
     //02/10/2018
  Serial.println(ESP.getFreeHeap());

 
} // initServicesAndCharacteristics


void setup() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  
  // setup wifi
  Serial.println("Setting up wifi...");
  setup_wifi();
  //setup_wifi2();
  setup_OTA(); // this has to happen BEFORE mdns, otherwise mdns doesn't work!! ??
  setup_mdns();
  setup_websocket();
  
  Serial.println("Setting up BLE...");
  BLEDevice::init("");
    //02/10/2018
  Serial.println(ESP.getFreeHeap());

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  bleStatus = BLE_SCANNING;
  
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);
  
} // setup

//for test
void server_loop(void)
{
    // Check if a client has connected
    WiFiClient client = server.available();
    if (!client) {
        return;
    }
    Serial.println("");
    Serial.println("New client");

    // Wait for data from client to become available
    while(client.connected() && !client.available()){
        delay(1);
    }

    // Read the first line of HTTP request
    String req = client.readStringUntil('\r');

    // First line of HTTP request looks like "GET /path HTTP/1.1"
    // Retrieve the "/path" part by finding the spaces
    int addr_start = req.indexOf(' ');
    int addr_end = req.indexOf(' ', addr_start + 1);
    if (addr_start == -1 || addr_end == -1) {
        Serial.print("Invalid request: ");
        Serial.println(req);
        return;
    }
    req = req.substring(addr_start + 1, addr_end);
    Serial.print("Request: ");
    Serial.println(req);
    client.flush();

    String s;
    if (req == "/")
    {
        IPAddress ip = WiFi.localIP();
        String ipStr = String(ip[0]) + '.' + String(ip[1]) + '.' + String(ip[2]) + '.' + String(ip[3]);
        s = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n<!DOCTYPE HTML>\r\n<html>Hello from ESP32 at ";
        s += ipStr;
        s += "</html>\r\n\r\n";
        Serial.println("Sending 200");
    }
    else
    {
        s = "HTTP/1.1 404 Not Found\r\n\r\n";
        Serial.println("Sending 404");
    }
    client.print(s);

    Serial.println("Done with client");
}


void loop() {
  bool retval;
  
  //wifi part of the loop
  server_loop(); // test
  webSocket.loop();
  ArduinoOTA.handle();    // listen for OTA events

  // update data to wifi client
  if ((millis() - wsMillis) >= WS_SEND_INTERVAL) {
    // send an update
    if (wsConnected) {
      // TODO : if BLE_CONNECTED, anders sturen we stale data
      buildBikeDataJSON();
      retval = webSocket.sendTXT(wsClientNum, wsDataBuffer );
      if (!retval) {
        Serial.println("loop: error sending data to ws client!");
      }
    }
    wsMillis = millis();
  }  
  
  // ble part of the loop
  if (bleStatus == BLE_NOT_CONNECTED) {
    // todo; als we ergens detecteren dat de connectie weg is, gaan we proberen opnieuw te connecteren
  }
  else if (bleStatus == BLE_SCANNING) {
  
  }
  else if (bleStatus == BLE_SCANNED) {
    // let's connect & do initial setup
    Serial.print("Connecting to ");
    Serial.println(DiretoAddress.toString().c_str());
    // als de direto pas is opgestart, wordt die onmiddellijk gescanned
    // de services worden correct opgelijst, callback descriptors correct geschreven en teruggelezen
    // maar toch komen de CSC/CP/TrainingStatus/FTMSStatus callbacks niet??
    // het blauwe ledje op de direto blijft dan ook knipperen
    // met een delay hier lukt het wel
    // ofwel eerst de direto opstarten en dan de dongle, dan ook ok
    // bij succesvol binden brandt de blauwe led op de direto vast ->dan ok
    // 
    delay(2000);
    initServicesAndCharacteristics();
    Serial.println("BLE setup done!");
    bleStatus = BLE_CONNECTED;
    
  }
  else if (bleStatus == BLE_CONNECTED) {
    // any repeated actions here;
    ftmsLoop();
  }
  
  // serial interface for testing
  byte b;
  char serbuf[10];
  uint16_t wheelcircumference;
  int targetResistance;

  if(Serial.available())
  {
    b = Serial.read();
    switch(b)
    {
      case 's' :
        ftms_Start();
        break;
      case 'p' :
        ftms_Pause();
        break;
      case 'z' :
        ftms_Stop();
        break;
      case 'x' : 
        bleClient->disconnect();
        break;
      case 'y' :
        initServicesAndCharacteristics();
        break;   
      case 'o' : // wheel circumference
        memset(serbuf, 0, sizeof(serbuf));          
        Serial.readBytesUntil('\n',serbuf, sizeof(serbuf)-1);
        wheelcircumference = atoi(serbuf);
        ftms_SetWheelCircumference(wheelcircumference);
        Serial.printf("setting bikeSimParameter windspeed\n");
        break;
      case 'w' : // bike sim windspeed
        memset(serbuf, 0, sizeof(serbuf));          
        Serial.readBytesUntil('\n',serbuf, sizeof(serbuf)-1);
        bikeSimWindSpeed = atoi(serbuf);
        ftms_SetBikeSimParameters(bikeSimWindSpeed,bikeSimGradient,bikeSimCrr,bikeSimCw);
        Serial.printf("setting bikeSimParameter windspeed\n");
        break;
      case 'g' : // bike sim gradient
        memset(serbuf, 0, sizeof(serbuf));          
        Serial.readBytesUntil('\n',serbuf, sizeof(serbuf)-1);
        bikeSimGradient = atoi(serbuf);
        ftms_SetBikeSimParameters(bikeSimWindSpeed,bikeSimGradient,bikeSimCrr,bikeSimCw);
        Serial.printf("setting bikeSimParameter gradient\n");
        break;
      case 'c' : // crr rolling resistance
        memset(serbuf, 0, sizeof(serbuf));          
        Serial.readBytesUntil('\n',serbuf, sizeof(serbuf)-1);
        bikeSimCrr = atoi(serbuf);
        ftms_SetBikeSimParameters(bikeSimWindSpeed,bikeSimGradient,bikeSimCrr,bikeSimCw);
        Serial.printf("setting bikeSimParameter Crr\n");
        break;
      case 'v' : // cw wind resistance
        memset(serbuf, 0, sizeof(serbuf));          
        Serial.readBytesUntil('\n',serbuf, sizeof(serbuf)-1);
        bikeSimCw = atoi(serbuf);
        ftms_SetBikeSimParameters(bikeSimWindSpeed,bikeSimGradient,bikeSimCrr,bikeSimCw);
        Serial.printf("setting bikeSimParameters Cw\n");
        break;
      case 'r' :
        memset(serbuf, 0, sizeof(serbuf));          
        Serial.readBytesUntil('\n',serbuf, sizeof(serbuf)-1);
        targetResistance = atoi(serbuf);
        ftms_SetResistance(targetResistance);
        Serial.printf("setting targetResistance to %d\n",targetResistance);
        break;
    }
  }  

} // End of loop



