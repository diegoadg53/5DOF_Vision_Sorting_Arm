#include <Bluepad32.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "PS3DataPacket.h"
#include "VisionPacket.h"
#include "DFRobot_HuskylensV2.h"
#include "DiagnosticQueue.h"
#include "TagVision.h"

#define LED_PIN 2

// Define which ESP32 pins connect to the camera
#define HL_RX 16   // ESP32 RX ← HuskyLens TX (blue wire)
#define HL_TX 17   // ESP32 TX → HuskyLens RX (green wire)
#define MAX_TRACKED_OBJECTS 10
// Diagnostic only: the name-suffix ID conversion below remains unchanged.
#define ENABLE_CLASSIFICATION_DIAGNOSTIC 1
constexpr uint32_t CLASSIFICATION_DIAGNOSTIC_MS = 500;
unsigned long lastClassificationLogMs = 0;
uint8_t nextClassificationLogBox = 0;
DiagnosticQueue visionDiagnostics;
struct CameraPollDiagnostics {
  uint32_t polls = 0, objectFrames = 0, emptyFrames = 0, errors = 0, lastPollMs = 0;
  int lastResult = -127; // No camera poll yet.
};
CameraPollDiagnostics cameraPollDiagnostics; // Access under mux across the two tasks.
uint32_t visionDataTx = 0, visionDataTxFailures = 0, lastVisionStatusMs = 0;

// --- Thread-Safe Shared State ---
SemaphoreHandle_t dataMutex;
QueueHandle_t commandQueue;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//Camera variables
HuskylensV2 huskylens;

volatile bool cameraIsOnline = false;
volatile bool switchSuccess = false;
volatile eAlgorithm_t currentAlgo = ALGORITHM_ANY;
bool switchAlgo = false;
uint8_t switchAlgoID = 0;

unsigned long lastSwitchTime = 0;

// Bounding Box Data
struct boundingBox {
  int16_t xCenter;
  int16_t yCenter;
  int16_t width;
  int16_t height;
  uint8_t ID;
  char name[24];
  char content[16];
};

struct CameraData {
  boundingBox boxes[MAX_TRACKED_OBJECTS];
  uint8_t count;
  bool isFresh;
};
CameraData latestDetect;

//Other variables
bool ledState = false;
unsigned long lastBlinkTime = 0;

// WIFI variables
const char* ssid     = "Red_Portenta";
const char* password = "robotica_uvg";
IPAddress unicastIP(192, 168, 4, 1);
bool WiFi_initialized = false;
bool WiFi_connected = false;

// UDP Control variables
WiFiUDP udp;
const int udpPort = 4210;          // Gamepad telemetry + heartbeat
PS3ControlPacket txPacket;
uint32_t packetCounter = 0;
unsigned long lastSentUDP = 0;
const unsigned long SENDING_DELAY_MS = 20;
bool UDP_initialized = false;

WiFiUDP cameraUDP;
const int cameraUDPport = 4220;

volatile bool cameraOnlineNotified = false;
volatile bool streamingEnabled = false;
bool packetMissed = false;
bool communicationError = false;

uint8_t lastMsgID = 0;

uint8_t objectsDetected = 0;

criticalMessage pendingESPmsg;
uint8_t globalMsgCounter = 1;
const unsigned long RETRY_TIMEOUT_MS = 100;
const uint8_t MAX_TRIES = 5;
bool messageAck = false;

unsigned long nextResyncTry = 0;
const unsigned long RESYNC_TIME_DELAY_MS = 3000;
uint8_t commFails = 0;
const uint8_t MAX_COMM_FAILS = 3;

// Bluepad32 Control variables
ControllerPtr PS3Control = nullptr;
bool BT_connected = false;

// Camera Task
void cameraManagerTask(void *pvParameters) {
  int errorCount = 0;
  const int MAX_ERRORS = 3;
  for (;;) {
    // STATE 1: INITIALIZATION & FAILSAFE RECOVERY
    if (!cameraIsOnline) {
      Serial.println("[CamTask] Attempting connection...");
      while (Serial2.available()) {
        Serial2.read();
      }
      if (huskylens.begin(Serial2)) {
        Serial.println("[CamTask] Camera Online!");
        portENTER_CRITICAL(&mux);
        cameraIsOnline = true;
        portEXIT_CRITICAL(&mux);
        errorCount = 0;
        
        // Re-apply the desired algorithm if recovering from a crash
        if (currentAlgo != ALGORITHM_ANY) {
          while (Serial2.available()) {
            Serial2.read();
          }
          huskylens.retry = 1;
          huskylens.switchAlgorithm(currentAlgo);
          huskylens.retry = 5;
          vTaskDelay(pdMS_TO_TICKS(5000)); 
        }
      } else {
        // Wait a second before retrying to prevent spamming
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue; 
      }
    }
    // STATE 2: CHECK FOR INCOMING COMMANDS (Switch Algorithm)
    eAlgorithm_t requestedAlgo;
    // Check queue without blocking (0 ticks)
    if (xQueueReceive(commandQueue, &requestedAlgo, 0) == pdPASS) {
      Serial.println("[CamTask] Switching Algorithm...");

      bool success = false;
      int attempts = 0;

      while (!success && attempts < 5) {
        attempts++;
        Serial.printf("[CamTask] Switch attempt %d/5\n", attempts);
        while (Serial2.available()) {
          Serial2.read();
        }
        huskylens.retry = 1;
        success = huskylens.switchAlgorithm(requestedAlgo);
        huskylens.retry = 5;

        if (!success) {
          success = true;
          vTaskDelay(pdMS_TO_TICKS(8000)); // Mandatory 5-sec warmup
        }
        

        // while (Serial2.available()) {
        //   Serial2.read();
        // }
        // eAlgorithm_t currentCamAlgo = requestedAlgo;

        // int8_t testRead = huskylens.ProtocolV2::getResult(currentCamAlgo);
        // if (testRead >= 0 && currentCamAlgo == requestedAlgo) {
        //   success = true;
        //   Serial.println("[CamTask] Algorithm visually and digitally verified!");
        // } else {
        //   Serial.printf("[CamTask] Verification failed. Camera is on %d. Retrying...\n", currentCamAlgo);
        // }
      }

      if (success) {
        currentAlgo = requestedAlgo;
        
        portENTER_CRITICAL(&mux);
        switchSuccess = true;       // Raise the success flag for the main FSM
        portEXIT_CRITICAL(&mux);

        errorCount = 0;
      } else {
        portENTER_CRITICAL(&mux);
        cameraIsOnline = false;
        switchSuccess = false;
        portEXIT_CRITICAL(&mux);
      }
      
      
      
      continue; // Skip the rest of the loop to process the next state
    }
    // STATE 3: POLL DATA
    if (streamingEnabled) {
      int8_t totalDetected = huskylens.getResult(currentAlgo);
      portENTER_CRITICAL(&mux);
      ++cameraPollDiagnostics.polls;
      cameraPollDiagnostics.lastPollMs = millis();
      cameraPollDiagnostics.lastResult = totalDetected;
      if (totalDetected > 0) ++cameraPollDiagnostics.objectFrames;
      else if (totalDetected == 0) ++cameraPollDiagnostics.emptyFrames;
      else ++cameraPollDiagnostics.errors;
      portEXIT_CRITICAL(&mux);

      while (Serial2.available()) {
        Serial2.read();
      }
      
      if (totalDetected >= 0) {
        errorCount = 0; // Reset error counter on successful read
        
        { // Publish empty frames too: sparse tag detection is not a link fault.
          if (xSemaphoreTake(dataMutex, portMAX_DELAY)) {
            latestDetect.count = 0; // Reset count for this frame
            
            // Drain all available results into our array
            while (huskylens.available(currentAlgo) && latestDetect.count < MAX_TRACKED_OBJECTS) {
              Result *r = huskylens.popCachedResult(currentAlgo);
              
              latestDetect.boxes[latestDetect.count].xCenter = r->xCenter;
              latestDetect.boxes[latestDetect.count].yCenter = r->yCenter;
              latestDetect.boxes[latestDetect.count].width   = r->width;
              latestDetect.boxes[latestDetect.count].height  = r->height;
              latestDetect.boxes[latestDetect.count].ID      = r->ID;
              strncpy(latestDetect.boxes[latestDetect.count].name,    r->name.c_str(),    23); 
              latestDetect.boxes[latestDetect.count].name[23] = '\0';
              strncpy(latestDetect.boxes[latestDetect.count].content, r->content.c_str(), 15);
              latestDetect.boxes[latestDetect.count].content[15] = '\0';
              
              latestDetect.count++;
            }
            latestDetect.isFresh = true;
            xSemaphoreGive(dataMutex);
          }
        }
        
        // Safety drain (just in case the library cache had > 10 items)
        while(huskylens.available(currentAlgo)) { 
          huskylens.popCachedResult(currentAlgo); 
        }
      } else {
        // Failsafe: getResult returned -1 (Timeout / No Response)
        errorCount++;
        Serial.printf("[CamTask] Read error %d/%d\n", errorCount, MAX_ERRORS);
        
        if (errorCount >= MAX_ERRORS) {
          // Camera crashed or wire unplugged. Drop offline to trigger recovery.
          portENTER_CRITICAL(&mux);
          cameraIsOnline = false;
          switchSuccess = false;
          portEXIT_CRITICAL(&mux);
        }
      }
    }

    // UBaseType_t stackLeft = uxTaskGetStackHighWaterMark(NULL);
    // Serial.printf("[CamTask] Stack free: %u words\n", stackLeft);

    // Yield to Watchdog
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

//UDP Functions
uint8_t computeChecksum(const uint8_t* data, size_t length) {
  uint8_t xorVal = 0;
  for (size_t i = 0; i < length; i++) {
    xorVal ^= data[i];
  }
  return xorVal;
}

void initializeUDP(){
  udp.begin(udpPort);
  cameraUDP.begin(cameraUDPport);
  Serial.println("[RED] Sockets UDP inicializados");
}

void shutUDP(){
  udp.stop();
  cameraUDP.stop();
  Serial.println("[RED] Sockets UDP apagados");
}


void sendControlPayloadUDP() {
  txPacket.seq = ++packetCounter;

  txPacket.lx = (int8_t)map(PS3Control->axisX(), -511, 512, -128, 127);
  txPacket.ly = (int8_t)map(PS3Control->axisY(), -511, 512, -128, 127);
  txPacket.rx = (int8_t)map(PS3Control->axisRX(), -511, 512, -128, 127);
  txPacket.ry = (int8_t)map(PS3Control->axisRY(), -511, 512, -128, 127);

  txPacket.buttons = (uint16_t)PS3Control->buttons();

  txPacket.checksum = computeChecksum((uint8_t*)&txPacket, sizeof(PS3ControlPacket)-1);

  udp.beginPacket(unicastIP, udpPort);
  udp.write((uint8_t*)&txPacket, sizeof(PS3ControlPacket));
  udp.endPacket();
}

void sendVisionPacket(const VisionPacket &pkt) {
  const bool began = cameraUDP.beginPacket(unicastIP, cameraUDPport) != 0;
  const size_t written = cameraUDP.write((const uint8_t*)&pkt, sizeof(VisionPacket));
  const bool ended = cameraUDP.endPacket() != 0;
  if (pkt.type == DATA_OBJECTS) {
    if (began && written == sizeof(VisionPacket) && ended) ++visionDataTx;
    else ++visionDataTxFailures;
  }
}

void sendCriticalMessage(typeMsg type, uint8_t param = 0) {
  pendingESPmsg.pkt.type = type;
  pendingESPmsg.pkt.msgID = globalMsgCounter++;
  if (globalMsgCounter == 0) globalMsgCounter = 1;

  if (type == CMD_SWITCH_ALGO) {
    pendingESPmsg.pkt.config.ALGO_ID = param;
  }

  pendingESPmsg.pending = true;
  pendingESPmsg.triesLeft = MAX_TRIES;
  pendingESPmsg.lastPing = millis();

  sendVisionPacket(pendingESPmsg.pkt);
}

void managePingsVision(){
  if (!pendingESPmsg.pending) return;

  if (millis() - pendingESPmsg.lastPing >= RETRY_TIMEOUT_MS) {
    if (pendingESPmsg.triesLeft > 0) {
      pendingESPmsg.triesLeft--;
      pendingESPmsg.lastPing = millis();
      sendVisionPacket(pendingESPmsg.pkt);
    } else {
      pendingESPmsg.pending = false;
      commFails++;

      if (commFails > MAX_COMM_FAILS) {
        commFails = 0;
        communicationError = true;
        return;
      }

      cameraUDP.stop();
      cameraUDP.begin(cameraUDPport);

      packetMissed = true;

      nextResyncTry = millis() + RESYNC_TIME_DELAY_MS;
    }
  }
}

void manageVisionPort() {
  int packetSize = cameraUDP.parsePacket();
  if (packetSize == sizeof(VisionPacket)) {
    VisionPacket rxPkt;
    cameraUDP.read((uint8_t*) &rxPkt, sizeof(VisionPacket));
    switch (rxPkt.type) {
      case CMD_SWITCH_ALGO: {
        VisionPacket ack;
        ack.type = ACK_OK;
        ack.msgID = rxPkt.msgID;
        ack.ack.typeAck = CMD_SWITCH_ALGO;

        sendVisionPacket(ack);

        // if (rxPkt.msgID != lastMsgID) {
        //   lastMsgID = rxPkt.msgID;
        //   switchAlgo = true;
        //   switchAlgoID = rxPkt.config.ALGO_ID;
        // }

        switchAlgo = true;
        switchAlgoID = rxPkt.config.ALGO_ID;
        break;
      }
      case CMD_START_STREAM: {
        VisionPacket ack;
        ack.type = ACK_OK;
        ack.msgID = rxPkt.msgID;
        ack.ack.typeAck = CMD_START_STREAM;

        sendVisionPacket(ack);
        streamingEnabled = true;
        if (ENABLE_CLASSIFICATION_DIAGNOSTIC)
          visionDiagnostics.push("[VISION ESP] START_STREAM received; polling enabled\n");
        break;
      }
      case CMD_END_STREAM: {
        VisionPacket ack;
        ack.type = ACK_OK;
        ack.msgID = rxPkt.msgID;
        ack.ack.typeAck = CMD_END_STREAM;

        sendVisionPacket(ack);
        streamingEnabled = false;
        if (ENABLE_CLASSIFICATION_DIAGNOSTIC)
          visionDiagnostics.push("[VISION ESP] END_STREAM received; polling disabled\n");
        break;
      }
      case ACK_OK: {
        if (pendingESPmsg.pending && rxPkt.msgID == pendingESPmsg.pkt.msgID) {
          pendingESPmsg.pending = false;
          messageAck = true;
        }
        break;
      }
      default:
        break;
    }
  }
}

// Bluepad32 functions
void onConnectedController(ControllerPtr ctl) {
  if (PS3Control == nullptr){
    Serial.println("Control principal conectado");
    PS3Control = ctl;
    BT_connected = true;
  } else {
    Serial.println("Ya hay un control activo");
    ctl->disconnect();
  }
}

void onDisconnectedController(ControllerPtr ctl){
  if (PS3Control == ctl) {
    Serial.println("Control desconectado.");
    PS3Control = nullptr;
    BT_connected = false;
  }
}

//WiFi functions
void initializeWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid, password);
  Serial.println("[RED] Wi-Fi inicializado");
}

void disconnectWiFi() {
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println("[RED] Wi-Fi apagado");
}

//FSM definition
enum State {
  WAITING_BT_CONNECTION,
  WAITING_WIFI_CONNECTION,
  NO_CAMERA,
  NORMAL_COMMUNICATION,
  WAITING_CAMERA_ONLINE_ACK,
  WAITING_CAMERA_OFFLINE_ACK,
  WAITING_SWITCH_ALGO,
  WAITING_NO_ALGO
};

State currentState = WAITING_BT_CONNECTION;
State lastState = NORMAL_COMMUNICATION;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Begin Serial2 with the baud you configured on the camera
  Serial2.begin(115200, SERIAL_8N1, HL_RX, HL_TX);

  // Initialize RTOS primitives
  dataMutex = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(1, sizeof(eAlgorithm_t)); // Queue holds 1 command

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Launch the Manager Task on Core 0
  xTaskCreatePinnedToCore(cameraManagerTask, "CameraManager", 4096, NULL, 1, NULL, 0);
}

void loop() {
  // Build a snapshot while holding dataMutex; print only after releasing it.
  char classificationDiagnostic[144] = {0};
  // put your main code here, to run repeatedly:
  BP32.update();

  managePingsVision();

  manageVisionPort();

  portENTER_CRITICAL(&mux);
  bool localSwitch = switchSuccess;
  bool localOnline = cameraIsOnline;
  portEXIT_CRITICAL(&mux);

  switch (currentState) {
    // ===================================
    // WAITING_BT_CONNECTION STATE DEF
    // ===================================
    case WAITING_BT_CONNECTION:
      //State initialization code:
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: WAITING BT CONNECTION");
        lastState = currentState;
      }
      //State running code:
      if (millis() - lastBlinkTime >= 500) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        lastBlinkTime = millis();
      }
      //State exit conditions:
      if (BT_connected) {
        currentState = WAITING_WIFI_CONNECTION;
      }
      break;
    // ===================================
    // WAITING_WIFI_CONNECTION STATE DEF
    // ===================================
    case WAITING_WIFI_CONNECTION:
      //State initialization code:
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: WAITING WIFI CONNECTION");
        lastState = currentState;
      }
      //State running code:
      if (!WiFi_initialized){
        initializeWiFi();
        WiFi_initialized = true;
      }

      if (millis() - lastBlinkTime >= 100) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW);
        lastBlinkTime = millis();
      }

      WiFi_connected = WiFi.status() == WL_CONNECTED;

      //State exit conditions:
      if (!BT_connected) {
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (WiFi_connected) {
        initializeUDP();
        digitalWrite(LED_PIN, HIGH);
        ledState = true;
        if (localOnline) {
          // Camera may have initialized before Wi-Fi. Portenta still needs
          // the acknowledged ONLINE event before it can enable camera modes.
          currentState = WAITING_CAMERA_ONLINE_ACK;
        } else {
          currentState = WAITING_CAMERA_OFFLINE_ACK;
        }
      }
      break;
    // ===================================
    // WAITING_CAMERA_OFFLINE_ACK STATE DEF
    // ===================================
    case WAITING_CAMERA_OFFLINE_ACK: {
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: WAITING CAMERA OFFLINE ACK");
        sendCriticalMessage(EVT_CAM_OFFLINE);
        lastState = currentState;
      }

      if (packetMissed && (millis() >= nextResyncTry)) {
        packetMissed = false;
        sendCriticalMessage(EVT_CAM_OFFLINE);
      }

      if (millis() - lastSentUDP >= SENDING_DELAY_MS) {
        lastSentUDP = millis();
        sendControlPayloadUDP();
      }

      if (!BT_connected) {
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (messageAck) {
        messageAck = false;
        currentState = NO_CAMERA;
      } else if (communicationError) {
        communicationError = false;
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      } else if (WiFi.status() != WL_CONNECTED) {
        shutUDP();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      }
      break;
    }
    // ===================================
    // NO_CAMERA STATE DEF
    // ===================================
    case NO_CAMERA: {
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: NO CAMERA");
        lastState = currentState;
      }

      if (millis() - lastSentUDP >= SENDING_DELAY_MS) {
        lastSentUDP = millis();
        sendControlPayloadUDP();
      }

      if (!BT_connected) {
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (localOnline) {
        currentState = WAITING_CAMERA_ONLINE_ACK;
      } else if (WiFi.status() != WL_CONNECTED) {
        shutUDP();
        WiFi_initialized = false;
        WiFi_connected = false;
        currentState = WAITING_WIFI_CONNECTION;
      }
      break;
    }
    // ===================================
    // WAITING_CAMERA_ONLINE_ACK STATE DEF
    // ===================================
    case WAITING_CAMERA_ONLINE_ACK: {
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: WAITING CAMERA ONLINE ACK");
        sendCriticalMessage(EVT_CAM_ONLINE);
        lastState = currentState;
      }

      if (packetMissed && (millis() >= nextResyncTry)) {
        packetMissed = false;
        sendCriticalMessage(EVT_CAM_ONLINE);
      }

      if (millis() - lastSentUDP >= SENDING_DELAY_MS) {
        lastSentUDP = millis();
        sendControlPayloadUDP();
      }

      if (!BT_connected) {
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (messageAck) {
        messageAck = false;
        currentState = NORMAL_COMMUNICATION;
      } else if (communicationError) {
        communicationError = false;
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      } else if (WiFi.status() != WL_CONNECTED) {
        shutUDP();
        WiFi_initialized = false;
        WiFi_connected = false;
        
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      }
      break;
    }
    // ===================================
    // NORMAL_COMMUNICATION STATE DEF
    // ===================================
    case NORMAL_COMMUNICATION:
      //State initialization code:
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: NORMAL COMMUNICATION");
        lastState = currentState;
      }
      //State running code:
      if (millis() - lastSentUDP >= SENDING_DELAY_MS) {
        lastSentUDP = millis();
        sendControlPayloadUDP();
      }

      if (streamingEnabled) {
        if (xSemaphoreTake(dataMutex, 0)) {
          if (latestDetect.isFresh) {
            latestDetect.isFresh = false;

            objectVision bufferCameraObjects[10];
            
            if (currentAlgo == ALGORITHM_TAG_RECOGNITION) {
              VisionPacket dataPkt{};
              dataPkt.type = DATA_OBJECTS;
              packAprilTags(latestDetect, dataPkt);
              sendVisionPacket(dataPkt);
            } else { // CUSTOME MODELS, ID is in name at last position "Pieza_02" -> ID:2
              const bool logClassification = ENABLE_CLASSIFICATION_DIAGNOSTIC &&
                  millis() - lastClassificationLogMs >= CLASSIFICATION_DIAGNOSTIC_MS;
              const uint8_t logBox = latestDetect.count ?
                  nextClassificationLogBox % latestDetect.count : 0;
              for (int i = 0; i < latestDetect.count; i++){
                const char* name = latestDetect.boxes[i].name;
                const char* underscore = strrchr(name, '_');
                int id = (underscore != nullptr) ? atoi(underscore + 1) : 0;
                bufferCameraObjects[i].xCenter = latestDetect.boxes[i].xCenter;
                bufferCameraObjects[i].yCenter = latestDetect.boxes[i].yCenter;
                bufferCameraObjects[i].width = latestDetect.boxes[i].width;
                bufferCameraObjects[i].height = latestDetect.boxes[i].height;
                bufferCameraObjects[i].ID = (uint8_t)id;
                if (logClassification && i == logBox) {
                  snprintf(classificationDiagnostic, sizeof(classificationDiagnostic),
                           "[CLASS TX] model=%u box=%u/%u cameraID=%u name=\"%s\" sentID=%u\n",
                           unsigned(currentAlgo), unsigned(i + 1), unsigned(latestDetect.count),
                           unsigned(latestDetect.boxes[i].ID), name,
                           unsigned(bufferCameraObjects[i].ID));
                  lastClassificationLogMs = millis();
                  ++nextClassificationLogBox; // Rotate samples if several boxes are present.
                }
              }
              VisionPacket dataPkt;
              dataPkt.type = DATA_OBJECTS;
              dataPkt.vision.amountoObjects = latestDetect.count;
              memcpy(dataPkt.vision.objList, bufferCameraObjects, latestDetect.count*sizeof(objectVision));
              sendVisionPacket(dataPkt);
            }
          }
          xSemaphoreGive(dataMutex);
        }
      }

      // Preserve a sampled line across loops even if the UART is currently busy.
      if (classificationDiagnostic[0])
        visionDiagnostics.push(classificationDiagnostic);

      //State exit conditions:
      if (!BT_connected) {
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (WiFi.status() != WL_CONNECTED) {
        shutUDP();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      } else if (switchAlgo) {
        switchAlgo = false;
        if ((switchAlgoID == 16)||(switchAlgoID >= 128)) {
          currentState = WAITING_SWITCH_ALGO;
        } else {
          currentState = WAITING_NO_ALGO;
        }
      } else if (!localOnline) {
        currentState = WAITING_CAMERA_OFFLINE_ACK;
      }
      break;
    // ===================================
    // WAITING_SWITCH_ALGO STATE DEF
    // ===================================
    case WAITING_SWITCH_ALGO: {
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: WAITING SWITCH ALGO");
        eAlgorithm_t target = (eAlgorithm_t)switchAlgoID;
        portENTER_CRITICAL(&mux);
        switchSuccess = false;
        portEXIT_CRITICAL(&mux);
        xQueueSend(commandQueue, &target, 0);
        lastState = currentState;
      }

      if (millis() - lastSentUDP >= SENDING_DELAY_MS) {
        lastSentUDP = millis();
        sendControlPayloadUDP();
      }

      if (localSwitch) {
        portENTER_CRITICAL(&mux);
        switchSuccess = false;
        portEXIT_CRITICAL(&mux);
        Serial.println("[SYSTEM] Algorithm switched succesfully");
        sendCriticalMessage(EVT_SWITCHED_ALGO);
      }

      if (packetMissed && (millis() >= nextResyncTry)) {
        packetMissed = false;
        sendCriticalMessage(EVT_SWITCHED_ALGO);
      }

      if (!BT_connected) {
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (WiFi.status() != WL_CONNECTED) {
        shutUDP();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      } else if (!localOnline) {
        currentState = WAITING_CAMERA_OFFLINE_ACK;
      } else if (messageAck) {
        messageAck = false;
        switchAlgo = false;
        Serial.println("[SYSTEM] Portenta acknowledge succesful switch");
        currentState = NORMAL_COMMUNICATION;
      } else if (communicationError) {
        communicationError = false;
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      }
      break;
    }
    // ===================================
    // WAITING_NO_ALGO STATE DEF
    // ===================================
    case WAITING_NO_ALGO: {
      if (currentState != lastState) {
        Serial.println("[SYSTEM] State changed to: WAITING NO ALGO");
        sendCriticalMessage(EVT_NO_ALGO);
        lastState = currentState;
      }

      if (millis() - lastSentUDP >= SENDING_DELAY_MS) {
        lastSentUDP = millis();
        sendControlPayloadUDP();
      }

      if (packetMissed && (millis() >= nextResyncTry)) {
        packetMissed = false;
        sendCriticalMessage(EVT_NO_ALGO);
      }

      if (!BT_connected) {
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_BT_CONNECTION;
      } else if (WiFi.status() != WL_CONNECTED) {
        shutUDP();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      } else if (!localOnline) {
        currentState = WAITING_CAMERA_OFFLINE_ACK;
      } else if (messageAck) {
        messageAck = false;
        Serial.println("[SYSTEM] Portenta acknowledge succesful switch");
        currentState = NORMAL_COMMUNICATION;
      } else if (communicationError) {
        communicationError = false;
        shutUDP();
        disconnectWiFi();
        WiFi_initialized = false;
        WiFi_connected = false;
        pendingESPmsg.pending = false;
        currentState = WAITING_WIFI_CONNECTION;
      }
      break;
    }
  }

  if (ENABLE_CLASSIFICATION_DIAGNOSTIC && WiFi_connected &&
      millis() - lastVisionStatusMs >= 2000) {
    lastVisionStatusMs = millis();
    CameraPollDiagnostics snapshot;
    portENTER_CRITICAL(&mux);
    snapshot = cameraPollDiagnostics;
    portEXIT_CRITICAL(&mux);
    char status[320];
    snprintf(status, sizeof(status),
             "[VISION ESP] stream=%u model=%u state=%u online=%u polls=%lu "
             "objectFrames=%lu empty=%lu errors=%lu lastResult=%d pollAge=%lums "
             "dataTx=%lu txFailures=%lu logDrops=%lu\n",
             unsigned(streamingEnabled), unsigned(currentAlgo), unsigned(currentState),
             unsigned(localOnline), (unsigned long)snapshot.polls,
             (unsigned long)snapshot.objectFrames, (unsigned long)snapshot.emptyFrames,
             (unsigned long)snapshot.errors, snapshot.lastResult,
             (unsigned long)(snapshot.polls ? millis() - snapshot.lastPollMs : 0),
             (unsigned long)visionDataTx, (unsigned long)visionDataTxFailures,
             (unsigned long)visionDiagnostics.dropped());
    visionDiagnostics.push(status);
  }
  visionDiagnostics.drain(Serial);
}
