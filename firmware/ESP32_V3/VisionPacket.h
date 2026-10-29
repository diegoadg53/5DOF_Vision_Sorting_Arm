#pragma once
#include <Arduino.h>

#pragma pack(push, 1)

enum typeMsg : uint8_t {
  // Comandos de Portenta -> ESP32
  CMD_SWITCH_ALGO  = 0x01,
  CMD_START_STREAM = 0x02,
  CMD_END_STREAM   = 0x03,

  // Respuestas/Eventos de ESP32 -> Portenta
  EVT_CAM_ONLINE   = 0x10,
  EVT_CAM_OFFLINE  = 0x11,
  EVT_SWITCHED_ALGO= 0x12,
  EVT_NO_ALGO      = 0x13,

  // Confirmación genérica
  ACK_OK           = 0x20,

  // No necesita ack
  DATA_OBJECTS     = 0x30
};

// Elemento individual detectado (4 * uint16_t + 1 * uint8_t = 9 bytes)
struct objectVision {
  uint16_t xCenter;
  uint16_t yCenter;
  uint16_t width;
  uint16_t height;
  uint8_t  ID;
};

struct VisionPacket {
  uint8_t type; // TipoMsg (1 byte)
  uint8_t msgID;

  // Usamos una union: comparten la misma memoria según el tipo de mensaje
  union {
    // Si tipo == CMD_CAMBIAR_ALGORITMO
    struct {
      uint8_t ALGO_ID; // 0 a 254
    } config;

    struct {
      uint8_t typeAck;
    } ack;

    // Si tipo == DATA_OBJETOS
    struct {
      uint8_t amountoObjects;   // 0 a 10
      objectVision objList[10];    // Hasta 10 objetos
    } vision;
  };
};

struct criticalMessage {
  VisionPacket pkt;
  bool pending = false;
  uint8_t triesLeft = 0;
  unsigned long lastPing = 0;
};

#pragma pack(pop)