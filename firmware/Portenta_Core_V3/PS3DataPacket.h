#pragma once
#include <Arduino.h>

// #pragma pack(push, 1): Le prohíbe al compilador meter "bytes de relleno" (padding).
// Garantiza que la estructura mida exactamente lo mismo en memoria en ambos ESP32.
#pragma pack(push, 1)

struct PS3ControlPacket {
  uint32_t seq;         // Contador incremental: si llega un paquete con seq menor al actual, es viejo y se descarta.
  
  // Joysticks analógicos (-128 a 127). Un int8_t ocupa 1 byte exacto.
  int8_t lx;            // Stick izquierdo X
  int8_t ly;            // Stick izquierdo Y
  int8_t rx;            // Stick derecho X
  int8_t ry;            // Stick derecho Y

  // Botones digitales empacados en bits (un bit por botón para ahorrar espacio):
  // bit 0: Cruz, bit 1: Círculo, bit 2: Triángulo, bit 3: Cuadrado, bit 4: L1, bit 5: R1, etc.
  uint16_t buttons;     

  uint8_t checksum;     // Suma de verificación simple (XOR) para descartar tramas corruptas por ruido de RF.
};

#pragma pack(pop)       // Restaura el empaquetado normal del compilador.