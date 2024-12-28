#pragma once

#include "Particle.h"

// Communicates with a PN532 via UART.
class PN532 {
 public:
  // Constructs a new PN532 controller.
  //
  // Args:
  //   serial_interface: The interface on which the PN532 is connected.
  //   reset_pin: P2 pin connected to P70_IRQ's RSTPD_N pin.
  //   irq_pin: P2 pin connected to PN532's P70_IRQ pin.
  PN532(USARTSerial* serial_interface, uint8_t reset_pin, uint8_t irq_pin);

  // Initializes the PN532 controller.
  //
  // Initializes the P2 hardware configuration (pinmodes, serial interface),
  // resets the PN532 and configures it for Initiator / PCD mode.
  void Begin();

 private:
  USARTSerial* serial_interface_;
  int8_t irq_pin_;
  int8_t reset_pin_;

  void Reset();
};
