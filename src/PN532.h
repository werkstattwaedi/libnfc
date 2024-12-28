#pragma once

#include "Particle.h"

/**
 * @brief main include file for to communicate with a PN532 via UART.
 */
class PN532 {
public:
  /**
   * @brief  Instantiates a new PN532 controller.
   *
   * @param  serialInterface The interface on which the PN532 is connected.
   * @param  resetPin P2 pin connected P70_IRQ's RSTPD_N pin.
   * @param  irqPin P2 pin connected to PN532's P70_IRQ pin.
   */
  PN532(USARTSerial* serialInterface, uint8_t resetPin, uint8_t irqPin);

  setup();

private:
};
