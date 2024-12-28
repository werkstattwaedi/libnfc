
#include "PN532.h"

PN532::PN532(USARTSerial* serialInterface, uint8_t resetPin, uint8_t irqPin)
    : serial_interface_(serialInterface),
      irq_pin_(irqPin),
      reset_pin_(resetPin) {}

void PN532::Begin() {
  pinMode(reset_pin_, OUTPUT);
  digitalWrite(reset_pin_, HIGH);

  serial_interface_->begin(115200);
}
