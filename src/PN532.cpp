
#include "PN532.h"

Logger pn532_log(PN532_LOGTAG);

#define PN532_FRAME_MAX_LENGTH 255
#define PN532_DEFAULT_TIMEOUT 1000

const uint8_t PN532_ACK[] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
const uint8_t PN532_NACK[] = {0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00};
const uint8_t PN532_FRAME_START[] = {PN532_PREAMBLE, PN532_STARTCODE1,
                                     PN532_STARTCODE2};
// PN532, V1.6
const uint8_t PN532_FIRMWARE_RESPONSE[] = {0x32, 0x01, 0x06, 0x07};

PN532::PN532(USARTSerial* serialInterface, uint8_t resetPin, uint8_t irqPin)
    : is_initialized_(false),
      serial_interface_(serialInterface),
      irq_pin_(irqPin),
      reset_pin_(resetPin) {}

Status PN532::Begin() {
  if (is_initialized_) {
    pn532_log.error("PN532::Begin() Already initialized");
    return Status::kError;
  }
  is_initialized_ = true;

  pn532_log.info("PN532::Begin [interface:%d, irq:%d, reset:%d]",
                 serial_interface_->interface(), irq_pin_, reset_pin_);

  os_semaphore_create(&response_available_, 1, 0);

  pinMode(reset_pin_, OUTPUT);
  digitalWrite(reset_pin_, HIGH);

  pinMode(irq_pin_, INPUT);
  attachInterrupt(irq_pin_, &PN532::ResponseAvailableInterruptHandler, this,
                  FALLING);

  serial_interface_->begin(115200);

  // 6.2.2 Dialog structure - timeout is 89ms at 115200 baud
  command_timeout_ms_ = 89;
  serial_interface_->setTimeout(command_timeout_ms_);

  return ResetController();
}

Status PN532::SendCommand(DataFrame* command_data, int retries) {
  Status status = WriteFrame(command_data);
  if (status != Status::kOk) {
    return status;
  }

  if (ReadAckFrame() != Status::kOk) {
    if (retries > 0) {
      pn532_log.warn("PN532::SendCommand failed to receive ACK, retrying...");
      return SendCommand(command_data, retries - 1);
    } else {
      pn532_log.error("PN532::SendCommand failed to receive ACK");
      return Status::kError;
    }
  }

  return Status::kOk;
}

Status PN532::ReceiveResponse(DataFrame* response_data,
                              system_tick_t timeout_ms, int retries) {
  // if (os_semaphore_take(response_available_, timeout_ms, false) != 0) {
  //   return Status::kTimeout;
  // }

  Status status = ReadFrame(response_data);
  if (status != Status::kOk) {
    if (retries > 0) {
      pn532_log.warn(
          "ReceiveResponse did not receive frame, retrying with NACK...");
      serial_interface_->write(PN532_NACK, sizeof(PN532_NACK));
      return ReceiveResponse(response_data, timeout_ms, retries - 1);
    } else {
      pn532_log.error("ReceiveResponse did not receive correct frame.");
      return Status::kError;
    }
  }

  return Status::kOk;
}

Status PN532::CallFunction(DataFrame* command_in_response_out,
                           system_tick_t timeout_ms, int retries) {
  while (serial_interface_->available()) {
    serial_interface_->read();
  }

  Status result = SendCommand(command_in_response_out, retries);
  if (result != Status::kOk) {
    pn532_log.error("CommandDialog Send failed");
    return result;
  }

  result = ReceiveResponse(command_in_response_out, timeout_ms, retries);
  if (result != Status::kOk) {
    pn532_log.error("CommandDialog Receibe failed");
    return result;
  }

  return Status::kOk;
}

Status PN532::ResetController() {
  pn532_log.info("PN532::ResetController");

  digitalWrite(reset_pin_, LOW);
  // 100us should be enough to reset, RSTOUT would indicate that PN532 is
  // actually reset. Since this is not wired, wait for 10ms, that should do the
  // trick.
  delay(50);
  digitalWrite(reset_pin_, HIGH);
  delay(50);

  // 6.3.2.3 Case of PN532 in Power Down mode
  // HSU wake up condition: the real waking up condition is the 5th rising edge
  // on the serial line, hence send first a 0x55 dummy byte and wait for the
  // waking up delay before sending the command frame.
  // serial_interface_->write(PN532_WAKEUP);

  // the host controller has to wait for at least T_osc_start before sending a
  // new command that will be properly understood. T_osc_start is typically a
  // few 100µs, but depending of the quartz, board layout and capacitors, it can
  // be up to 2ms
  // delay(2);

  uint8_t data[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  serial_interface_->write(data, sizeof(data));

  // uint8_t data[] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  //                   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  //                   0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};
  // serial_interface_->write(data, sizeof(data));

  // After reset, SAMConfiguration must be executed as a first command
  // https://files.waveshare.com/upload/b/bb/Pn532um.pdf
  // see p89
  DataFrame sam_configuration{
      .command = PN532_COMMAND_SAMCONFIGURATION,
      .params =
          {
              0x01,  // normal mode
              0x14,  // 0x14, timeout 50ms * 20 = 1 second
              0x01,  // use IRQ pin!
          },
      .params_length = 3};

  Status result = CallFunction(&sam_configuration);
  if (result != Status::kOk) {
    pn532_log.error("ResetController SAMConfiguration failed");
    return result;
  }

  return CheckControllerFirmware();
}

Status PN532::CheckControllerFirmware() {
  DataFrame get_firmware_version{.command = PN532_COMMAND_GETFIRMWAREVERSION,
                                 .params_length = 0};

  Status result = CallFunction(&get_firmware_version);
  if (result != Status::kOk) {
    pn532_log.error("CheckControllerFirmwarefailed");
    return result;
  }

  if (get_firmware_version.params_length != sizeof(PN532_FIRMWARE_RESPONSE) ||
      memcmp(get_firmware_version.params, PN532_FIRMWARE_RESPONSE,
             sizeof(PN532_FIRMWARE_RESPONSE)) != 0) {
    pn532_log.error("Firmware doesn't match!");
    return Status::kError;
  }

  return Status::kOk;
}

Status PN532::WriteFrame(DataFrame* command_data) {
  // packet data length includes the TFI byte, hence + 1
  size_t length = command_data->params_length + 2;
  if (length > PN532_FRAME_MAX_LENGTH) {
    pn532_log.error("command_data packet is too long (%d bytes)", length);
    return Status::kError;
  }

  // See https://files.waveshare.com/upload/b/bb/Pn532um.pdf
  // 6.2 Host controller communication protocol

  // [Byte 0..2] Frame start
  serial_interface_->write(PN532_FRAME_START, sizeof(PN532_FRAME_START));

  // [Byte 3] Command length (includes TFI, hence + 1)
  serial_interface_->write(length);

  // [Byte 4] Command length checksum
  serial_interface_->write(~length + 1);

  // Data starting from here is included in the checksum.
  // [Byte 5] Frame identifier
  serial_interface_->write(PN532_HOSTTOPN532);
  serial_interface_->write(command_data->command);
  // [Bytes 6..n] packet data
  serial_interface_->write(command_data->params, command_data->params_length);

  uint8_t checksum = PN532_HOSTTOPN532 + command_data->command;
  for (uint8_t i = 0; i < command_data->params_length; i++) {
    checksum += command_data->params[i];
  }

  // [Byte n+1] checksum
  serial_interface_->write(~checksum + 1);
  // [Byte n+2] postamble
  serial_interface_->write(PN532_POSTAMBLE);

  serial_interface_->flush();

  if (pn532_log.isTraceEnabled()) {
    pn532_log.trace(
        "WriteFrame(%#04x)[%s]", command_data->command,
        BytesToHexString(command_data->params, command_data->params_length)
            .c_str());
  }

  return Status::kOk;
}

Status PN532::ReadFrame(DataFrame* response_data) {
  // See https://files.waveshare.com/upload/b/bb/Pn532um.pdf
  // 6.2 Host controller communication protocol

  ConsumeFrameStartSequence();

  uint8_t frame_length = serial_interface_->read();
  uint8_t frame_length_checksum = serial_interface_->read();
  // Check length & length checksum match.
  if (((frame_length + frame_length_checksum) & 0xFF) != 0) {
    pn532_log.error("Response length checksum did not match length!");
    return Status::kError;
  }

  // Check TFI byte matches.
  uint8_t frame_identifier = serial_interface_->read();
  if (frame_identifier != PN532_PN532TOHOST) {
    pn532_log.error("TFI byte (%#04x) did not match expected value",
                    frame_identifier);
    return Status::kError;
  }

  // Check response command matches
  uint8_t response_command = serial_interface_->read();
  if (response_command != response_data->command + 1) {
    pn532_log.error(
        "response_command (%#04x) did not match expected command (%#04x)",
        response_command, response_data->command);
    return Status::kError;
  }

  // Read frame with expected length of data. Note:
  // - TFI byte and response_command was already consumed above.
  // - response_data's packetData is big enough to contain the maximal
  // frame_length
  response_data->params_length = frame_length - 2;
  size_t bytes_read = serial_interface_->readBytes(
      (char*)response_data->params, response_data->params_length);
  if (bytes_read != response_data->params_length) {
    pn532_log.error(
        "Response stream tearminated early. Read %d bytes, expected %d",
        bytes_read, response_data->params_length);
    return Status::kError;
  }

  if (pn532_log.isTraceEnabled()) {
    pn532_log.trace(
        "ReadFrame(%#04x)[%s]", response_command,
        BytesToHexString(response_data->params, response_data->params_length)
            .c_str());
  }

  uint8_t checksum = frame_identifier + response_command;
  // Check frame checksum value matches bytes.
  for (uint8_t i = 0; i < response_data->params_length; i++) {
    checksum += response_data->params[i];
  }

  // Add last "Data checksum byte".
  checksum += serial_interface_->read();
  if (checksum != 0) {
    pn532_log.error("Response checksum did not match expected checksum");
    return Status::kError;
  }

  return Status::kOk;
}

Status PN532::ReadAckFrame() {
  // See https://files.waveshare.com/upload/b/bb/Pn532um.pdf
  // 6.2.1.3 ACK frame
  // This is essentially an empty frame, with a data LEN 0

  ConsumeFrameStartSequence();
  uint8_t frame_length = serial_interface_->read();
  if (frame_length != 0) {
    pn532_log.error("ACK frame must be 0 length");
    return Status::kError;
  }

  uint8_t frame_length_checksum = serial_interface_->read();
  if (frame_length_checksum != 0xff) {
    pn532_log.error("ACK frame length checksum invalid");
    return Status::kError;
  }

  return Status::kOk;
}

Status PN532::ConsumeFrameStartSequence() {
  // Skip reading until start sequence 0x00 0xFF is received.
  uint8_t start_1 = serial_interface_->read();
  uint8_t start_2 = serial_interface_->read();

  uint8_t skipped_count = 0;
  while (start_1 != PN532_STARTCODE1 || start_2 != PN532_STARTCODE2) {
    start_1 = start_2;
    start_2 = serial_interface_->read();
    skipped_count++;
    if (skipped_count > PN532_FRAME_MAX_LENGTH) {
      pn532_log.error("Response frame preamble does not contain 0x00FF!");
      return Status::kError;
    }
  }

  return Status::kOk;
}

void PN532::ResponseAvailableInterruptHandler() {
  os_semaphore_give(response_available_, false);
}