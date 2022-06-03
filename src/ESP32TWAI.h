// olys ESP32S3 TWAI Implementation
//

#ifndef ESP32_TWAI_H
#define ESP32_TWAI_H

#include "CANController.h"
#include "CAN.h"

// default timeout for a response in milliseconds
#define TWAI_DEFAULT_TIMEOUT CAN_DEFAULT_TIMEOUT

class ESP32TWAIClass : public CANControllerClass {

public:
  ESP32TWAIClass();
  virtual ~ESP32TWAIClass();

  virtual int begin(long baudRate);
  virtual void end();

  virtual int endPacket();

  virtual int parsePacket();
  virtual int peekPacket(uint32_t identifier, uint8_t *CANdata, uint32_t timeout_ms);

  virtual int udsRead(uint32_t txECU, uint16_t txID, uint32_t rxECU, uint8_t* rxData, uint8_t rxLen);
  virtual int canCMD(uint32_t txECU, uint8_t* txData);

  virtual void onReceive(void(*callback)(int));

  using CANControllerClass::filter;
  virtual int filter(int id, int mask);
  using CANControllerClass::filterExtended;
  virtual int filterExtended(long id, long mask);

  virtual int sleep();
  virtual int wakeup();

  virtual int clearRXqueue();
  virtual int clearTXqueue();

  virtual int qIgnition();
  virtual int qVoltage();

private:
  void reset();

};

extern ESP32TWAIClass CAN;

#endif
