// olys ESP32S3 TWAI Implementation
//

#include "Arduino.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

#include "ESP32TWAI.h"

/* --------------------- Definitions and static variables ------------------ */
#define DEBUG              1
#define LOGGING_TAG        "TWAI"

//RejsaCAN 3.0b
#define BLUE_LED          10  // GPIO10
#define YELLOW_LED        11  // GPIO11
#define FORCE_KEEP_ON     17  // GPIO17   // OUTPUT   SET TO HIGH TO KEEP BOARD RUNNING WHEN POWER VOLTAGE DROPS BELOW 13,3V (WHEN CAR ENGINE ISN'T CHARGING THE CAR BATTERY)

#define SENSE_V_DIG        8  // GPIO08   // INPUT    USE "INPUT_PULLUP". HIGH = CAR ENGINE IS RUNNING AND IS CHARGING CAR BATTERY
#define SENSE_V_ANA        9  // GPIO09   // INPUT    UNCALIBRATED ANALOG REPRESENTATION OF CAR BATTERY VOLTAGE

#define CAN_TX            14  // GPIO14
#define CAN_RX            13  // GPIO13

#define SWITCHED_3V3      21  // GPIO21  (HI_DRIVER)

#define SD_CARD           45  // GPIO45
#define MISO              37  // GPIO37
#define CLK               36  // GPIO36
#define MOSI              35  // GPIO35

#define SDA                1  // GPIO01
#define SCL                2  // GPIO02




//Configuration
#define TX_GPIO_NUM        (gpio_num_t)CONFIG_ESP32TWAI_TX_GPIO_NUM      // Im Konfigmenü zu diesem Projekt auf 14 gestellt
#define RX_GPIO_NUM        (gpio_num_t)CONFIG_ESP32TWAI_RX_GPIO_NUM      // Im Konfigmenü zu diesem Projekt auf 13 gestellt


static twai_message_t TWAI_RX_message;
static twai_message_t TWAI_TX_message;

static esp_err_t _TWAI_RX_state;
static esp_err_t _TWAI_TX_state;

static TaskHandle_t _RX_task;


static twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
// static twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

//Set normal mode, TX & RX queue to hold 5 messages
static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                              .tx_io = TX_GPIO_NUM, .rx_io = RX_GPIO_NUM,
                                              .clkout_io = TWAI_IO_UNUSED, .bus_off_io = TWAI_IO_UNUSED,
                                              .tx_queue_len = 5, .rx_queue_len = 5,
                                              .alerts_enabled = TWAI_ALERT_NONE,
                                              .clkout_divider = 0,
                                              .intr_flags = ESP_INTR_FLAG_LEVEL1};





// ----------------------------------------------------------------------
ESP32TWAIClass::ESP32TWAIClass() :
  CANControllerClass()
{
}

ESP32TWAIClass::~ESP32TWAIClass()
{
}


// ----------------------------------------------------------------------
int ESP32TWAIClass::begin(long baudRate)
{
  ESP_LOGI(LOGGING_TAG, "BEGIN");

/*
  pinMode(SWITCHED_3V3, OUTPUT);
  digitalWrite(SWITCHED_3V3, HIGH);
*/

  pinMode(SENSE_V_DIG, INPUT_PULLUP);

  pinMode(FORCE_KEEP_ON, OUTPUT);
  digitalWrite(FORCE_KEEP_ON, HIGH);

/*
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(BLUE_LED, LOW);
 */

  pinMode(YELLOW_LED, OUTPUT);
  digitalWrite(YELLOW_LED, HIGH);



  CANControllerClass::begin(baudRate);

  switch (baudRate) {
    case (long)1000E3:
      t_config = TWAI_TIMING_CONFIG_1MBITS();
      break;

    case (long)500E3:
      t_config = TWAI_TIMING_CONFIG_500KBITS();
      break;

    case (long)250E3:
      t_config = TWAI_TIMING_CONFIG_250KBITS();
      break;

    case (long)125E3:
      t_config = TWAI_TIMING_CONFIG_125KBITS();
      break;

    case (long)100E3:
      t_config = TWAI_TIMING_CONFIG_100KBITS();
      break;

    default:
      return 0;
  }


// set filter to allow anything
f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// set filter (oly)
/*
uint32_t acccode = (uint32_t)0x7e8 <<21;
uint32_t accmask = ((uint32_t)(0x7e8 ^ 0x7ec) <<21) | 0x1fffff; // 0x7e8 XOR 0x7ec  // VIN (smart & a3)

f_config = {.acceptance_code = acccode, .acceptance_mask = accmask, .single_filter = true};
*/

  //Install and start TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(LOGGING_TAG, "Driver installed");

  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(LOGGING_TAG, "Driver started");

  ESP_LOGI(LOGGING_TAG, "code: %x", f_config.acceptance_code);
  ESP_LOGI(LOGGING_TAG, "mask: %x", f_config.acceptance_mask);

  ESP_LOGI(LOGGING_TAG, "3s to go");
  vTaskDelay(pdMS_TO_TICKS(3000));


  digitalWrite(YELLOW_LED, LOW);
  return 1;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::filter(int id, int mask)
{ // 11bit
  id = (uint32_t)(id & TWAI_STD_ID_MASK) <<21;                    // = TWAI_STD_ID_MASK (0x7FF)
  mask = ((uint32_t)(mask & TWAI_STD_ID_MASK) <<21) | 0x1fffff;

  ESP_ERROR_CHECK(twai_stop());
  ESP_LOGI(LOGGING_TAG, "filter: Driver stopped");

  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(LOGGING_TAG, "filter: Driver uninstalled");  

  f_config = {.acceptance_code = (uint32_t)id, .acceptance_mask = (uint32_t)mask, .single_filter = true};
  ESP_LOGI(LOGGING_TAG, "code: %x", f_config.acceptance_code);
  ESP_LOGI(LOGGING_TAG, "mask: %x", f_config.acceptance_mask);

  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(LOGGING_TAG, "Driver installed");

  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(LOGGING_TAG, "filter: Driver started");

  return 1;
}


// ----------------------------------------------------------------------
int ESP32TWAIClass::filterExtended(long id, long mask)
{ // 29bit
  id = (uint32_t)(id & TWAI_EXTD_ID_MASK) <<3;                // = TWAI_EXTD_ID_MASK (0x1FFFFFFF)
  mask = ((uint32_t)(mask & TWAI_EXTD_ID_MASK) <<3) | 0x7;

  ESP_ERROR_CHECK(twai_stop());
  ESP_LOGI(LOGGING_TAG, "filter: Driver stopped");

  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(LOGGING_TAG, "filter: Driver uninstalled");  

  f_config = {.acceptance_code = (uint32_t)id, .acceptance_mask = (uint32_t)mask, .single_filter = true};
  ESP_LOGI(LOGGING_TAG, "code: %x", f_config.acceptance_code);
  ESP_LOGI(LOGGING_TAG, "mask: %x", f_config.acceptance_mask);

  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(LOGGING_TAG, "Driver installed");

  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(LOGGING_TAG, "filter: Driver started");


  return 1;
}
// ----------------------------------------------------------------------
void ESP32TWAIClass::end()
{
  //Stop and uninstall TWAI driver
  ESP_ERROR_CHECK(twai_stop());
  ESP_LOGI(LOGGING_TAG, "Driver stopped");
  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(LOGGING_TAG, "Driver uninstalled");

  CANControllerClass::end();
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::endPacket()
{
  if (!CANControllerClass::endPacket()) {
    return 0;
  }
 // ESP_LOGI(LOGGING_TAG, "endPacket"); 


  TWAI_TX_message.extd = _txExtended;
  TWAI_TX_message.identifier = _txId;
  TWAI_TX_message.data_length_code = (uint8_t)_txDlc;

  if (_txRtr) {
    ESP_LOGI(LOGGING_TAG, "endPacket: RtR Message");     
    TWAI_TX_message.rtr = true;
  } else {
    TWAI_TX_message.rtr = false;

    for (int i = 0; i < _txLength; i++) {
      TWAI_TX_message.data[i] = _txData[i];
    }
  }

  digitalWrite(YELLOW_LED, HIGH);
  _TWAI_TX_state = twai_transmit(&TWAI_TX_message, pdMS_TO_TICKS(TWAI_DEFAULT_TIMEOUT));
  digitalWrite(YELLOW_LED, LOW);

  if (_TWAI_TX_state == ESP_OK) {
    ESP_LOGI(LOGGING_TAG, "TX %03x <- (l:%02x) (d:%02x %02x %02x %02x %02x %02x %02x %02x)", TWAI_TX_message.identifier,  TWAI_TX_message.data_length_code, TWAI_TX_message.data[0],  TWAI_TX_message.data[1],  TWAI_TX_message.data[2],  TWAI_TX_message.data[3],  TWAI_TX_message.data[4],  TWAI_TX_message.data[5],  TWAI_TX_message.data[6],  TWAI_TX_message.data[7]);
  }else{
      switch (_TWAI_TX_state) {
        case ESP_ERR_TIMEOUT:
          ESP_LOGE(LOGGING_TAG, "endPacket: ESP_ERR_TIMEOUT"); 
          break;

        case ESP_ERR_INVALID_ARG:
          ESP_LOGE(LOGGING_TAG, "endPacket: ESP_ERR_INVALID_ARG"); 
          break;

        case ESP_ERR_INVALID_STATE:
          ESP_LOGE(LOGGING_TAG, "endPacket: ESP_ERR_INVALID_STATE"); 
          break;

        default:
          ESP_LOGE(LOGGING_TAG, "endPacket: unknown error: %x", _TWAI_TX_state); 
          break;
      }    
    ESP_LOGE(LOGGING_TAG, "FAIL: TX %03x <- (l:%02x) (d:%02x %02x %02x %02x %02x %02x %02x %02x)", TWAI_TX_message.identifier,  TWAI_TX_message.data_length_code, TWAI_TX_message.data[0],  TWAI_TX_message.data[1],  TWAI_TX_message.data[2],  TWAI_TX_message.data[3],  TWAI_TX_message.data[4],  TWAI_TX_message.data[5],  TWAI_TX_message.data[6],  TWAI_TX_message.data[7]);
  }


  if (_TWAI_TX_state == ESP_OK) {
    return 1;
  }else{
    return 0;
  }

}

// ----------------------------------------------------------------------
int ESP32TWAIClass::parsePacket()
{
//  ESP_LOGI(LOGGING_TAG, "parsePacket: waiting for incoming packet"); 

  twai_message_t rx_msg;

  esp_err_t trErrCode = twai_receive(&rx_msg, pdMS_TO_TICKS(TWAI_DEFAULT_TIMEOUT));  // war: portMAX_DELAY
  digitalWrite(YELLOW_LED, HIGH);

  if (trErrCode == ESP_OK) {

    ESP_LOGI(LOGGING_TAG, "RX %03x -> (l:%02x) (d:%02x %02x %02x %02x %02x %02x %02x %02x)", rx_msg.identifier, rx_msg.data_length_code, rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3], rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);

    _rxExtended = (rx_msg.extd) ? true : false;
    _rxRtr = (rx_msg.rtr) ? true : false;
    _rxDlc = (rx_msg.data_length_code & 0x0f);
    _rxIndex = 0;

    _rxId = rx_msg.identifier;


    if (_rxRtr) {
        ESP_LOGI(LOGGING_TAG, "parsePacket: RtR Message");        
      _rxLength = 0;
    } else {
      _rxLength = _rxDlc;

      for (int i = 0; i < _rxLength; i++) {
        _rxData[i] = rx_msg.data[i];
      }
    }

    digitalWrite(YELLOW_LED, LOW);
    return _rxDlc;

  }else{
    switch (trErrCode) {
      case ESP_ERR_TIMEOUT:
        ESP_LOGE(LOGGING_TAG, "parsePacket: ESP_ERR_TIMEOUT"); 
        break;

      case ESP_ERR_INVALID_ARG:
        ESP_LOGE(LOGGING_TAG, "parsePacket: ESP_ERR_INVALID_ARG"); 
        break;

      case ESP_ERR_INVALID_STATE:
        ESP_LOGE(LOGGING_TAG, "parsePacket: ESP_ERR_INVALID_STATE"); 
        break;

      default:
        ESP_LOGE(LOGGING_TAG, "parsePacket: unknown error: %x", trErrCode); 
        break;
    }

    digitalWrite(YELLOW_LED, LOW);
    return 0;     // error or timeout
  }

}



// ----------------------------------------------------------------------
// ----------------------------------------------------------------------
int ESP32TWAIClass::udsRead(uint32_t txECU, uint16_t txID, uint32_t rxECU, uint8_t* rxData, uint8_t rxLen)
// schickt ein UDS Kommando (0x22) an Steuergerät txID und schreibt die empfangenen Bytes nach rxData
{
  ESP_LOGI(LOGGING_TAG, "udsRead");

  if ((txECU > CAN_EXTD_ID_MASK) || (rxECU > CAN_EXTD_ID_MASK)) {
    return 0;
  }

  bool extMsg;

  if ((txECU & CAN_STD_ID_MASK) == txECU) {
    extMsg = false;
    beginPacket(txECU, 8);              // 11bit
  }else{
    extMsg = true;    
    beginExtendedPacket(txECU, 8);      // 29bit
  }

  clearRXqueue();                       // erstmal die Schlange leeren

  
  for (int retries = 10; retries > 0; retries--) {

    write(0x03);                        // number of additional bytes
    write(0x22);                        // UDS SID "read data by identifier"
    write((txID & 0xff00) >> 8);        // identifier 1st byte
    write(txID & 0xff);                 // identifier 2nd byte

    if (endPacket()) {
      // send success
      break;
    } else if (retries <= 1) {
        ESP_LOGE(LOGGING_TAG, "udsRead: request send FAIL");
        return 0;
    }
  }

  bool splitResponse = (rxLen > 4);     // es werden mehr als 4 Bytes angefordert, das wird 'ne Multiframe-Antwort

  if (rxLen > 4) {
    ESP_LOGW(LOGGING_TAG, "udsRead: expecting MultiFrame Message");
/* 
VIN (10 14 62 F8 02 57 56 57)
0x1014 which means extended message with a length of 0x14 = 20 bytes. 
Then 0x62 means a reply to the 0x22 UDS READ request and 0xF802 is the PID that was requested. 
The last 3 Bytes (57 56 57) are the first 3 Bytes of data.

 */

    for (unsigned long start = millis(); (millis() - start) < CAN_DEFAULT_TIMEOUT;) {
      if ((parsePacket() != 0)
//              && (rxECU == 0 ? true : CAN.rxId() == rxECU)                    // hat die gewünschte ECU hat geantwortet
              && (CAN.read() == 0x10)                     // [0] 0x10 = Extended Message
              && (CAN.read())                             // [1] Länge der Nachricht
              && (CAN.read() == (0x22 | 0x40))            // [2] antwort auf Kommando 0x22 (ergibt 0x62)
              && (CAN.read() == ((txID & 0xff00) >> 8))   // [3] ist es die antwort auf identifier 1st byte
              && (CAN.read() == (txID & 0xff)) ) {        // [4] ist es die antwort auf identifier 2nd byte


        ESP_LOGI(LOGGING_TAG, "udsRead: SUCCESS - got response from %03x", CAN.rxId());

        int read = 0;
        read += CAN.readBytes((uint8_t*)rxData, 3);         // und jetzt die restlichen 3 bytes

        for (int i = 0; read < rxLen; i++) {
          delay(30);                                    // kurz warten mit dem ACK

          // send request for the next chunk
          if (extMsg) {
  //          CAN.beginExtendedPacket(OBD2_CAN29_BROADCAST_ID, 8); // 0x18db33f1
            CAN.beginExtendedPacket(CAN.rxId() -8, 8);  // so wird korrekt geantwortet
          } else {
            CAN.beginPacket(CAN.rxId() -8, 8);  // so wird korrekt geantwortet
          }
          CAN.write(0x30);                      // 0x30 = ok, schick' mehr (flow-control)
          CAN.write(0x01);                      // 0x01 = bitte nur einen weiteren Frame
          CAN.write(0x00);                      // 0x00 = so schnell wie möglich
          CAN.endPacket();

          ESP_LOGI(LOGGING_TAG, "wait for CAN Response");

          while ((CAN.parsePacket() == 0) || (CAN.read() != (0x21 + i))){
            if ((millis() - start) > CAN_DEFAULT_TIMEOUT){   // timeout
              return 0;
            }
          }
          while (CAN.available()) {
            ((uint8_t*)rxData)[read++] = CAN.read();
          }  
        }

        return read;
      }
    }

  }else{
    ESP_LOGW(LOGGING_TAG, "udsRead: expecting SingleFrame Message");    

    for (unsigned long start = millis(); (millis() - start) < CAN_DEFAULT_TIMEOUT;) {
      if ((parsePacket() != 0)
  //            && (CAN.rxId() == rxECU)                    // hat die gewünschte ECU hat geantwortet?
              && (CAN.read() > 0x03)                      // [0] antwortlänge, bei >3 sind daten dabei
              && (CAN.read() == (0x22 | 0x40))            // [1] antwort auf Kommando 0x22 (ergibt 0x62)
              && (CAN.read() == ((txID & 0xff00) >> 8))   // [2] ist es die antwort auf identifier 1st byte
              && (CAN.read() == (txID & 0xff)) ) {        // [3] ist es die antwort auf identifier 2nd byte


        ESP_LOGI(LOGGING_TAG, "udsRead: SUCCESS - got response");

        // got response
        return CAN.readBytes((uint8_t*)rxData, rxLen);    // restliche Datenbytes nach rxData einlesen (können max. 4 sein)
      }
    }
  }

  ESP_LOGE(LOGGING_TAG, "udsRead: FAIL - no response");
  // no or wrong response  
  return 0;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::canCMD(uint32_t txECU, uint8_t* txData, uint32_t rxECU)
// sendet 8 Bytes txData an txECU
{
  ESP_LOGI(LOGGING_TAG, "canCMD");
  ESP_LOGI(LOGGING_TAG, "TX %03x <- (d:%02x %02x %02x %02x %02x %02x %02x %02x)", txECU, txData[0], txData[1], txData[2], txData[3], txData[4], txData[5], txData[6], txData[7]);

  if (txECU > CAN_EXTD_ID_MASK) {
    return 0;
  }

  if ((txECU & CAN_STD_ID_MASK) == txECU) {
    beginPacket(txECU, 8);              // 11bit
  }else{
    beginExtendedPacket(txECU, 8);      // 29bit
  }

  for (int retries = 10; retries > 0; retries--) {
    write(txData, 8);

    if (endPacket()) {
      // send success
      break;
    } else if (retries <= 1) {
        ESP_LOGE(LOGGING_TAG, "canCMD: FAIL");
        return 0;
    }
  }

  ESP_LOGI(LOGGING_TAG, "canCMD: success");

//  return 1;


  ESP_LOGW(LOGGING_TAG, "canCMD: expecting SingleFrame Message");    

  for (unsigned long start = millis(); (millis() - start) < CAN_DEFAULT_TIMEOUT;) {
    if ((parsePacket() != 0)
            && (CAN.rxId() == rxECU)                    // hat die gewünschte ECU hat geantwortet?
            && (CAN.read() > 0x03)                      // [0] antwortlänge, bei >3 sind daten dabei
//            && (CAN.read() == (0x22 | 0x40))            // [1] antwort auf Kommando 0x22 (ergibt 0x62)
//            && (CAN.read() == ((txID & 0xff00) >> 8))   // [2] ist es die antwort auf identifier 1st byte
//            && (CAN.read() == (txID & 0xff))            // [3] ist es die antwort auf identifier 2nd byte
    ){

      ESP_LOGI(LOGGING_TAG, "canCMD: SUCCESS - got response");

      // got response
//      return CAN.readBytes((uint8_t*)rxData, rxLen);    // restliche Datenbytes nach rxData einlesen (können max. 4 sein)
      return 1;
    }
  }

  return 0;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::peekPacket(uint32_t identifier, uint8_t *CANdata, uint32_t timeout_ms)
// wartet timeout_ms bis ein CAN-Paket mit der ID identifier ge-broadcastet wird
// und schreibt alle 8 empfangenen CAN-Bytes nach CANdata
{
  ESP_LOGI(LOGGING_TAG, "peekPacket: waiting for a special incoming packet: 0x%x", identifier); 

  twai_message_t rx_msg;
  uint32_t pollStart = millis();

  while(millis() < (pollStart + timeout_ms)){

    esp_err_t trErrCode = twai_receive(&rx_msg, pdMS_TO_TICKS(TWAI_DEFAULT_TIMEOUT));  // war: portMAX_DELAY

    if (trErrCode == ESP_OK) {
      _rxExtended = (rx_msg.extd) ? true : false;
      _rxRtr = (rx_msg.rtr) ? true : false;
      _rxDlc = (rx_msg.data_length_code & 0x0f);
      _rxIndex = 0;
      _rxLength = 0;
      _rxId = rx_msg.identifier;

      if (_rxRtr) {
        ESP_LOGW(LOGGING_TAG, "peekPacket: RtR Message");        
        return 0;
      }

      if (rx_msg.identifier == identifier){
        ESP_LOGI(LOGGING_TAG, "peekPacket: special packet arrived");             
        ESP_LOGI(LOGGING_TAG, "RX %03x -> (l:%02x) (d:%02x %02x %02x %02x %02x %02x %02x %02x)", rx_msg.identifier, rx_msg.data_length_code, rx_msg.data[0], rx_msg.data[1], rx_msg.data[2], rx_msg.data[3], rx_msg.data[4], rx_msg.data[5], rx_msg.data[6], rx_msg.data[7]);

        _rxLength = _rxDlc;

        for (int i = 0; i < _rxLength; i++) {
          _rxData[i] = rx_msg.data[i];
          CANdata[i] = rx_msg.data[i];          
        }
        return _rxLength;
      }
    }else{
      switch (trErrCode) {
        case ESP_ERR_TIMEOUT:
          ESP_LOGE(LOGGING_TAG, "peekPacket: ESP_ERR_TIMEOUT"); 
          break;

        case ESP_ERR_INVALID_ARG:
          ESP_LOGE(LOGGING_TAG, "peekPacket: ESP_ERR_INVALID_ARG"); 
          break;

        case ESP_ERR_INVALID_STATE:
          ESP_LOGE(LOGGING_TAG, "peekPacket: ESP_ERR_INVALID_STATE"); 
          break;

        default:
          ESP_LOGE(LOGGING_TAG, "peekPacket: unknown error: %x", trErrCode); 
          break;
      }
    }
  }

  ESP_LOGE(LOGGING_TAG, "peekPacket: no special packet seen");   
  return 0;
}


// ----------------------------------------------------------------------
// registriert eine callback-funktion die aufgerufen werden soll 
// wenn daten auf dem CAN-Bus ankommen
void ESP32TWAIClass::onReceive(void(*callback)(int))
{
  CANControllerClass::onReceive(callback);
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::powerOff()
// if ignition is OFF & USB-Power is not connected - the Module will immediately loose power
{
  ESP_LOGI(LOGGING_TAG, "shutdown ESP32TWAI CAN Dongle in 3s");
  delay(3000);

  digitalWrite(FORCE_KEEP_ON, LOW);

  return 1;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::sleep()
{
  ESP_ERROR_CHECK(twai_stop());
  ESP_LOGI(LOGGING_TAG, "Driver stopped");
/*
  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(LOGGING_TAG, "Driver uninstalled");
*/
  return 1;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::wakeup()
{
/*  
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(LOGGING_TAG, "Driver installed");
 */  
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(LOGGING_TAG, "Driver started");

  return 1;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::clearRXqueue()
{
  ESP_ERROR_CHECK(twai_clear_receive_queue());
  ESP_LOGI(LOGGING_TAG, "RX queue cleared");

  return 1;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::clearTXqueue()
{
  ESP_ERROR_CHECK(twai_clear_transmit_queue());
  ESP_LOGI(LOGGING_TAG, "TX queue cleared");

  return 1;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::qIgnition()
{
  int ignition = digitalRead(SENSE_V_DIG);
//  ESP_LOGI(LOGGING_TAG, "qIginition: %d", ignition);

  return ignition;
}

// ----------------------------------------------------------------------
int ESP32TWAIClass::qVoltage()
{
  int voltage = analogRead(SENSE_V_ANA);
//  ESP_LOGI(LOGGING_TAG, "qVoltage: %d", voltage);

  return voltage;
}


// ----------------------------------------------------------------------
ESP32TWAIClass CAN;

// #endif
