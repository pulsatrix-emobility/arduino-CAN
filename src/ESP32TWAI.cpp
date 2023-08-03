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
  ESP_LOGW(LOGGING_TAG, "FORCE_KEEP_ON for the ESP32TWAI CAN Dongle");

  pinMode(FORCE_KEEP_ON, OUTPUT);
  digitalWrite(FORCE_KEEP_ON, HIGH);
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

/*
  ESP_LOGI(LOGGING_TAG, "3s to go");
  vTaskDelay(pdMS_TO_TICKS(3000));
 */

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
    return _rxLength;

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
