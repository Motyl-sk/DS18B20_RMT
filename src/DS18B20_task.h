#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "freertos/queue.h"

class DS18B20_task {

public:
  static constexpr uint8_t  MAX_SENSORS = 8;
  static constexpr float    BAD_TEMP_C  = -128.0f;
  static constexpr uint64_t ADDR_EMPTY  = 0xFFFFFFFFFFFFFFFFULL;

  struct Cfg {
    uint8_t pin = 255;
    uint32_t interval_ms = 10000;
    uint8_t max_sensors = MAX_SENSORS;
    bool pullup = true;
    uint32_t taskStackWords = 3072;
    UBaseType_t task_priority = 1;
  };

  DS18B20_task();
  ~DS18B20_task();

  // Inicializuje a spustí background task podľa nastaveného configu
  bool init(const Cfg& cfg);

  void requestScan();
  float getTemp(uint8_t index) const;
  uint64_t getAddr(uint8_t index) const;
  bool setAddr(uint8_t index, uint64_t address, bool save = true);
  bool clearAddr(uint8_t index, bool save = true);
  uint8_t getFoundAddrs(uint64_t* out, uint8_t capacity) const;

  // Uvoľní všetky RMT zdroje a vypne 1-Wire (de-inicializácia)
  void end();

private:
  enum class Error : uint8_t {
    OK = 0,
    CRC,
    BadData,
    Timeout,
    Driver
  };

  static constexpr uint8_t  MAX_BLOCKS = 64;
  static constexpr uint32_t MIN_STACK_WORDS = 2048;

  // --- 1-Wire/DS18B20 protocol constants ---
  static constexpr uint16_t OW_RESET_PULSE = 500;
  static constexpr uint16_t OW_RESET_WAIT = 200;
  static constexpr uint16_t OW_RESET_PRESENCE_WAIT_MIN = 15;
  static constexpr uint16_t OW_RESET_PRESENCE_MIN = 60;
  static constexpr uint16_t OW_SLOT_BIT_SAMPLE_TIME = 15;
  static constexpr uint16_t OW_SLOT_START = 2;
  static constexpr uint16_t OW_SLOT_BIT = 60;
  static constexpr uint16_t OW_SLOT_RECOVERY = 5;
  static constexpr uint16_t OW_TIMEOUT = 50;

  // Internal tables (slot -> address / temperature)
  uint64_t _addr[MAX_SENSORS];
  float    _temp[MAX_SENSORS];

  uint8_t  _pin = 255;
  uint32_t _intervalMs = 10000;
  uint8_t  _maxSensors = MAX_SENSORS;
  volatile bool _started = false;
  TaskHandle_t _starter = nullptr;

  // Internal scan cache
  uint64_t _found[MAX_SENSORS] = {0};
  uint8_t  _foundCount = 0;

  // Control
  volatile bool _scanRequested = true;
  volatile bool _stopRequested = false;
  uint8_t _failStreak = 0;
  TaskHandle_t _task = nullptr;
  portMUX_TYPE _mux = portMUX_INITIALIZER_UNLOCKED;

  // --- 1-Wire/DS18B20 RMT and protocol state ---
  rmt_channel_handle_t _owTxChannel = nullptr;
  rmt_channel_handle_t _owRxChannel = nullptr;
  rmt_encoder_handle_t _owCopyEncoder = nullptr;
  rmt_encoder_handle_t _owBytesEncoder = nullptr;
  rmt_symbol_word_t _owBuf[MAX_BLOCKS];
  QueueHandle_t _owQueue = nullptr;
  bool _oneWireReady = false;

  // --- 1-Wire/DS18B20 protocol helpers ---
  bool oneWireReset();
  bool oneWireWrite(const uint8_t data, uint8_t len = 8);
  bool oneWireRead(uint8_t &data, uint8_t len = 8);
  bool oneWireRequest();
  Error oneWireGetTemp(uint64_t &addr, float &temp);
  uint8_t oneWireSearch(uint64_t *addresses, uint8_t total);

  // --- 1-Wire/DS18B20 static protocol symbols ---
  static rmt_symbol_word_t _owBit0;
  static rmt_symbol_word_t _owBit1;
  static const rmt_transmit_config_t _owTxConf;
  static const rmt_receive_config_t _owRxConf;

  // --- Task helpers ---
  static void taskThunk(void* arg);
  void taskBody();
  void doScan();
  void autoAssignFound();
  bool isFound(uint64_t addr) const;
};
