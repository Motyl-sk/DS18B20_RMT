#include "DS18B20_task.h"
#include <Preferences.h>

// CRC tabuľka (z OneWireESP32)
static const uint8_t crc_table[] = {0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65, 157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220, 35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98, 190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255, 70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7, 219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154, 101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36, 248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185, 140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205, 17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80, 175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238, 50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115, 202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139, 87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22, 233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168, 116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53};

rmt_symbol_word_t DS18B20_task::_owBit0 = {
    .duration0 = DS18B20_task::OW_SLOT_START + DS18B20_task::OW_SLOT_BIT,
    .level0 = 0,
    .duration1 = DS18B20_task::OW_SLOT_RECOVERY,
    .level1 = 1
};
rmt_symbol_word_t DS18B20_task::_owBit1 = {
    .duration0 = DS18B20_task::OW_SLOT_START,
    .level0 = 0,
    .duration1 = DS18B20_task::OW_SLOT_BIT + DS18B20_task::OW_SLOT_RECOVERY,
    .level1 = 1
};
const rmt_transmit_config_t DS18B20_task::_owTxConf = {
    .loop_count = 0,
    .flags = { .eot_level = 1 }
};
const rmt_receive_config_t DS18B20_task::_owRxConf = {
    .signal_range_min_ns = 1000,
    .signal_range_max_ns = (DS18B20_task::OW_RESET_PULSE + DS18B20_task::OW_RESET_WAIT) * 1000,
};

DS18B20_task::DS18B20_task() {
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    _addr[i] = ADDR_EMPTY;
    _temp[i] = BAD_TEMP_C;
  }
}

DS18B20_task::~DS18B20_task() {
  end();
}

// --- Privátne metódy ---

bool DS18B20_task::oneWireReset() {
    rmt_symbol_word_t symbol_reset = {
        .duration0 = OW_RESET_PULSE,
        .level0 = 0,
        .duration1 = OW_RESET_WAIT,
        .level1 = 1
    };
    rmt_rx_done_event_data_t evt;
    rmt_receive(_owRxChannel, _owBuf, sizeof(_owBuf), &_owRxConf);
    rmt_transmit(_owTxChannel, _owCopyEncoder, &symbol_reset, sizeof(rmt_symbol_word_t), &_owTxConf);
    bool found = false;
    if (xQueueReceive(_owQueue, &evt, pdMS_TO_TICKS(OW_TIMEOUT)) == pdTRUE) {
        size_t symbol_num = evt.num_symbols;
        rmt_symbol_word_t *symbols = evt.received_symbols;
        if (symbol_num > 1) {
            if (symbols[0].level1 == 1) {
                if (symbols[0].duration1 > OW_RESET_PRESENCE_WAIT_MIN && symbols[1].duration0 > OW_RESET_PRESENCE_MIN) {
                    found = true;
                }
            } else {
                if (symbols[0].duration0 > OW_RESET_PRESENCE_WAIT_MIN && symbols[1].duration1 > OW_RESET_PRESENCE_MIN) {
                    found = true;
                }
            }
        }
        if (rmt_tx_wait_all_done(_owTxChannel, OW_TIMEOUT) != ESP_OK) {
            found = false;
        }
    }
    return found;
}

bool DS18B20_task::oneWireRead(uint8_t &data, uint8_t len) {
    rmt_rx_done_event_data_t evt;
    rmt_receive(_owRxChannel, _owBuf, sizeof(_owBuf), &_owRxConf);
    if (!oneWireWrite((len > 1) ? 0xff : 1, len) || xQueueReceive(_owQueue, &evt, pdMS_TO_TICKS(OW_TIMEOUT)) != pdTRUE) {
        return false;
    }
    size_t symbol_num = evt.num_symbols;
    rmt_symbol_word_t *symbol = evt.received_symbols;
    data = 0;
    for (uint8_t i = 0; i < symbol_num && i < 8; i++) {
        if (!(symbol[i].duration0 > OW_SLOT_BIT_SAMPLE_TIME)) {
            data |= 1 << i;
        }
    }
    if (len != 8) { data = data & 0x01; }
    return true;
}

bool DS18B20_task::oneWireWrite(const uint8_t data, uint8_t len) {
    if (len < 8) {
        const rmt_symbol_word_t *sb;
        for (uint8_t i = 0; i < len; i++) {
            sb = &_owBit0;
            if ((data & (1 << i)) != 0) {
                sb = &_owBit1;
            }
            if (rmt_transmit(_owTxChannel, _owCopyEncoder, sb, sizeof(rmt_symbol_word_t), &_owTxConf) != ESP_OK) {
                return false;
            }
        }
    } else {
        if (rmt_transmit(_owTxChannel, _owBytesEncoder, &data, 1, &_owTxConf) != ESP_OK) {
            return false;
        }
    }
    return (rmt_tx_wait_all_done(_owTxChannel, OW_TIMEOUT) == ESP_OK);
}

bool DS18B20_task::oneWireRequest() {
    if (_oneWireReady && oneWireReset()) {
        oneWireWrite(0xCC);
        oneWireWrite(0x44);
        return true;
    }
    return false;
}

DS18B20_task::Error DS18B20_task::oneWireGetTemp(uint64_t &addr, float &temp) {
    Error error = Error::OK;
    if (!_oneWireReady) { return Error::Driver; }
    if (oneWireReset()) {
        oneWireWrite(0x55);
        uint64_t wireAddr = __builtin_bswap64(addr);
        uint8_t *a = (uint8_t *)&wireAddr;
        for (uint8_t i = 0; i < 8; i++) {
            oneWireWrite(a[i]);
        }
        oneWireWrite(0xBE);
        uint8_t data[9]; uint16_t zero = 0; uint8_t crc = 0;
        for (uint8_t j = 0; j < 9; j++) {
            if (!oneWireRead(data[j], 8)) { data[j] = 0; }
            zero += data[j];
            if (j < 8) { crc = crc_table[crc ^ data[j]]; }
        }
        if (zero != 0x8f7 && zero) {
            if (data[8] == crc) {
                int16_t t = (data[1] << 8) | data[0];
                temp = ((float)t / 16.0);
            } else {
                error = Error::CRC;
            }
        } else {
            error = Error::BadData;
        }
    } else {
        error = Error::Timeout;
    }
    return error;
}

uint8_t DS18B20_task::oneWireSearch(uint64_t *addresses, uint8_t total) {
    int8_t last_src;
    int8_t last_dev = -1;
    uint8_t found = 0;
    uint8_t loop = 1;
    if (!_oneWireReady) { return found; }
    uint64_t addr = 0;
    while (loop && found < total) {
        loop = 0;
        last_src = last_dev;
        if (!oneWireReset()) {
            found = 0;
            break;
        }
        oneWireWrite(0xF0, 8);
        for (uint8_t i = 0; i < 64; i += 1) {
            uint8_t bitA, bitB; uint64_t m = 1ULL << i;
            if (!oneWireRead(bitA, 1) || !oneWireRead(bitB, 1) || (bitA && bitB)) {
                addr = found = loop = 0;
                break;
            } else if (!bitA && !bitB) {
                if (i == last_src) {
                    oneWireWrite(1, 1); addr |= m;
                } else {
                    if ((addr & m) == 0 || i > last_src) {
                        oneWireWrite(0, 1); loop = 1; addr &= ~m;
                        last_dev = i;
                    } else {
                        oneWireWrite(1, 1);
                    }
                }
            } else {
                if (bitA) {
                    oneWireWrite(1, 1); addr |= m;
                } else {
                    oneWireWrite(0, 1); addr &= ~m;
                }
            }
        }
        if (addr) {
            uint8_t *raw = (uint8_t*)&addr;
            uint8_t crc = 0;
            for (uint8_t b = 0; b < 7; b++) crc = crc_table[crc ^ raw[b]];
            if (crc == raw[7]) {
                addresses[found] = __builtin_bswap64(addr);
                found++;
            }
        }
    }
    return found;
}

// --- Preferences ---
static constexpr const char* PREFS_NS = "ds18b20";
static inline void makeKey(char* key, size_t n, uint8_t idx) {
  // "a0".."a7"
  snprintf(key, n, "a%u", (unsigned)idx);
}

bool DS18B20_task::init(const Cfg& cfg) {
  if (_started) return false;
  if (!GPIO_IS_VALID_GPIO(cfg.pin)) return false;
  _started = true;

  _pin = cfg.pin;
  _intervalMs = (cfg.interval_ms < 1000) ? 1000 : cfg.interval_ms;
  _maxSensors = (cfg.max_sensors == 0 || cfg.max_sensors > MAX_SENSORS) ? MAX_SENSORS : cfg.max_sensors;

  uint32_t stackWords = (cfg.taskStackWords < MIN_STACK_WORDS) ? MIN_STACK_WORDS : cfg.taskStackWords;
  UBaseType_t prio = (cfg.task_priority < 1) ? 1 : (cfg.task_priority > 5) ? 5 : cfg.task_priority;

  pinMode(_pin, cfg.pullup ? INPUT_PULLUP : INPUT);

  // --- 1-Wire RMT INIT ---
  rmt_bytes_encoder_config_t bnc = {
    .bit0 = _owBit0,
    .bit1 = _owBit1,
    .flags = { .msb_first = 0 }
  };
  if (rmt_new_bytes_encoder(&bnc, &_owBytesEncoder) != ESP_OK) { end(); return false; }
  rmt_copy_encoder_config_t cnc = {};
  if (rmt_new_copy_encoder(&cnc, &_owCopyEncoder) != ESP_OK) { end(); return false; }

  const rmt_rx_channel_config_t rxconf = {
    .gpio_num = (gpio_num_t)_pin,
    .clk_src = RMT_CLK_SRC_APB,
    .resolution_hz = 1000000,
    .mem_block_symbols = MAX_BLOCKS
  };
  if (rmt_new_rx_channel(&rxconf, &_owRxChannel) != ESP_OK) { end(); return false; }

  const rmt_tx_channel_config_t txconf = {
    .gpio_num = (gpio_num_t)_pin,
    .clk_src = RMT_CLK_SRC_APB,
    .resolution_hz = 1000000,
    .mem_block_symbols = MAX_BLOCKS,
    .trans_queue_depth = 4,
    .flags = { .io_loop_back = 1, .io_od_mode = 1 }
  };
  if (rmt_new_tx_channel(&txconf, &_owTxChannel) != ESP_OK) { end(); return false; }

  _owQueue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
  if (_owQueue == NULL) { end(); return false; }

  rmt_rx_event_callbacks_t rx_callbacks = { .on_recv_done = [](rmt_channel_handle_t ch, const rmt_rx_done_event_data_t *edata, void *udata) -> bool {
    BaseType_t h = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)udata, edata, &h);
    return (h == pdTRUE);
  }
  };
  if (rmt_rx_register_event_callbacks(_owRxChannel, &rx_callbacks, _owQueue) != ESP_OK) { end(); return false; }
  if (rmt_enable(_owRxChannel) != ESP_OK) { end(); return false; }
  if (rmt_enable(_owTxChannel) != ESP_OK) { end(); return false; }

  static rmt_symbol_word_t release_symbol = {
    .duration0 = 1,
    .level0 = 1,
    .duration1 = 0,
    .level1 = 1,
  };
  rmt_transmit(_owTxChannel, _owCopyEncoder, &release_symbol, sizeof(rmt_symbol_word_t), &_owTxConf);

  _oneWireReady = true;

  // Load address table from Preferences (flash reads outside critical section)
  {
    Preferences prefs;
    if (prefs.begin(PREFS_NS, true)) {
      uint64_t loaded[MAX_SENSORS];
      for (uint8_t i = 0; i < _maxSensors; i++) {
        char key[6];
        makeKey(key, sizeof(key), i);
        loaded[i] = prefs.getULong64(key, ADDR_EMPTY);
      }
      prefs.end();
      portENTER_CRITICAL(&_mux);
      for (uint8_t i = 0; i < _maxSensors; i++) {
        _addr[i] = loaded[i];
        _temp[i] = BAD_TEMP_C;
      }
      portEXIT_CRITICAL(&_mux);
    }
  }

  _starter = xTaskGetCurrentTaskHandle();

  if (xTaskCreate(taskThunk, "ds18b20", stackWords, this, prio, &_task) != pdPASS) {
    end();
    return false;
  }

  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2500)) == 0) {
    end();
    return false;
  }

  return true;
}

void DS18B20_task::requestScan() {
  _scanRequested = true;
}

float DS18B20_task::getTemp(uint8_t index) const {
  if (index >= _maxSensors) return BAD_TEMP_C;
  float v;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  v = _temp[index];
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  return v;
}

uint64_t DS18B20_task::getAddr(uint8_t index) const {
  if (index >= _maxSensors) return ADDR_EMPTY;
  uint64_t a;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  a = _addr[index];
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  return a;
}

bool DS18B20_task::setAddr(uint8_t index, uint64_t address, bool save) {
  if (index >= _maxSensors) return false;

  uint64_t newAddr = address;

  portENTER_CRITICAL(&_mux);

  // 1) EXPLICIT CLEAR: ADDR_EMPTY znamená vymaž slot
  if (newAddr == ADDR_EMPTY) {
    _addr[index] = ADDR_EMPTY;
    _temp[index] = BAD_TEMP_C;
    portEXIT_CRITICAL(&_mux);

    if (save) {
      Preferences prefs;
      if (prefs.begin(PREFS_NS, false)) {
        char key[6];
        makeKey(key, sizeof(key), index);
        prefs.putULong64(key, ADDR_EMPTY);
        prefs.end();
      }
    }
    return true;
  }

  // 2) AUTO MODE: address == 0 => vyber prvé neznáme čidlo zo scanu
  if (newAddr == 0) {
    bool found = false;

    for (uint8_t i = 0; i < _foundCount; i++) {
      uint64_t candidate = _found[i];

      // je už použité v inom slote?
      bool used = false;
      for (uint8_t s = 0; s < _maxSensors; s++) {
        if (_addr[s] == candidate) {
          used = true;
          break;
        }
      }

      if (!used) {
        newAddr = candidate;
        found = true;
        break;
      }
    }

    if (!found) {
      portEXIT_CRITICAL(&_mux);
      return false; // žiadne nové čidlo
    }
  }

  // 3) zapíš adresu do slotu
  _addr[index] = newAddr;
  _temp[index] = BAD_TEMP_C;

  portEXIT_CRITICAL(&_mux);

  // 4) ulož do Preferences
  if (save) {
    Preferences prefs;
    if (prefs.begin(PREFS_NS, false)) {
      char key[6];
      makeKey(key, sizeof(key), index);
      prefs.putULong64(key, newAddr);
      prefs.end();
    }
  }

  return true;
}


bool DS18B20_task::clearAddr(uint8_t index, bool save) {
  return setAddr(index, ADDR_EMPTY, save);
}

uint8_t DS18B20_task::getFoundAddrs(uint64_t* out, uint8_t capacity) const {
  if (!out || capacity == 0) return 0;
  uint8_t n;
  portENTER_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  n = _foundCount;
  if (n > capacity) n = capacity;
  for (uint8_t i = 0; i < n; i++) out[i] = _found[i];
  portEXIT_CRITICAL(const_cast<portMUX_TYPE*>(&_mux));
  return n;
}

void DS18B20_task::taskThunk(void* arg) {
  auto* self = static_cast<DS18B20_task*>(arg);
  self->taskBody();
  vTaskDelete(nullptr);
}

bool DS18B20_task::isFound(uint64_t addr) const {
  for (uint8_t i = 0; i < _foundCount; i++) {
    if (_found[i] == addr) return true;
  }
  return false;
}

void DS18B20_task::doScan() {
  uint64_t tmp[MAX_SENSORS] = {};
  uint8_t n = oneWireSearch(tmp, _maxSensors);
  portENTER_CRITICAL(&_mux);
  for (uint8_t i = 0; i < _maxSensors; i++) _found[i] = (i < n) ? tmp[i] : 0;
  _foundCount = n;
  portEXIT_CRITICAL(&_mux);
}

void DS18B20_task::autoAssignFound() {
  uint8_t newSlots[MAX_SENSORS];
  uint8_t newCount = 0;

  portENTER_CRITICAL(&_mux);
  for (uint8_t f = 0; f < _foundCount; f++) {
    uint64_t candidate = _found[f];
    if (candidate == 0 || candidate == ADDR_EMPTY) continue;

    bool assigned = false;
    for (uint8_t s = 0; s < _maxSensors; s++) {
      if (_addr[s] == candidate) { assigned = true; break; }
    }
    if (assigned) continue;

    for (uint8_t s = 0; s < _maxSensors; s++) {
      if (_addr[s] == ADDR_EMPTY) {
        _addr[s] = candidate;
        _temp[s] = BAD_TEMP_C;
        newSlots[newCount++] = s;
        break;
      }
    }
  }
  portEXIT_CRITICAL(&_mux);

  if (newCount > 0) {
    Preferences prefs;
    if (prefs.begin(PREFS_NS, false)) {
      for (uint8_t i = 0; i < newCount; i++) {
        char key[6];
        makeKey(key, sizeof(key), newSlots[i]);
        portENTER_CRITICAL(&_mux);
        uint64_t a = _addr[newSlots[i]];
        portEXIT_CRITICAL(&_mux);
        prefs.putULong64(key, a);
      }
      prefs.end();
    }
  }
}

void DS18B20_task::taskBody() {
  TickType_t lastWake = xTaskGetTickCount();

  doScan();
  autoAssignFound();

  // Handshake for init()
  if (_starter) xTaskNotifyGive(_starter);

  while (!_stopRequested) {
    if (_scanRequested || _failStreak >= 3) {
      _scanRequested = false;
      _failStreak = 0;
      doScan();
      autoAssignFound();
    }

    uint64_t addrSnap[MAX_SENSORS];
    portENTER_CRITICAL(&_mux);
    for (uint8_t i = 0; i < _maxSensors; i++) addrSnap[i] = _addr[i];
    portEXIT_CRITICAL(&_mux);

    bool anyActive = false;
    for (uint8_t i = 0; i < _maxSensors; i++) {
      uint64_t a = addrSnap[i];
      if (a != 0 && a != ADDR_EMPTY) { anyActive = true; break; }
    }

    if (anyActive) {
      bool anyFail = false;
      float tempLocal[MAX_SENSORS];
      for (uint8_t i = 0; i < _maxSensors; i++) tempLocal[i] = BAD_TEMP_C;

      if (oneWireRequest()) {
        vTaskDelay(pdMS_TO_TICKS(750)); // 12-bit conversion

        for (uint8_t i = 0; i < _maxSensors; i++) {
          uint64_t addr = addrSnap[i];
          if (addr == 0 || addr == ADDR_EMPTY) continue;
          if (!isFound(addr)) { anyFail = true; continue; }

          float t;
          Error err = oneWireGetTemp(addr, t);
          if (err == Error::OK) {
            tempLocal[i] = t;
          } else {
            anyFail = true;
          }
        }
      } else {
        anyFail = true;
      }

      portENTER_CRITICAL(&_mux);
      for (uint8_t i = 0; i < _maxSensors; i++) _temp[i] = tempLocal[i];
      portEXIT_CRITICAL(&_mux);

      if (anyFail) {
        if (_failStreak < 255) _failStreak++;
      }
    } else {
      portENTER_CRITICAL(&_mux);
      for (uint8_t i = 0; i < _maxSensors; i++) _temp[i] = BAD_TEMP_C;
      portEXIT_CRITICAL(&_mux);
    }

    TickType_t now = xTaskGetTickCount();
    TickType_t elapsed = now - lastWake;
    TickType_t period = pdMS_TO_TICKS(_intervalMs);
    if (elapsed < period) {
      ulTaskNotifyTake(pdTRUE, period - elapsed);
    }
    lastWake = xTaskGetTickCount();
  }

  _task = nullptr;
}

void DS18B20_task::end() {
  if (!_started) return;

  if (_task) {
    _stopRequested = true;
    xTaskNotifyGive(_task);                    // prebuď ak spí vo vTaskDelayUntil
    while (_task) vTaskDelay(pdMS_TO_TICKS(1)); // počkaj kým sa task sám ukončí
  }

  if (_owRxChannel) {
    rmt_disable(_owRxChannel);
    rmt_del_channel(_owRxChannel);
    _owRxChannel = nullptr;
  }
  if (_owTxChannel) {
    rmt_disable(_owTxChannel);
    rmt_del_channel(_owTxChannel);
    _owTxChannel = nullptr;
  }
  if (_owBytesEncoder) {
    rmt_del_encoder(_owBytesEncoder);
    _owBytesEncoder = nullptr;
  }
  if (_owCopyEncoder) {
    rmt_del_encoder(_owCopyEncoder);
    _owCopyEncoder = nullptr;
  }
  if (_owQueue) {
    vQueueDelete(_owQueue);
    _owQueue = nullptr;
  }

  _oneWireReady = false;
  _stopRequested = false;
  _started = false;
}
