# DS18B20_RMT

DS18B20 temperature sensor library for ESP32 using the RMT peripheral for 1-Wire communication. No bit-banging — all bus timing is handled in hardware.

## Features

- **Hardware 1-Wire timing** via RMT (TX + RX channels)
- **Background FreeRTOS task** — periodic measurement without blocking your `loop()`
- **Auto-scan & auto-assign** — new sensors are detected and assigned to free slots automatically
- **NVS persistence** — sensor address assignments survive reboot (ESP32 Preferences)
- **CRC8 validation** — on both bus scan and temperature readout
- **Thread-safe** — all public getters/setters use spinlock (`portMUX`)
- **Graceful shutdown** — `end()` signals the task to stop cleanly before releasing resources
- **Multiple instances** — each instance uses its own pin and RMT channels (max 4 instances per ESP32)
- **Input sanitization** — all `Cfg` parameters are clamped to safe ranges

## Supported Hardware

- ESP32, ESP32-S2, ESP32-S3, ESP32-C3 (any chip with RMT v2 peripheral)
- DS18B20 sensors (any number per bus, up to `MAX_SENSORS` = 8 per instance)

## Installation

Copy the `DS18B20_RMT` folder into your Arduino `libraries` directory:

```
Arduino/libraries/DS18B20_RMT/
├── library.properties
├── keywords.txt
├── README.md
├── src/
│   ├── DS18B20_task.h
│   └── DS18B20_task.cpp
└── examples/
    └── BasicUsage/
        └── BasicUsage.ino
```

## Quick Start

```cpp
#include <DS18B20_task.h>

DS18B20_task dallas;

void setup() {
  Serial.begin(115200);

  DS18B20_task::Cfg cfg;
  cfg.pin = 16;           // GPIO pin
  cfg.interval_ms = 5000; // measurement interval
  cfg.max_sensors = 4;    // max sensors to track

  if (dallas.init(cfg)) {
    Serial.println("OK");
  }
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    float t = dallas.getTemp(i);
    if (t != DS18B20_task::BAD_TEMP_C) {
      Serial.printf("[%d] %.2f C\n", i, t);
    }
  }
  delay(5000);
}
```

## Configuration

All parameters have safe defaults and are clamped in `init()`:

| Parameter | Default | Range | Description |
|---|---|---|---|
| `pin` | 255 (invalid) | valid GPIO | 1-Wire data pin. **Must be set.** |
| `interval_ms` | 10000 | >= 1000 | Measurement interval in ms |
| `max_sensors` | 8 | 1–8 | Max sensor slots to use |
| `pullup` | true | — | Enable internal pull-up on data pin |
| `taskStackWords` | 3072 | >= 2048 | FreeRTOS task stack size (words) |
| `task_priority` | 1 | 1–5 | FreeRTOS task priority |

## API Reference

### Lifecycle

| Method | Description |
|---|---|
| `bool init(const Cfg& cfg)` | Initialize RMT, load addresses from NVS, start background task. Returns `false` on failure. |
| `void end()` | Gracefully stop the task and release all RMT resources. Safe to call `init()` again after. |

### Temperature & Addresses

| Method | Description |
|---|---|
| `float getTemp(uint8_t index)` | Get last measured temperature for slot. Returns `BAD_TEMP_C` (-128) if invalid. |
| `uint64_t getAddr(uint8_t index)` | Get sensor address assigned to slot. Returns `ADDR_EMPTY` if unassigned. |
| `bool setAddr(uint8_t index, uint64_t addr, bool save=true)` | Assign address to slot. Pass `0` to auto-assign from scan. Pass `ADDR_EMPTY` to clear. |
| `bool clearAddr(uint8_t index, bool save=true)` | Clear slot (shortcut for `setAddr(index, ADDR_EMPTY)`). |

### Scan

| Method | Description |
|---|---|
| `void requestScan()` | Request a bus re-scan on next measurement cycle. |
| `uint8_t getFoundAddrs(uint64_t* out, uint8_t capacity)` | Copy last scan results into buffer. Returns count found. |

### Constants

| Constant | Value | Description |
|---|---|---|
| `MAX_SENSORS` | 8 | Maximum sensor slots per instance |
| `BAD_TEMP_C` | -128.0f | Returned when temperature is unavailable |
| `ADDR_EMPTY` | 0xFFFFFFFFFFFFFFFF | Empty/unassigned slot marker |

## Address Format

Addresses are stored as `uint64_t` with the family code (0x28 for DS18B20) as the most significant byte:

```
0x28FF1234567890AB
  ^^ family code
```

## Multiple Buses

```cpp
DS18B20_task bus1;
DS18B20_task bus2;

DS18B20_task::Cfg cfg1;
cfg1.pin = 16;
bus1.init(cfg1);

DS18B20_task::Cfg cfg2;
cfg2.pin = 17;
bus2.init(cfg2);
```

Each instance uses 1 TX + 1 RX RMT channel. ESP32 has 4 of each, so max 4 instances.

## Resource Usage

- **RAM**: ~200 bytes per instance (+ task stack)
- **Task stack**: 12 KB default (3072 words × 4), ~2 KB free with 8 sensors
- **CPU**: ~0.05% at 5s interval with 1 sensor
- **NVS**: 8 bytes per sensor slot (stored as `uint64_t`)
- **RMT**: 1 TX + 1 RX channel per instance

## License

MIT
