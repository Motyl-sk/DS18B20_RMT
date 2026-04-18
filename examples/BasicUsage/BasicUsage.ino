#include <DS18B20_task.h>

#define ONE_WIRE_PIN  16
#define MAX_SENSORS   6

DS18B20_task dallas;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  DS18B20_task::Cfg cfg;
  cfg.pin = ONE_WIRE_PIN;
  cfg.interval_ms = 5000;
  cfg.max_sensors = MAX_SENSORS;

  if (dallas.init(cfg)) {
    Serial.println("DS18B20 started.");
  } else {
    Serial.println("DS18B20 init failed.");
  }
}

void loop() {
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint < 5000) return;
  lastPrint = millis();

  // Print found sensors
  uint64_t addrs[MAX_SENSORS] = {};
  uint8_t count = dallas.getFoundAddrs(addrs, MAX_SENSORS);
  Serial.printf("Sensors found: %d\n", count);
  for (uint8_t i = 0; i < count; i++) {
    Serial.printf("  [%d] %016llX\n", i, addrs[i]);
  }

  // Print temperatures
  Serial.println("Temperatures:");
  for (uint8_t i = 0; i < MAX_SENSORS; i++) {
    uint64_t addr = dallas.getAddr(i);
    if (addr == DS18B20_task::ADDR_EMPTY) continue;
    float t = dallas.getTemp(i);
    Serial.printf("  [%d] %016llX: %.2f C\n", i, addr, t);
  }
  Serial.println();
}
