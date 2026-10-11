#include <ModbusRTUSlave.h>
#include <HX711.h>
#include <EEPROM.h>
#define HX_DOUT    3
#define HX_SCK     2
#define DE_RE_PIN  4
HX711 scale;
ModbusRTUSlave modbus(Serial, DE_RE_PIN);
uint16_t regs[30];
enum
{
  REG_WEIGHT      = 0,
  REG_CMD         = 12,
  REG_KNOWN_W     = 13,
  REG_ADDR        = 20,
  REG_BAUD        = 21
};
struct Config
{
  uint8_t address;
  uint8_t baudCode;
  float scaleFactor;
  uint16_t knownWeight;
  uint16_t averageCount;
  long offset;          // Changed to long (HX711 offset is a long)
};
Config cfg;
const long baudTable[] =
{
  0,
  2400,
  4800,
  9600
};
// ---------------- EEPROM ----------------
void loadConfig() {
  EEPROM.get(0, cfg);
  if (cfg.address < 1 || cfg.address > 255 ||
      cfg.baudCode < 1 || cfg.baudCode > 3 ||
      isnan(cfg.scaleFactor) || cfg.scaleFactor == 0) {
    cfg.address = 3;
    cfg.baudCode = 3;
    cfg.scaleFactor = 1.0;
    cfg.knownWeight = 4;
    cfg.averageCount = 30;
    cfg.offset = 0;
  }
}
void saveConfig() {
  EEPROM.put(0, cfg);
}
void restartModbus()
{
  Serial.end();
  delay(200);
  long baud = baudTable[cfg.baudCode];
  Serial.begin(baud);
  modbus.configureHoldingRegisters(regs, 30);
  modbus.begin(cfg.address, baud);
}
void setup() {
  loadConfig();
  restartModbus();
  scale.begin(HX_DOUT, HX_SCK);
  // **KEY FIX: Restore saved calibration on startup**
  scale.set_scale(cfg.scaleFactor);
  scale.set_offset(cfg.offset);
  regs[REG_KNOWN_W] = cfg.knownWeight;
  regs[REG_ADDR] = cfg.address;
  regs[REG_BAUD] = cfg.baudCode;
}
void loop() {
  modbus.poll();
  // WEIGHT
  int16_t weight10 = (int16_t)(scale.get_units(cfg.averageCount) * 10.0f);
  regs[REG_WEIGHT] = weight10;
  // TARE
  if (regs[REG_CMD] == 1)
  {
    scale.tare(cfg.averageCount);
    cfg.offset = scale.get_offset();  // Use get_offset() after tare
    saveConfig();
    regs[REG_CMD] = 0;
  }
  // CALIBRATE
  if (regs[REG_CMD] == 2)
  {
    if (regs[REG_KNOWN_W] > 0)
    {
      cfg.knownWeight = regs[REG_KNOWN_W];
      long reading = scale.read_average(cfg.averageCount);
      cfg.scaleFactor = (float)(reading - cfg.offset) / (float)cfg.knownWeight;
      scale.set_scale(cfg.scaleFactor);
      saveConfig();
    }
    regs[REG_CMD] = 0;
  }
  // KNOWN WEIGHT (only save, don't recalibrate)
  if (regs[REG_KNOWN_W] != cfg.knownWeight)
  {
    cfg.knownWeight = regs[REG_KNOWN_W];
    saveConfig();
  }
  // ADDRESS
  if (regs[REG_ADDR] != cfg.address)
  {
    uint8_t addr = regs[REG_ADDR];
    if (addr >= 1 && addr <= 255)
    {
      cfg.address = addr;
      saveConfig();
      restartModbus();
    }
  }
  // BAUD
  if (regs[REG_BAUD] != cfg.baudCode)
  {
    uint8_t baudCode = regs[REG_BAUD];
    if (baudCode >= 1 && baudCode <= 3)
    {
      cfg.baudCode = baudCode;
      saveConfig();
      restartModbus();
    }
  }
}

// void setup() {
//   // Initialize serial communication
//   Serial.begin(9600);
//   while (!Serial) {
//     ; // Wait for serial port to connect (needed for native USB boards like Leonardo/Micro)
//   }
//   Serial.println("RESETING EEPROM");
//   for (int i = 0; i < EEPROM.length(); i++) {
//     EEPROM.write(i, 255); // 255 (0xFF) resets EEPROM memory cells back to absolute blank factory state
//   }
//   Serial.println("EEPROM RESETED");
// }


// void loop(){

//   for (int i = 0; i < 32; i++) {
//     Serial.print(EEPROM.read(i), HEX);
//     Serial.print(" ");
//   }
//   Serial.println("GO GOGO");
//   delay(10000);
// }