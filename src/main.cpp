#include <ModbusRTUSlave.h>
#include <HX711.h>
#include <EEPROM.h>
#include <scale_logic.h>
#define HX_DOUT    3
#define HX_SCK     2
#define DE_RE_PIN  4
// No sample for this long = load cell fault. HX711 is 10 SPS by default and
// needs ~400 ms after power-up.
#define HX_TIMEOUT_MS 500
HX711 scale;
class Hx711LoadCell : public LoadCell
{
public:
  bool isReady() override { return scale.is_ready(); }
  long read() override { return scale.read(); }   // returns at once when isReady()
};
Hx711LoadCell loadCell;
Sampler sampler;
ModbusRTUSlave modbus(Serial, DE_RE_PIN);
uint16_t regs[30];
enum
{
  REG_WEIGHT      = 0,
  REG_STATUS      = 1,
  REG_CMD         = 12,
  REG_KNOWN_W     = 13,
  REG_ADDR        = 20,
  REG_BAUD        = 21
};
// REG_STATUS bits
#define STATUS_LOADCELL_FAULT 0x0001   // no HX711 sample for HX_TIMEOUT_MS
#define STATUS_CAL_REJECTED   0x0002   // last calibration rejected (no load / no known weight)
#define STATUS_OVERRANGE      0x0004   // weight clamped to the int16 range
Config cfg;
// Command waiting for the next full block of samples (CMD_NONE if idle).
uint16_t pendingCmd = CMD_NONE;
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
  if (!isConfigValid(cfg)) {
    cfg = defaultConfig();
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
void setStatus(uint16_t bit, bool on)
{
  if (on)
    regs[REG_STATUS] |= bit;
  else
    regs[REG_STATUS] &= ~bit;
}
void setup() {
  loadConfig();
  restartModbus();
  scale.begin(HX_DOUT, HX_SCK);
  sampler.begin(millis());
  regs[REG_KNOWN_W] = cfg.knownWeight;
  regs[REG_ADDR] = cfg.address;
  regs[REG_BAUD] = cfg.baudCode;
}
void runCommand(uint16_t cmd, long raw)
{
  if (cmd == CMD_TARE)
  {
    cfg.offset = raw;
    saveConfig();
  }
  else if (cmd == CMD_CALIBRATE)
  {
    bool ok = computeScaleFactor(raw, cfg.offset, cfg.knownWeight, cfg.scaleFactor);
    setStatus(STATUS_CAL_REJECTED, !ok);
    if (ok)
      saveConfig();
  }
}
void loop() {
  modbus.poll();
  // KNOWN WEIGHT (only save, don't recalibrate)
  if (regs[REG_KNOWN_W] != cfg.knownWeight)
  {
    cfg.knownWeight = regs[REG_KNOWN_W];
    saveConfig();
  }
  // COMMAND: accepted on write, run on the next full block so it only uses
  // samples taken after it arrived. REG_CMD reads non-zero until it is done.
  if (regs[REG_CMD] != pendingCmd)
  {
    pendingCmd = acceptCommand(regs[REG_CMD], regs[REG_KNOWN_W]);
    regs[REG_CMD] = pendingCmd;
    if (pendingCmd != CMD_NONE)
      sampler.restart();
  }
  // WEIGHT (never waits for the HX711; on fault the last good weight stays)
  uint32_t now = millis();
  long raw;
  if (sampler.poll(loadCell, cfg.averageCount, now, raw))
  {
    if (pendingCmd != CMD_NONE)
    {
      runCommand(pendingCmd, raw);
      pendingCmd = CMD_NONE;
      regs[REG_CMD] = CMD_NONE;
    }
    bool overRange;
    regs[REG_WEIGHT] = weightToRegister(rawToUnits(raw, cfg.offset, cfg.scaleFactor), overRange);
    setStatus(STATUS_OVERRANGE, overRange);
  }
  // LOAD CELL FAULT: drop any pending command
  bool fault = sampler.faulted(now, HX_TIMEOUT_MS);
  setStatus(STATUS_LOADCELL_FAULT, fault);
  if (fault && pendingCmd != CMD_NONE)
  {
    pendingCmd = CMD_NONE;
    regs[REG_CMD] = CMD_NONE;
  }
  // ADDRESS
  if (applyAddressWrite(regs[REG_ADDR], cfg.address))
  {
    saveConfig();
    restartModbus();
  }
  // BAUD
  if (applyBaudWrite(regs[REG_BAUD], cfg.baudCode))
  {
    saveConfig();
    restartModbus();
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