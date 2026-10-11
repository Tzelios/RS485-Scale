// Hardware-free scale logic, kept out of main.cpp so it can be unit tested
// on the host (pio test -e native).
#pragma once
#include <stdint.h>

// Stored as-is at EEPROM address 0. Changing field order or types breaks
// the configuration already saved on deployed boards.
struct Config
{
  uint8_t address;
  uint8_t baudCode;
  float scaleFactor;
  uint16_t knownWeight;
  uint16_t averageCount;
  long offset;          // Changed to long (HX711 offset is a long)
};

const uint8_t MAX_ADDRESS = 247;          // Modbus reserves 248-255
const uint16_t MAX_AVERAGE_COUNT = 255;   // keeps the raw sum within a 32-bit long
const long RAW_MIN = -8388608;            // HX711 is a 24-bit signed ADC
const long RAW_MAX = 8388607;

Config defaultConfig();
bool isConfigValid(const Config &cfg);

// Apply a master's write to the address/baud holding register.
// Returns true when the stored value changed (caller must save and restart
// Modbus). Always leaves reg equal to the value in use.
bool applyAddressWrite(uint16_t &reg, uint8_t &address);
bool applyBaudWrite(uint16_t &reg, uint8_t &baudCode);

enum : uint16_t
{
  CMD_NONE      = 0,
  CMD_TARE      = 1,
  CMD_CALIBRATE = 2
};
// Command to start for a write to the command register: CMD_NONE for unknown
// commands and for a calibration with no known weight.
uint16_t acceptCommand(uint16_t cmd, uint16_t knownWeight);

// Smallest |reading - offset|, in raw counts, accepted for calibration.
const long MIN_CAL_COUNTS = 100;
// Sets factor and returns true, or returns false (factor untouched) when the
// known weight is 0 or the reading is too close to the tare offset.
bool computeScaleFactor(long reading, long offset, uint16_t knownWeight, float &factor);

// Same formula as HX711::get_units.
float rawToUnits(long raw, long offset, float scaleFactor);

// Weight in scale units -> register value with 0.1 resolution, clamped to the
// int16 range. overRange is set when the value was clamped or not a number.
int16_t weightToRegister(float units, bool &overRange);

// Load cell seen through the two calls the sampler needs, so it can be
// faked in tests. main.cpp adapts the HX711 to this.
struct LoadCell
{
  virtual bool isReady() = 0;
  virtual long read() = 0;
};

// Non-blocking block averager: poll() once per loop takes at most one sample
// and never waits for the load cell.
class Sampler
{
public:
  void begin(uint32_t nowMs);
  // Returns true (and sets avg) when `count` samples have been collected;
  // the next block then starts empty. A count of 0 is treated as 1.
  bool poll(LoadCell &cell, uint16_t count, uint32_t nowMs, long &avg);
  // Drop the partly collected block.
  void restart();
  // True when no sample has arrived for more than timeoutMs.
  bool faulted(uint32_t nowMs, uint32_t timeoutMs) const;

private:
  long sum_ = 0;
  uint16_t n_ = 0;
  uint32_t lastSampleMs_ = 0;
};
