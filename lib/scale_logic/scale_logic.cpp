#include "scale_logic.h"
#include <math.h>

Config defaultConfig()
{
  Config cfg;
  cfg.address = 3;
  cfg.baudCode = 3;
  cfg.scaleFactor = 1.0;
  cfg.knownWeight = 4;
  cfg.averageCount = 30;
  cfg.offset = 0;
  return cfg;
}

bool isConfigValid(const Config &cfg)
{
  return cfg.address >= 1 && cfg.address <= MAX_ADDRESS &&
         cfg.baudCode >= 1 && cfg.baudCode <= 3 &&
         isfinite(cfg.scaleFactor) && cfg.scaleFactor != 0 &&
         cfg.averageCount >= 1 && cfg.averageCount <= MAX_AVERAGE_COUNT &&
         cfg.offset >= RAW_MIN && cfg.offset <= RAW_MAX;
}

bool applyAddressWrite(uint16_t &reg, uint8_t &address)
{
  bool changed = false;
  if (reg != address && reg >= 1 && reg <= MAX_ADDRESS)
  {
    address = reg;
    changed = true;
  }
  reg = address;   // keep register in sync; reverts rejected values
  return changed;
}

bool applyBaudWrite(uint16_t &reg, uint8_t &baudCode)
{
  bool changed = false;
  if (reg != baudCode && reg >= 1 && reg <= 3)
  {
    baudCode = reg;
    changed = true;
  }
  reg = baudCode;  // keep register in sync; reverts rejected values
  return changed;
}

uint16_t acceptCommand(uint16_t cmd, uint16_t knownWeight)
{
  if (cmd == CMD_TARE)
    return CMD_TARE;
  if (cmd == CMD_CALIBRATE && knownWeight > 0)
    return CMD_CALIBRATE;
  return CMD_NONE;
}

bool computeScaleFactor(long reading, long offset, uint16_t knownWeight, float &factor)
{
  long delta = reading - offset;
  if (knownWeight == 0 || (delta < MIN_CAL_COUNTS && delta > -MIN_CAL_COUNTS))
    return false;
  factor = (float)delta / (float)knownWeight;
  return true;
}

float rawToUnits(long raw, long offset, float scaleFactor)
{
  return (float)(raw - offset) / scaleFactor;
}

int16_t weightToRegister(float units, bool &overRange)
{
  float scaled = units * 10.0f;
  overRange = true;
  if (isnan(scaled))
    return 0;
  if (scaled > 32767.0f)
    return 32767;
  if (scaled < -32768.0f)
    return -32768;
  overRange = false;
  return (int16_t)scaled;
}

void Sampler::begin(uint32_t nowMs) { lastSampleMs_ = nowMs; restart(); }

bool Sampler::poll(LoadCell &cell, uint16_t count, uint32_t nowMs, long &avg)
{
  if (!cell.isReady())
    return false;
  sum_ += cell.read();
  n_++;
  lastSampleMs_ = nowMs;
  if (n_ < (count == 0 ? 1 : count))
    return false;
  avg = sum_ / (long)n_;
  restart();
  return true;
}

void Sampler::restart() { sum_ = 0; n_ = 0; }

bool Sampler::faulted(uint32_t nowMs, uint32_t timeoutMs) const
{
  return nowMs - lastSampleMs_ > timeoutMs;   // unsigned: survives millis() wraparound
}
