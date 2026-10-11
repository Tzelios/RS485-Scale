#include <unity.h>
#include <scale_logic.h>
#include <math.h>
#include <string.h>

void setUp() {}
void tearDown() {}

// ---------------- address register ----------------

void test_address_valid_write_is_applied() {
  uint16_t reg = 7;
  uint8_t address = 3;
  TEST_ASSERT_TRUE(applyAddressWrite(reg, address));
  TEST_ASSERT_EQUAL_UINT8(7, address);
  TEST_ASSERT_EQUAL_UINT16(7, reg);
}

void test_address_unchanged_register_is_noop() {
  uint16_t reg = 3;
  uint8_t address = 3;
  TEST_ASSERT_FALSE(applyAddressWrite(reg, address));
  TEST_ASSERT_EQUAL_UINT8(3, address);
}

void test_address_zero_is_rejected_and_reverted() {
  uint16_t reg = 0;
  uint8_t address = 3;
  TEST_ASSERT_FALSE(applyAddressWrite(reg, address));
  TEST_ASSERT_EQUAL_UINT8(3, address);
  TEST_ASSERT_EQUAL_UINT16(3, reg);
}

// 259 truncated to uint8_t is 3: must not be accepted as address 3.
void test_address_above_8_bits_is_rejected_not_truncated() {
  uint16_t reg = 259;
  uint8_t address = 7;
  TEST_ASSERT_FALSE(applyAddressWrite(reg, address));
  TEST_ASSERT_EQUAL_UINT8(7, address);
  TEST_ASSERT_EQUAL_UINT16(7, reg);
}

// The original bug: a value that truncates to the current address made every
// loop pass save to EEPROM and restart Modbus.
void test_address_bad_write_does_not_retrigger_on_next_loop() {
  uint16_t reg = 259;
  uint8_t address = 3;
  applyAddressWrite(reg, address);
  TEST_ASSERT_FALSE(applyAddressWrite(reg, address));
  TEST_ASSERT_FALSE(applyAddressWrite(reg, address));
}

// Modbus reserves 248-255.
void test_address_reserved_range_is_rejected() {
  uint16_t reg = 248;
  uint8_t address = 3;
  TEST_ASSERT_FALSE(applyAddressWrite(reg, address));
  TEST_ASSERT_EQUAL_UINT8(3, address);
  reg = 247;
  TEST_ASSERT_TRUE(applyAddressWrite(reg, address));
  TEST_ASSERT_EQUAL_UINT8(247, address);
}

// ---------------- baud register ----------------

void test_baud_valid_write_is_applied() {
  uint16_t reg = 2;
  uint8_t baud = 3;
  TEST_ASSERT_TRUE(applyBaudWrite(reg, baud));
  TEST_ASSERT_EQUAL_UINT8(2, baud);
  TEST_ASSERT_EQUAL_UINT16(2, reg);
}

void test_baud_out_of_range_is_rejected_and_reverted() {
  uint16_t reg = 4;
  uint8_t baud = 3;
  TEST_ASSERT_FALSE(applyBaudWrite(reg, baud));
  TEST_ASSERT_EQUAL_UINT8(3, baud);
  TEST_ASSERT_EQUAL_UINT16(3, reg);
}

void test_baud_above_8_bits_is_rejected_not_truncated() {
  uint16_t reg = 258;   // truncates to 2
  uint8_t baud = 3;
  TEST_ASSERT_FALSE(applyBaudWrite(reg, baud));
  TEST_ASSERT_EQUAL_UINT8(3, baud);
  TEST_ASSERT_EQUAL_UINT16(3, reg);
}

void test_baud_bad_write_does_not_retrigger_on_next_loop() {
  uint16_t reg = 259;   // truncates to 3
  uint8_t baud = 3;
  applyBaudWrite(reg, baud);
  TEST_ASSERT_FALSE(applyBaudWrite(reg, baud));
  TEST_ASSERT_FALSE(applyBaudWrite(reg, baud));
}

// ---------------- load cell sampling (#2, #3) ----------------

struct FakeLoadCell : LoadCell
{
  bool ready = true;
  long value = 0;
  int reads = 0;
  bool isReady() override { return ready; }
  long read() override { reads++; return value; }
};

void test_sampler_never_reads_when_cell_not_ready() {
  FakeLoadCell cell;
  cell.ready = false;
  Sampler s;
  s.begin(0);
  long avg = 42;
  for (int i = 0; i < 100; i++)
    TEST_ASSERT_FALSE(s.poll(cell, 3, i, avg));
  TEST_ASSERT_EQUAL_INT(0, cell.reads);
  TEST_ASSERT_EQUAL_INT32(42, avg);
}

// Non-blocking: one poll takes at most one sample, however many are wanted.
void test_sampler_takes_one_sample_per_poll() {
  FakeLoadCell cell;
  Sampler s;
  s.begin(0);
  long avg;
  TEST_ASSERT_FALSE(s.poll(cell, 30, 0, avg));
  TEST_ASSERT_EQUAL_INT(1, cell.reads);
}

void test_sampler_returns_block_average() {
  FakeLoadCell cell;
  Sampler s;
  s.begin(0);
  long avg = 0;
  cell.value = 10; TEST_ASSERT_FALSE(s.poll(cell, 3, 0, avg));
  cell.value = 20; TEST_ASSERT_FALSE(s.poll(cell, 3, 1, avg));
  cell.value = 30; TEST_ASSERT_TRUE(s.poll(cell, 3, 2, avg));
  TEST_ASSERT_EQUAL_INT32(20, avg);
  // next block starts empty
  cell.value = 100; TEST_ASSERT_FALSE(s.poll(cell, 3, 3, avg));
  TEST_ASSERT_FALSE(s.poll(cell, 3, 4, avg));
  TEST_ASSERT_TRUE(s.poll(cell, 3, 5, avg));
  TEST_ASSERT_EQUAL_INT32(100, avg);
}

void test_sampler_restart_drops_partial_block() {
  FakeLoadCell cell;
  Sampler s;
  s.begin(0);
  long avg = 0;
  cell.value = 1000;
  s.poll(cell, 2, 0, avg);
  s.restart();
  cell.value = 10;
  TEST_ASSERT_FALSE(s.poll(cell, 2, 1, avg));
  TEST_ASSERT_TRUE(s.poll(cell, 2, 2, avg));
  TEST_ASSERT_EQUAL_INT32(10, avg);
}

void test_sampler_zero_count_is_treated_as_one() {
  FakeLoadCell cell;
  cell.value = 7;
  Sampler s;
  s.begin(0);
  long avg = 0;
  TEST_ASSERT_TRUE(s.poll(cell, 0, 0, avg));
  TEST_ASSERT_EQUAL_INT32(7, avg);
}

void test_sampler_faults_only_after_timeout_without_samples() {
  FakeLoadCell cell;
  Sampler s;
  s.begin(1000);
  TEST_ASSERT_FALSE(s.faulted(1500, 500));
  TEST_ASSERT_TRUE(s.faulted(1501, 500));
  long avg;
  s.poll(cell, 30, 1600, avg);          // a sample arrives
  TEST_ASSERT_FALSE(s.faulted(1700, 500));
}

void test_sampler_fault_survives_millis_wraparound() {
  FakeLoadCell cell;
  Sampler s;
  s.begin(0xFFFFFF00UL);
  TEST_ASSERT_FALSE(s.faulted(0x10, 500));   // 272 ms elapsed
  TEST_ASSERT_TRUE(s.faulted(0x200, 500));   // 768 ms elapsed
}

void test_raw_to_units_matches_hx711_get_units() {
  // HX711::get_units = (read_average - OFFSET) / SCALE
  TEST_ASSERT_EQUAL_FLOAT(2.5f, rawToUnits(1500, 1000, 200.0f));
  TEST_ASSERT_EQUAL_FLOAT(-1.0f, rawToUnits(800, 1000, 200.0f));
}

// ---------------- calibration (#4) ----------------

void test_calibration_computes_factor() {
  float f = 0;
  TEST_ASSERT_TRUE(computeScaleFactor(21000, 1000, 4, f));
  TEST_ASSERT_EQUAL_FLOAT(5000.0f, f);
}

void test_calibration_accepts_reversed_load_cell() {
  float f = 0;
  TEST_ASSERT_TRUE(computeScaleFactor(-19000, 1000, 4, f));
  TEST_ASSERT_EQUAL_FLOAT(-5000.0f, f);
}

void test_calibration_rejects_reading_equal_to_offset() {
  float f = 123.0f;
  TEST_ASSERT_FALSE(computeScaleFactor(1000, 1000, 4, f));
  TEST_ASSERT_EQUAL_FLOAT(123.0f, f);
}

void test_calibration_rejects_reading_within_noise_of_offset() {
  float f = 123.0f;
  TEST_ASSERT_FALSE(computeScaleFactor(1000 + MIN_CAL_COUNTS - 1, 1000, 4, f));
  TEST_ASSERT_FALSE(computeScaleFactor(1000 - MIN_CAL_COUNTS + 1, 1000, 4, f));
  TEST_ASSERT_EQUAL_FLOAT(123.0f, f);
  TEST_ASSERT_TRUE(computeScaleFactor(1000 + MIN_CAL_COUNTS, 1000, 4, f));
}

void test_calibration_rejects_zero_known_weight() {
  float f = 123.0f;
  TEST_ASSERT_FALSE(computeScaleFactor(21000, 1000, 0, f));
  TEST_ASSERT_EQUAL_FLOAT(123.0f, f);
}

// ---------------- weight register (#5) ----------------

void test_weight_register_in_range() {
  bool over = true;
  TEST_ASSERT_EQUAL_INT16(123, weightToRegister(12.34f, over));
  TEST_ASSERT_FALSE(over);
  TEST_ASSERT_EQUAL_INT16(-50, weightToRegister(-5.0f, over));
  TEST_ASSERT_FALSE(over);
  TEST_ASSERT_EQUAL_INT16(30000, weightToRegister(3000.0f, over));
  TEST_ASSERT_FALSE(over);
}

void test_weight_register_clamps_high() {
  bool over = false;
  TEST_ASSERT_EQUAL_INT16(32767, weightToRegister(5000.0f, over));
  TEST_ASSERT_TRUE(over);
}

void test_weight_register_clamps_low() {
  bool over = false;
  TEST_ASSERT_EQUAL_INT16(-32768, weightToRegister(-5000.0f, over));
  TEST_ASSERT_TRUE(over);
}

void test_weight_register_handles_non_numbers() {
  bool over = false;
  TEST_ASSERT_EQUAL_INT16(0, weightToRegister(NAN, over));
  TEST_ASSERT_TRUE(over);
  over = false;
  TEST_ASSERT_EQUAL_INT16(32767, weightToRegister(INFINITY, over));
  TEST_ASSERT_TRUE(over);
}

// ---------------- stored config (#6) ----------------

void test_config_default_is_valid() {
  TEST_ASSERT_TRUE(isConfigValid(defaultConfig()));
}

void test_config_erased_eeprom_is_invalid() {
  Config c;
  memset(&c, 0xFF, sizeof c);
  TEST_ASSERT_FALSE(isConfigValid(c));
}

void test_config_rejects_bad_address() {
  Config c = defaultConfig();
  c.address = 0;   TEST_ASSERT_FALSE(isConfigValid(c));
  c.address = 248; TEST_ASSERT_FALSE(isConfigValid(c));
  c.address = 247; TEST_ASSERT_TRUE(isConfigValid(c));
}

void test_config_rejects_bad_baud() {
  Config c = defaultConfig();
  c.baudCode = 0; TEST_ASSERT_FALSE(isConfigValid(c));
  c.baudCode = 4; TEST_ASSERT_FALSE(isConfigValid(c));
}

void test_config_rejects_bad_scale_factor() {
  Config c = defaultConfig();
  c.scaleFactor = 0;        TEST_ASSERT_FALSE(isConfigValid(c));
  c.scaleFactor = NAN;      TEST_ASSERT_FALSE(isConfigValid(c));
  c.scaleFactor = INFINITY; TEST_ASSERT_FALSE(isConfigValid(c));
}

void test_config_rejects_bad_average_count() {
  Config c = defaultConfig();
  c.averageCount = 0;   TEST_ASSERT_FALSE(isConfigValid(c));
  c.averageCount = 256; TEST_ASSERT_FALSE(isConfigValid(c));
  c.averageCount = 255; TEST_ASSERT_TRUE(isConfigValid(c));
}

void test_config_rejects_offset_outside_24_bits() {
  Config c = defaultConfig();
  c.offset = RAW_MAX + 1; TEST_ASSERT_FALSE(isConfigValid(c));
  c.offset = RAW_MIN - 1; TEST_ASSERT_FALSE(isConfigValid(c));
  c.offset = RAW_MIN;     TEST_ASSERT_TRUE(isConfigValid(c));
}

// ---------------- command register (#7) ----------------

void test_command_known_commands_are_accepted() {
  TEST_ASSERT_EQUAL_UINT16(CMD_TARE, acceptCommand(CMD_TARE, 0));
  TEST_ASSERT_EQUAL_UINT16(CMD_CALIBRATE, acceptCommand(CMD_CALIBRATE, 4));
}

void test_command_calibrate_without_known_weight_is_dropped() {
  TEST_ASSERT_EQUAL_UINT16(CMD_NONE, acceptCommand(CMD_CALIBRATE, 0));
}

void test_command_unknown_values_are_dropped() {
  TEST_ASSERT_EQUAL_UINT16(CMD_NONE, acceptCommand(3, 4));
  TEST_ASSERT_EQUAL_UINT16(CMD_NONE, acceptCommand(0xFFFF, 4));
  TEST_ASSERT_EQUAL_UINT16(CMD_NONE, acceptCommand(CMD_NONE, 4));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_address_valid_write_is_applied);
  RUN_TEST(test_address_unchanged_register_is_noop);
  RUN_TEST(test_address_zero_is_rejected_and_reverted);
  RUN_TEST(test_address_above_8_bits_is_rejected_not_truncated);
  RUN_TEST(test_address_bad_write_does_not_retrigger_on_next_loop);
  RUN_TEST(test_address_reserved_range_is_rejected);
  RUN_TEST(test_baud_valid_write_is_applied);
  RUN_TEST(test_baud_out_of_range_is_rejected_and_reverted);
  RUN_TEST(test_baud_above_8_bits_is_rejected_not_truncated);
  RUN_TEST(test_baud_bad_write_does_not_retrigger_on_next_loop);
  RUN_TEST(test_sampler_never_reads_when_cell_not_ready);
  RUN_TEST(test_sampler_takes_one_sample_per_poll);
  RUN_TEST(test_sampler_returns_block_average);
  RUN_TEST(test_sampler_restart_drops_partial_block);
  RUN_TEST(test_sampler_zero_count_is_treated_as_one);
  RUN_TEST(test_sampler_faults_only_after_timeout_without_samples);
  RUN_TEST(test_sampler_fault_survives_millis_wraparound);
  RUN_TEST(test_raw_to_units_matches_hx711_get_units);
  RUN_TEST(test_calibration_computes_factor);
  RUN_TEST(test_calibration_accepts_reversed_load_cell);
  RUN_TEST(test_calibration_rejects_reading_equal_to_offset);
  RUN_TEST(test_calibration_rejects_reading_within_noise_of_offset);
  RUN_TEST(test_calibration_rejects_zero_known_weight);
  RUN_TEST(test_weight_register_in_range);
  RUN_TEST(test_weight_register_clamps_high);
  RUN_TEST(test_weight_register_clamps_low);
  RUN_TEST(test_weight_register_handles_non_numbers);
  RUN_TEST(test_config_default_is_valid);
  RUN_TEST(test_config_erased_eeprom_is_invalid);
  RUN_TEST(test_config_rejects_bad_address);
  RUN_TEST(test_config_rejects_bad_baud);
  RUN_TEST(test_config_rejects_bad_scale_factor);
  RUN_TEST(test_config_rejects_bad_average_count);
  RUN_TEST(test_config_rejects_offset_outside_24_bits);
  RUN_TEST(test_command_known_commands_are_accepted);
  RUN_TEST(test_command_calibrate_without_known_weight_is_dropped);
  RUN_TEST(test_command_unknown_values_are_dropped);
  return UNITY_END();
}
