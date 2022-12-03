// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <VL53L0X.h>
#include <I2CDev.h>
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#define I2C &hi2c1
extern I2C_HandleTypeDef hi2c1;
//extern UART_HandleTypeDef huart2;
//extern char msg[64];

void setup_VL53L0X(VL53L0X * lidar){
  lidar->address=0x29;//ADDRESS_DEFAULT;
  lidar->io_timeout=0;
  lidar->did_timeout=false;
}

void setAddress(VL53L0X * lidar, uint8_t new_addr)
{
  writeReg(lidar,I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
  lidar->address = new_addr;
}

uint8_t getAddress(VL53L0X * lidar)
{
	return lidar->address;
}

uint8_t getWhoAmI(VL53L0X * lidar)
{
	return readReg(lidar,IDENTIFICATION_MODEL_ID);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool init(VL53L0X * lidar,bool io_2v8)
{
  // VL53L0X_DataInit() begin

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    writeReg(lidar,VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      readReg(lidar,(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01)); // set bit 0
  }

  // "Set I2C standard mode"
  writeReg(lidar,0x88, 0x00);

  writeReg(lidar,0x80, 0x01);
  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x00);
  lidar->stop_variable = readReg(lidar,0x91);
  //snprintf(msg,sizeof(msg),"Stop variable is: 0x%X , should be 0x%X\r\n",lidar->stop_variable,readReg(lidar,0x91));
  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  writeReg(lidar,0x00, 0x01);
  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(lidar,MSRC_CONFIG_CONTROL, readReg(lidar,MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(lidar,0.25);

  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(lidar,&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  readMulti(lidar,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  //snprintf(msg,sizeof(msg),"Spads: 0x%X, 0x%X, 0x%X, 0x%X, 0x%X, 0x%X\r\n",ref_spad_map[0],ref_spad_map[1],ref_spad_map[2],ref_spad_map[3],ref_spad_map[4],ref_spad_map[5]);
  //HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(lidar,DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 7));
    }
    else if ((ref_spad_map[i / 8] >> (i % 7)) & 0x1)
    {
      spads_enabled++;
    }
  }

  writeMulti(lidar,GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x00);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x09, 0x00);
  writeReg(lidar,0x10, 0x00);
  writeReg(lidar,0x11, 0x00);

  writeReg(lidar,0x24, 0x01);
  writeReg(lidar,0x25, 0xFF);
  writeReg(lidar,0x75, 0x00);

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x4E, 0x2C);
  writeReg(lidar,0x48, 0x00);
  writeReg(lidar,0x30, 0x20);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x30, 0x09);
  writeReg(lidar,0x54, 0x00);
  writeReg(lidar,0x31, 0x04);
  writeReg(lidar,0x32, 0x03);
  writeReg(lidar,0x40, 0x83);
  writeReg(lidar,0x46, 0x25);
  writeReg(lidar,0x60, 0x00);
  writeReg(lidar,0x27, 0x00);
  writeReg(lidar,0x50, 0x06);
  writeReg(lidar,0x51, 0x00);
  writeReg(lidar,0x52, 0x96);
  writeReg(lidar,0x56, 0x08);
  writeReg(lidar,0x57, 0x30);
  writeReg(lidar,0x61, 0x00);
  writeReg(lidar,0x62, 0x00);
  writeReg(lidar,0x64, 0x00);
  writeReg(lidar,0x65, 0x00);
  writeReg(lidar,0x66, 0xA0);

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x22, 0x32);
  writeReg(lidar,0x47, 0x14);
  writeReg(lidar,0x49, 0xFF);
  writeReg(lidar,0x4A, 0x00);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x7A, 0x0A);
  writeReg(lidar,0x7B, 0x00);
  writeReg(lidar,0x78, 0x21);

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x23, 0x34);
  writeReg(lidar,0x42, 0x00);
  writeReg(lidar,0x44, 0xFF);
  writeReg(lidar,0x45, 0x26);
  writeReg(lidar,0x46, 0x05);
  writeReg(lidar,0x40, 0x40);
  writeReg(lidar,0x0E, 0x06);
  writeReg(lidar,0x20, 0x1A);
  writeReg(lidar,0x43, 0x40);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x34, 0x03);
  writeReg(lidar,0x35, 0x44);

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x31, 0x04);
  writeReg(lidar,0x4B, 0x09);
  writeReg(lidar,0x4C, 0x05);
  writeReg(lidar,0x4D, 0x04);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x44, 0x00);
  writeReg(lidar,0x45, 0x20);
  writeReg(lidar,0x47, 0x08);
  writeReg(lidar,0x48, 0x28);
  writeReg(lidar,0x67, 0x00);
  writeReg(lidar,0x70, 0x04);
  writeReg(lidar,0x71, 0x01);
  writeReg(lidar,0x72, 0xFE);
  writeReg(lidar,0x76, 0x00);
  writeReg(lidar,0x77, 0x00);

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x0D, 0x01);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x80, 0x01);
  writeReg(lidar,0x01, 0xF8);

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x8E, 0x01);
  writeReg(lidar,0x00, 0x01);
  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  writeReg(lidar,SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(lidar,GPIO_HV_MUX_ACTIVE_HIGH, readReg(lidar,GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(lidar,SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  lidar->measurement_timing_budget_us = getMeasurementTimingBudget(lidar);

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(lidar,lidar->measurement_timing_budget_us);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(lidar,0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(lidar,0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

// Write an 8-bit register
void writeReg(VL53L0X * lidar,uint8_t reg, uint8_t value)
{
	writeByte(I2C,lidar->address,reg,value);
	lidar->last_status = 0;
}

// Write a 16-bit register
void writeReg16Bit(VL53L0X * lidar,uint8_t reg, uint16_t value)
{
  writeWord(I2C,lidar->address,reg,value);
  lidar->last_status = 0;
}

// Write a 32-bit register
void writeReg32Bit(VL53L0X * lidar,uint8_t reg, uint32_t value)
{
  write32Bits(I2C,lidar->address,reg,value);
  lidar->last_status = 0;
}

// Read an 8-bit register
uint8_t readReg(VL53L0X * lidar,uint8_t reg)
{
  uint8_t value;
  readByte(I2C,lidar->address,reg,&value,I2CTIMEOUT);

  lidar->last_status = 0;
  return value;
}

// Read a 16-bit register
uint16_t readReg16Bit(VL53L0X * lidar,uint8_t reg)
{
  uint16_t value;
  readWord(I2C,lidar->address,reg,&value,I2CTIMEOUT);

  lidar->last_status = 0;
  return value;
}

// Read a 32-bit register
uint32_t readReg32Bit(VL53L0X * lidar,uint8_t reg)
{
  uint32_t value;
  read32Bits(I2C,lidar->address,reg,&value,I2CTIMEOUT);
  lidar->last_status = 0;
  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void writeMulti(VL53L0X * lidar,uint8_t reg, uint8_t * src, uint8_t count)
{
	writeBytes(I2C,lidar->address,reg,count,src);
	lidar->last_status = 0;
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void readMulti(VL53L0X * lidar,uint8_t reg, uint8_t * dst, uint8_t count)
{
	readBytes(I2C,lidar->address,reg,count,dst,I2CTIMEOUT);
	lidar->last_status = 0;
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
bool setSignalRateLimit(VL53L0X * lidar,float limit_Mcps)
{
  if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

  // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
  writeReg16Bit(lidar,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (int)limit_Mcps * (1 << 7));
  return true;
}

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(VL53L0X * lidar)
{
  return (float)readReg16Bit(lidar,FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(VL53L0X * lidar,uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(lidar,&enables);
  getSequenceStepTimeouts(lidar,&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(lidar,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    lidar->measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t getMeasurementTimingBudget(VL53L0X * lidar)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(lidar,&enables);
  getSequenceStepTimeouts(lidar,&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  lidar->measurement_timing_budget_us = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool setVcselPulsePeriod(VL53L0X * lidar,vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(lidar,&enables);
  getSequenceStepTimeouts(lidar,&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        writeReg(lidar,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        writeReg(lidar,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        writeReg(lidar,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        writeReg(lidar,PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    writeReg(lidar,PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    writeReg(lidar,PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16Bit(lidar,PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(lidar,MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(lidar,GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(lidar,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(lidar,0xFF, 0x01);
        writeReg(lidar,ALGO_PHASECAL_LIM, 0x30);
        writeReg(lidar,0xFF, 0x00);
        break;

      case 10:
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(lidar,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(lidar,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(lidar,0xFF, 0x01);
        writeReg(lidar,ALGO_PHASECAL_LIM, 0x20);
        writeReg(lidar,0xFF, 0x00);
        break;

      case 12:
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(lidar,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(lidar,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(lidar,0xFF, 0x01);
        writeReg(lidar,ALGO_PHASECAL_LIM, 0x20);
        writeReg(lidar,0xFF, 0x00);
        break;

      case 14:
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(lidar,FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(lidar,GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(lidar,ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(lidar,0xFF, 0x01);
        writeReg(lidar,ALGO_PHASECAL_LIM, 0x20);
        writeReg(lidar,0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    writeReg(lidar,FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(lidar,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(lidar,lidar->measurement_timing_budget_us);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = readReg(lidar,SYSTEM_SEQUENCE_CONFIG);
  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(lidar,0x0);
  writeReg(lidar,SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(VL53L0X * lidar,vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(lidar,PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(lidar,FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void startContinuous(VL53L0X * lidar,uint32_t period_ms)
{
  writeReg(lidar,0x80, 0x01);
  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x00);
  writeReg(lidar,0x91, lidar->stop_variable);
  writeReg(lidar,0x00, 0x01);
  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
    uint16_t osc_calibrate_val = readReg16Bit(lidar,OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    writeReg32Bit(lidar,SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    writeReg(lidar,SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
	//snprintf(msg,sizeof(msg),"Starting continuous...\r\n");
	//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 0xFFFF);
    writeReg(lidar,SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void stopContinuous(VL53L0X * lidar)
{
  writeReg(lidar,SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x00);
  writeReg(lidar,0x91, 0x00);
  writeReg(lidar,0x00, 0x01);
  writeReg(lidar,0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
uint16_t readRangeContinuousMillimeters(VL53L0X * lidar)
{
  startTimeout(lidar);
  while ((readReg(lidar,RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired(lidar))
    {
      lidar->did_timeout = true;
      return 65535;
    }
  }

  // assumptions: Linearity Corrective Gain is 1000 (default);
  // fractional ranging is not enabled
  uint16_t range = readReg16Bit(lidar,RESULT_RANGE_STATUS + 10);

  writeReg(lidar,SYSTEM_INTERRUPT_CLEAR, 0x01);

  return range;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
uint16_t readRangeSingleMillimeters(VL53L0X * lidar)
{
  writeReg(lidar,0x80, 0x01);
  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x00);
  writeReg(lidar,0x91, lidar->stop_variable);
  writeReg(lidar,0x00, 0x01);
  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x80, 0x00);

  writeReg(lidar,SYSRANGE_START, 0x01);

  // "Wait until start bit has been cleared"
  startTimeout(lidar);
  while (readReg(lidar,SYSRANGE_START) & 0x01)
  {
    if (checkTimeoutExpired(lidar))
    {
      lidar->did_timeout = true;
      return 65535;
    }
  }

  return readRangeContinuousMillimeters(lidar);
}

void setTimeout(VL53L0X * lidar,uint16_t timeout)
{
	lidar->io_timeout = timeout;
}

uint16_t getTimeout(VL53L0X * lidar)
{
	return lidar->io_timeout;
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool timeoutOccurred(VL53L0X * lidar)
{
  bool tmp = lidar->did_timeout;
  lidar->did_timeout = false;
  return tmp;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool getSpadInfo(VL53L0X * lidar,uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(lidar,0x80, 0x01);
  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x00);

  writeReg(lidar,0xFF, 0x06);
  writeReg(lidar,0x83, readReg(lidar,0x83) | 0x04);
  writeReg(lidar,0xFF, 0x07);
  writeReg(lidar,0x81, 0x01);

  writeReg(lidar,0x80, 0x01);

  writeReg(lidar,0x94, 0x6b);
  writeReg(lidar,0x83, 0x00);
  startTimeout(lidar);
  while (readReg(lidar,0x83) == 0x00)
  {
    if (checkTimeoutExpired(lidar)) { return false; }
  }
  writeReg(lidar,0x83, 0x01);
  tmp = readReg(lidar,0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(lidar,0x81, 0x00);
  writeReg(lidar,0xFF, 0x06);
  writeReg(lidar,0x83, readReg(lidar,0x83)  & ~0x04);
  writeReg(lidar,0xFF, 0x01);
  writeReg(lidar,0x00, 0x01);

  writeReg(lidar,0xFF, 0x00);
  writeReg(lidar,0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(VL53L0X * lidar,SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(lidar,SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(VL53L0X * lidar,SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(lidar,VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(lidar,MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(lidar,PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(lidar,VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(lidar,FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(VL53L0X * lidar,uint8_t vhv_init_byte)
{
  writeReg(lidar,SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout(lidar);
  
  
  while ((readReg(lidar,RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (checkTimeoutExpired(lidar)) { return false; }
  }

  writeReg(lidar,SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(lidar,SYSRANGE_START, 0x00);

  return true;
}


void startTimeout(VL53L0X * lidar)
{
	lidar->timeout_start_ms = HAL_GetTick();
}

// Check if timeout is enabled (set to nonzero value) and has expired
bool checkTimeoutExpired(VL53L0X * lidar)
{
	if (lidar->io_timeout > 0 && (HAL_GetTick() - lidar->timeout_start_ms) > lidar->io_timeout)
		{
			return true;
		}
	else
	{
		return false;
	}
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
uint8_t decodeVcselPeriod(uint8_t reg_val)
{
	return (((reg_val) + 1) << 1);
}

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
uint8_t encodeVcselPeriod(uint8_t period_pclks)
{
	return (((period_pclks) >> 1) - 1);
}

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
uint32_t calcMacroPeriod(uint8_t vcsel_period_pclks)
{
	return ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000);
}