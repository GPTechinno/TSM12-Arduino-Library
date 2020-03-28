/*!
 *  @file TSM12.cpp
 */

#include "TSM12.h"
#include <Wire.h>

// REGISTER ADDRESSES
#define TSM12_SENSITIVITY 0x02 /**< First Sensitivity Control Register */
#define TSM12_CTRL_1      0x08 /**< General Control Register 1 */
#define TSM12_CTRL_2      0x09 /**< General Control Register 2 */
#define TSM12_REF_RST     0x0A /**< First Channel Reference Reset Control Register */
#define TSM12_CH_HOLD     0x0C /**< First Channel Sensing Control Register */
#define TSM12_CAL_HOLD    0x0E /**< First Channel Calibration Control Register */
#define TSM12_OUTPUT      0x10 /**< First Output Register */

// TSM12_SENSITIVITY bits
#define TSM12_SENSITIVITY_OFFSET_M  0  /**< Middle Sensitivity bits are at position 0 */
#define TSM12_SENSITIVITY_OFFSET_HL 3  /**< High/Low Sensitivity bit is at position 3 */
#define TSM12_SENSITIVITY_OFFSET_C  4  /**< Next Channel Sensitivity bits are at position 4 */
// TSM12_CTRL_1 bits
#define TSM12_CTRL1_MS_AUTO 0x00    /**< Mode Selection : auto alternate (fast/slow) mode */
#define TSM12_CTRL1_MS_FAST 0x80    /**< Mode Selection : fast mode */
#define TSM12_CTRL1_FTC_19  0x00    /**< First Touch Control : 19 * 16^4 * 1-Period (ms) */
#define TSM12_CTRL1_FTC_37  0x20    /**< First Touch Control : 37 * 16^4 * 1-Period (ms) */
#define TSM12_CTRL1_FTC_56  0x40    /**< First Touch Control : 56 * 16^4 * 1-Period (ms) */
#define TSM12_CTRL1_FTC_74  0x60    /**< First Touch Control : 74 * 16^4 * 1-Period (ms) */
#define TSM12_CTRL1_ILC_MH  0x00    /**< Interrupt Level Control : Interrupt is on middle or high output */
#define TSM12_CTRL1_ILC_LMH 0x08    /**< Interrupt Level Control : Interrupt is on low or middle or high output */
#define TSM12_CTRL1_ILC_MH2 0x10    /**< Interrupt Level Control : Interrupt is on middle or high output */
#define TSM12_CTRL1_ILC_H   0x18    /**< Interrupt Level Control : Interrupt is on high output */
#define TSM12_CTRL1_RTC_MAX 0x08    /**< Response Time Control : Response period = RTC[2:0] + 2 */
// TSM12_CTRL_2 bits
#define TSM12_CTRL2_SRST_DIS  0x00  /**< Software Reset : Disable */
#define TSM12_CTRL2_SRST_ENA  0x08  /**< Software Reset : Enable */
#define TSM12_CTRL2_SLEEP_DIS 0x00  /**< Sleep Mode : Disable */
#define TSM12_CTRL2_SLEEP_ENA 0x04  /**< Sleep Mode : Enable */
#define TSM12_CTRL2_INIT      0x03  /**< These bits must be written during a system initialize phase. */

/*!
 *  @brief  Instantiates a new TSM12 Capacitive Touch Sensor chip with the I2C address on a
 * TwoWire interface
 *  @param  i2c_en_pin The Arduino Pin connected to TSM12 I2C_EN signal
 */
TSM12::TSM12(const uint8_t i2c_en_pin)
    : _i2c_en_pin(i2c_en_pin), _i2c_addr(TSM12_I2C_ADDR_IDSEL_GND), _i2c(&Wire) {}

/*!
 *  @brief  Instantiates a new TSM12 Capacitive Touch Sensor chip with the I2C address on a
 * TwoWire interface
 *  @param  i2c_en_pin The Arduino Pin connected to TSM12 I2C_EN signal
 *  @param  addr The 7-bit I2C address to locate this chip, default is TSM12_I2C_ADDR_IDSEL_GND
 */
TSM12::TSM12(const uint8_t i2c_en_pin, const uint8_t addr)
    : _i2c_en_pin(i2c_en_pin), _i2c_addr(addr), _i2c(&Wire) {}

/*!
 *  @brief  Instantiates a new TSM12 Capacitive Touch Sensor chip with the I2C address on a
 * TwoWire interface
 *  @param  addr The 7-bit I2C address to locate this chip, default is TSM12_I2C_ADDR_IDSEL_GND
 *  @param  i2c  A reference to a 'TwoWire' object that we'll use to communicate
 *  with
 */
TSM12::TSM12(const uint8_t i2c_en_pin, const uint8_t addr, TwoWire &i2c)
    : _i2c_en_pin(i2c_en_pin), _i2c_addr(addr), _i2c(&i2c) {}

/*!
 *  @brief  Setup I2C interface and hardware init
 */
void TSM12::begin() {
  begin(0x0000U); // Use all 12 channels
}

/*!
 *  @brief  Setup I2C interface and hardware init
 *  @param  channels_mask Channel to be used, by default ALL
 */
void TSM12::begin(uint16_t channels_mask) {
  _i2c->begin();

  pinMode(_i2c_en_pin, OUTPUT);
  digitalWrite(_i2c_en_pin, HIGH);
  delay(100);

  enableCommunication();
  delay(75);
  write8(TSM12_CTRL_2, TSM12_CTRL2_SRST_ENA | TSM12_CTRL2_SLEEP_ENA | TSM12_CTRL2_INIT);
  delay(10);
  // setAllSensitivity(TSM12_SENS_M_070, TSM12_SENS_HL_20);
  uint8_t sens = (TSM12_SENS_M_070 << TSM12_SENSITIVITY_OFFSET_M) | (TSM12_SENS_HL_20 << TSM12_SENSITIVITY_OFFSET_HL);
  sens |= (sens << TSM12_SENSITIVITY_OFFSET_C);
  for (uint8_t i = 0U; i < (TSM12_CHANNEL_COUNT_MAX / 2U); i++) {
    write8(TSM12_SENSITIVITY + i, sens);
  }
  write8(TSM12_CTRL_1, TSM12_CTRL1_MS_AUTO | TSM12_CTRL1_FTC_37 | TSM12_CTRL1_ILC_H | 0x01U);
  write16(TSM12_REF_RST, channels_mask);
  write16(TSM12_CH_HOLD, channels_mask);
  write16(TSM12_CAL_HOLD, channels_mask);
  write8(TSM12_CTRL_2, TSM12_CTRL2_INIT);
  disableCommunication();
}

/*!
 *  @brief  Set Sensitivity for a specific channel
 *  @param  channel Selected Channel
 *  @param  m Middle Sensitivity
 *  @param  hl High Low Sensitivity
 *  @return status
 */
uint8_t TSM12::setChannelSensitivity(uint8_t channel, E_TSM12_SensitivityM_t m, E_TSM12_SensitivityHL_t hl) {
  uint8_t ret = TSM12_STATUS_OK;

  // Check Arguments
  if ((m >= TSM12_SENS_M_MAX) &&
      (hl >= TSM12_SENS_HL_MAX) &&
      (channel >= TSM12_CHANNEL_COUNT_MAX)
      ) {
    return TSM12_STATUS_ERR_INVALID;
  }

  uint8_t sens = (m << TSM12_SENSITIVITY_OFFSET_M) + (hl << TSM12_SENSITIVITY_OFFSET_HL);
  if (channel % 2U == 0U) {
    sens = sens << TSM12_SENSITIVITY_OFFSET_C;
  }

  enableCommunication();
  ret = write8(TSM12_SENSITIVITY + (uint8_t)(channel / 2U), sens);
  disableCommunication();
  return ret;
}

/*!
 *  @brief  Set Sensitivity for all channels
 *  @param  m Middle Sensitivity
 *  @param  hl High Low Sensitivity
 *  @return status
 */
uint8_t TSM12::setAllSensitivity(E_TSM12_SensitivityM_t m, E_TSM12_SensitivityHL_t hl) {
  uint8_t ret = TSM12_STATUS_OK;

  // Check Arguments
  if ((m >= TSM12_SENS_M_MAX) &&
      (hl >= TSM12_SENS_HL_MAX)
      ) {
    return TSM12_STATUS_ERR_INVALID;
  }

  for (uint8_t i = 0U; i < TSM12_CHANNEL_COUNT_MAX; i++) {
    ret |= setChannelSensitivity(i, m, hl);
  }

  return ret;
}

/*!
 *  @brief  Reset some channel references
 *  @param  channels_mask Channel mask to choose which channel reference will be reseted
 *  @return status
 */
uint8_t TSM12::resetReference(uint16_t channels_mask) {
  uint8_t ret = TSM12_STATUS_OK;

  channels_mask &= 0x0FFFU; // TSM12 has only 12 channels

  enableCommunication();
  ret = write16(TSM12_REF_RST, channels_mask);
  disableCommunication();
  return ret;
}

/*!
 *  @brief  Reset some channel references
 *  @param  channels_mask Channel mask to choose which channel will be holded
 *  @return status
 */
uint8_t TSM12::holdChannel(uint16_t channels_mask) {
  uint8_t ret = TSM12_STATUS_OK;

  channels_mask &= 0x0FFFU; // TSM12 has only 12 channels

  enableCommunication();
  ret = write16(TSM12_CH_HOLD, channels_mask);
  disableCommunication();
  return ret;
}

/*!
 *  @brief  Reset some channel references
 *  @param  channels_mask Channel mask to choose which channel calibration will be holded
 *  @return status
 */
uint8_t TSM12::holdCalibration(uint16_t channels_mask) {
  uint8_t ret = TSM12_STATUS_OK;

  channels_mask &= 0x0FFFU; // TSM12 has only 12 channels

  enableCommunication();
  ret = write16(TSM12_CAL_HOLD, channels_mask);
  disableCommunication();
  return ret;
}

/*!
 *  @brief  Gets all channel outputs
 *  @param  poutputs Array of all Channels Outputs, will be filled with actual outputs
 *  @return status
 */
uint8_t TSM12::getAllChannelOutputs(uint8_t *pdata, size_t datalen) {
  uint8_t ret = TSM12_STATUS_OK;

  if (datalen != 3U) {
    return TSM12_STATUS_ERR_INVALID;
  }
  enableCommunication();
  for (uint8_t i = 0U; i < 3U; i++)
  {
    ret |= read8(TSM12_OUTPUT + i, &pdata[i]);
  }
  disableCommunication();
  return ret;
}

/******************* Low level interface */
void TSM12::enableCommunication() {
  digitalWrite(_i2c_en_pin, LOW);
}

void TSM12::disableCommunication() {
  digitalWrite(_i2c_en_pin, HIGH);
}

uint8_t TSM12::read8(uint8_t addr, uint8_t *pdata) {
  uint8_t ret = TSM12_STATUS_OK;
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(addr);
  ret = _i2c->endTransmission();
  _i2c->requestFrom((uint8_t)_i2c_addr, (uint8_t)1);
  *pdata = _i2c->read();
  return ret;
}

uint8_t TSM12::write8(uint8_t addr, uint8_t data) {
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(addr);
  _i2c->write(data);
  return _i2c->endTransmission();
}

uint8_t TSM12::write16(uint8_t addr, uint16_t data) {
  _i2c->beginTransmission(_i2c_addr);
  _i2c->write(addr);
  _i2c->write((uint8_t)(data & 0xFFU));
  _i2c->write((uint8_t)((data >> 8U) & 0xFFU));
  return _i2c->endTransmission();
}
