/*!
 *  @file TSM12.h
 */
#ifndef _TSM12_H
#define _TSM12_H

#include <Arduino.h>
#include <Wire.h>

#define TSM12_CHANNEL_COUNT_MAX 12 /**< TSM12 has 12 Channels */

#define TSM12_I2C_ADDR_IDSEL_GND 0x68 /**< TSM12 I2C Slave Address when ID_SEL = GND (0xD0 >> 1) */
#define TSM12_I2C_ADDR_IDSEL_VCC 0x78 /**< TSM12 I2C Slave Address when ID_SEL = VDD (0xF0 >> 1) */

#define TSM12_STATUS_OK 0            /**< Success */
#define TSM12_STATUS_ERR_TOO_LONG 1  /**< Data too long to fit in transmit buffer */
#define TSM12_STATUS_ERR_BAD_ADDR 2  /**< Received NACK on transmit of address */
#define TSM12_STATUS_ERR_BAD_DATA 3  /**< Received NACK on transmit of data */
#define TSM12_STATUS_ERR_OTHER    4  /**< Other error */
#define TSM12_STATUS_ERR_INVALID  10 /**< Error code when checking arguments failed */

typedef enum {
  TSM12_SENS_M_050 = 0U, /**< 0.50% */
  TSM12_SENS_M_070 = 1U, /**< 0.70% */
  TSM12_SENS_M_090 = 2U, /**< 0.90% */
  TSM12_SENS_M_120 = 3U, /**< 1.20% */
  TSM12_SENS_M_150 = 4U, /**< 1.50% */
  TSM12_SENS_M_205 = 5U, /**< 2.05% */
  TSM12_SENS_M_255 = 6U, /**< 2.55% */
  TSM12_SENS_M_355 = 7U, /**< 3.55% */
  TSM12_SENS_M_MAX = 8U, /**< Number of Middle Sensitivity allowed - do not use as argument */
} E_TSM12_SensitivityM_t; /**< Middle Sensitivity */

typedef enum {
  TSM12_SENS_HL_20  = 0U, /**< High/Low Sensitivity are +/- 20% of Middle Sensitivity */
  TSM12_SENS_HL_30  = 1U, /**< High/Low Sensitivity are +/- 30% of Middle Sensitivity */
  TSM12_SENS_HL_MAX = 2U /**< Number of High Low Sensitivity allowed - do not use as argument */
} E_TSM12_SensitivityHL_t; /**< High Low Sensitivity */

typedef enum {
  TSM12_OUTPUT_NONE   = 0U, /**< No Output */
  TSM12_OUTPUT_LOW    = 1U, /**< No Output */
  TSM12_OUTPUT_MIDDLE = 2U, /**< No Output */
  TSM12_OUTPUT_HIGH   = 3U /**< No Output */
} E_TSM12_Output_t; /**< Channel Output */

/*!
 *  @brief  Class that stores state and functions for interacting with TSM12
 * Capacitive Touch Sensor chip
 */
class TSM12 {
public:
  TSM12(const uint8_t i2c_en_pin);
  TSM12(const uint8_t i2c_en_pin, const uint8_t addr);
  TSM12(const uint8_t i2c_en_pin, const uint8_t addr, TwoWire &i2c);
  void begin();
  void begin(uint16_t channels_mask);
  uint8_t setAllSensitivity(E_TSM12_SensitivityM_t m, E_TSM12_SensitivityHL_t hl);
  uint8_t setChannelSensitivity(uint8_t channel, E_TSM12_SensitivityM_t m, E_TSM12_SensitivityHL_t hl);
  uint8_t resetReference(uint16_t channels_mask);
  uint8_t holdChannel(uint16_t channels_mask);
  uint8_t holdCalibration(uint16_t channels_mask);
  uint8_t getAllChannelOutputs(E_TSM12_Output_t *poutputs[TSM12_CHANNEL_COUNT_MAX]);

private:
  uint8_t _i2c_en_pin;
  uint8_t _i2c_addr;
  TwoWire *_i2c;

  void enableCommunication();
  void disableCommunication();
  uint8_t read8(uint8_t addr);
  uint8_t write8(uint8_t addr, uint8_t data);
  uint8_t write16(uint8_t addr, uint16_t data);
};

#endif
