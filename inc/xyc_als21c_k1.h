/*!
 *
 * 	I2C Driver for NEWOPT XYC_ALS21C_K1 ambient light sensor
 *
 * 	This is a library for the NEWOPT XYC_ALS21C_K1 breakout:
 * 	http://oshwlab.com/koendv/xyc_als21c_k1
 *
 */

#ifndef _XYC_ALS21C_K1_H
#define _XYC_ALS21C_K1_H

#ifdef __cplusplus

#include <cstdint>

namespace als21c {
#else
#include <stdint.h>
#include <stdbool.h>
#endif

/* I2C device address */
#define ALS21C_I2C_ADDR 0x38

/* product number register */
#define ALS21C_PRODUCT_ID 0x1011 /* xyc-als21c-k1 */

/*!
   read_als() and read_lux() values that indicate an error condition:
   ALS21C_ERR_SATURATION: error in the analog part (amplifier, comparator)
   ALS21C_ERR_OVERFLOW: error in the digital part (counter)
   ALS21C_ERR_NOT_READY: reading too soon (sensor still counting)
   Error numbers are negative.
*/
#define ALS21C_ERR_SATURATION -1
#define ALS21C_ERR_OVERFLOW -2
#define ALS21C_ERR_NOT_READY -3

/*! list of ALS registers */
enum {
  ALS21C_REG_SYSM_CTRL = 0x00,
  ALS21C_REG_INT_CTRL = 0x01,
  ALS21C_REG_INT_FLAG = 0x02,
  ALS21C_REG_WAIT_TIME = 0x03,
  ALS21C_REG_ALS_GAIN = 0x04,
  ALS21C_REG_ALS_TIME = 0x05,
  ALS21C_REG_PERSISTENCE = 0x0B,
  ALS21C_REG_ALS_THRES_L = 0x0C,
  ALS21C_REG_ALS_THRES_H = 0x0E,
  ALS21C_REG_DATA_STATUS = 0x17,
  ALS21C_REG_ALS_DATA = 0x1E,
  ALS21C_REG_PROD_ID = 0xBC,
};

/*! PGA_ALS gain values */
typedef enum {
  ALS21C_GAIN_1X = 0x01,
  ALS21C_GAIN_4X = 0x02,
  ALS21C_GAIN_16X = 0x04,
  ALS21C_GAIN_64X = 0x08,
  ALS21C_GAIN_256X = 0x10,
} als21c_gain_t;

/*! ALS integration time units. 1 unit = 1.171 milliseconds */
typedef enum {
  ALS21C_INT_TIME_1T = 0x00,
  ALS21C_INT_TIME_4T = 0x01,
  ALS21C_INT_TIME_16T = 0x02,
  ALS21C_INT_TIME_64T = 0x03,
} als21c_int_time_t;

/*! ALS wait time units. 1 unit = 8 milliseconds */
typedef enum {
  ALS21C_WAIT_TIME_1T = 0x00, /* 8 milliseconds */
  ALS21C_WAIT_TIME_2T = 0x01, /* 16 milliseconds */
  ALS21C_WAIT_TIME_4T = 0x02, /* 32 milliseconds */
  ALS21C_WAIT_TIME_8T = 0x03, /* 64 milliseconds */
} als21c_wait_time_t;

bool als21c_begin();
void als21c_end();
void als21c_reset();
void als21c_enable(bool onoff);
void als21c_enable_once(bool onoff);
void als21c_set_gain(uint8_t pdsel, als21c_gain_t pdals);
void als21c_set_gain_value(uint32_t gain);
uint32_t als21c_get_gain_value(void);
void als21c_set_integration(als21c_int_time_t itime, uint8_t icount);
void als21c_set_integration_time(uint32_t count);
uint32_t als21c_get_integration_time(void);
void als21c_set_integration_time_millisec(uint32_t millisec);
uint32_t als21c_get_integration_time_millisec(void);
void als21c_set_wait(als21c_wait_time_t unit, uint8_t count);
void als21c_set_wait_time_millisec(uint16_t millisec);
uint32_t als21c_get_wait_time_millisec(void);
uint32_t als21c_get_delay_millisec();
int32_t als21c_get_max_count(void);
void als21c_increase_gain(void);
void als21c_decrease_gain(void);
int32_t als21c_read_als(void);
int32_t als21c_read_lux(void);
void als21c_set_auto_lux(bool onoff);
void als21c_enable_interrupt(bool onoff);
void als21c_enable_als_sync(bool onoff);
bool als21c_interrupt_status(void);
void als21c_clear_interrupt(void);
void als21c_set_persistence(uint8_t pers);
void als21c_set_low_threshold(uint16_t value);
void als21c_set_high_threshold(uint16_t value);
uint16_t als21c_get_product_id(void);

/* low-level register access */
void als21c_set_reg_sysm_ctrl(void);
void als21c_set_reg_int_ctrl(void);
void als21c_set_reg_int_flag(void);
void als21c_get_reg_int_flag(void);
void als21c_set_reg_wait_time(void);
void als21c_set_reg_als_gain(void);
void als21c_set_reg_als_time(void);
void als21c_set_reg_persistence(void);
void als21c_get_reg_data_status(void);
/* os-dependent */
void als21c_dump_regs(void);
void als21c_i2c_write8(const uint8_t reg, const uint8_t data);
void als21c_i2c_write16(const uint8_t reg, const uint16_t data);
uint8_t als21c_i2c_read8(const uint8_t reg);
uint16_t als21c_i2c_read16(const uint8_t reg);

typedef struct {
  /* sysm_ctrl register */
  uint8_t swrst : 1;
  uint8_t en_wait : 1;
  uint8_t en_frst : 1;
  uint8_t en_once : 1;
  uint8_t en_als : 1;
  /* int_ctrl register */
  uint8_t als_sync : 1;
  uint8_t en_aint : 1;
  /* int_flag register */
  uint8_t int_por : 1;
  uint8_t data_flag : 1;
  uint8_t int_als : 1;
  /* wait_time register */
  uint8_t wtime_unit : 2;
  uint8_t wtime : 6;
  /* als_gain register */
  uint8_t pd_sel : 1;
  uint8_t pga_als : 5;
  /* als_time register */
  uint8_t als_conv : 4;
  uint8_t int_time : 2;
  /* persistence register */
  uint8_t int_src : 1;
  uint8_t prs_als : 4;
  /* data status register */
  uint8_t data_ready : 1;
  uint8_t saturation_als : 1;
  uint8_t saturation_comp : 1;

  /* automatically adjust gain and integration time */
  bool auto_lux;
} als21c_data_s;

extern als21c_data_s als21c_data;

#ifdef __cplusplus
} /* namespace als21c */
#endif

#endif
