/*!
 *
 * 	I2C Driver for NEWOPT XYC_ALS21C ambient light sensor
 *
 * 	This is a library for the NEWOPT XYC_ALS21C breakout:
 * 	http://oshwlab.com/koendv/xyc_als21c_k1
 *
 */

#include <xyc_als21c_k1.h>

#ifdef __cplusplus
#include <cstring>

#ifdef ARDUINO
#include <Arduino.h>
#endif

namespace als21c {
#else
#include <string.h>
#endif

als21c_data_s als21c_data;

#define ALS21C_USE_INT

#ifdef ALS21C_USE_INT

/*
 * lookup table for lux values.
 * lux = 478.233 * x^1 + 0.0416391 * x^3 + -1.18758e-06 * x^5
 * where x = count / (gain * itime * (als_conv + 1))
 *
 * The relationship between lux and ALS_DATA register value has been
 * obtained by curve-fitting VEML7700 and XYC-ALS21C-K1 measurements.
 * Note that this calibration is based on a specific setup.
 * Results may vary depending on temperature, sensor, and light source.
 * If you have access to more accurate calibration data,
 * please share your findings to improve this driver.
 */

static const uint32_t lux_table[] = {
  0, 478, 957, 1436, 1916, 2396, 2878, 3362,
  3847, 4334, 4824, 5316, 5810, 6308, 6809, 7313,
  7821, 8333, 8849, 9369, 9894, 10424, 10958, 11498,
  12044, 12595, 13152, 13715, 14284, 14860, 15442, 16032,
  16628, 17232, 17843, 18461, 19087, 19721, 20364, 21014,
  21673, 22340, 23016, 23700, 24393, 25096, 25807, 26528,
  27258, 27997, 28745, 29504, 30271, 31049, 31836, 32633,
  33440, 34256, 35082, 35919, 36765, 37620, 38486, 39362,
  40247, 41142, 42047, 42962, 43886, 44819, 45763, 46715,
  47677, 48647, 49627, 50616, 51613, 52619, 53633, 54656,
  55686, 56725, 57771, 58824, 59885, 60952, 62026, 63107,
  64193, 65285, 66383, 67486, 68594, 69707, 70823, 71943,
  73067, 74193, 75322, 76454, 77587, 78721, 79856, 80991,
  82126, 83260, 84393, 85524, 86653, 87779, 88901, 90019,
  91133, 92241, 93343, 94438, 95526, 96606, 97677, 98738,
  99790, 100830, 101858, 102873, 103876, 104863, 105836, 106793,
  107732, 108654, 109557, 110441
};

/* convert adc count to lux using integer */
int32_t als21c_count_to_lux(uint16_t count) {
  int32_t gain, integration_time;
  int32_t lux;

  gain = als21c_get_gain_value();
  integration_time = als21c_get_integration_time();

  /* linear interpolation in lookup table. integer math, suitable for small microcontroller */
  const uint32_t last_index = sizeof(lux_table) / sizeof(lux_table[0]) - 1;
  int32_t q = (256 * count) / (gain * integration_time); /* normalized counts, multiplied by 256 */
  int32_t x1 = q >> 8;
  int32_t delta_x = q & 0xff;

  if (x1 > last_index - 1) {
    x1 = last_index - 1;
    delta_x = 0;
  }

  /* lookup in table */
  int32_t y1 = lux_table[x1];
  int32_t y2 = lux_table[x1 + 1];
  /* interpolate */
  int32_t delta_y = ((y2 - y1) * delta_x) / 256;
  lux = y1 + delta_y;

  return lux;
}

#else

/* convert adc count to lux using float */
int32_t als21c_count_to_lux(uint16_t count) {
  /* datasheet: als21c measures up to 110000 lux, about x = 130.498 */
  const float max_x = 130.498;
  float gain, integration_time, x, x2, lux_f;
  int32_t lux_i;
  gain = als21c_get_gain_value();
  integration_time = als21c_get_integration_time();
  /* normalized count */
  x = (float)count / (gain * integration_time);
  if (x > max_x) x = max_x;
  /* lux = 478.233 * x^1 + 0.0416391 * x^3 + -1.18758e-06 * x^5 */
  x2 = x * x;
  lux_f = ((-1.18758e-06 * x2 + 0.0416391) * x2 + 478.233) * x;
  lux_i = lux_f + 0.5;
  return lux_i;
}

#endif

/*!
 * @brief  initializes ambient light sensor
 */
bool als21c_begin() {
  if (als21c_get_product_id() != ALS21C_PRODUCT_ID)
    return false;
  /* reset */
  als21c_reset();
  return true;
}

/*!
 * @brief  stop ambient light sensor measurement and interrupts
 */
void als21c_end() {
  als21c_i2c_write8(ALS21C_REG_SYSM_CTRL, 0x0); /* disable als */
  als21c_i2c_write8(ALS21C_REG_INT_CTRL, 0x0);  /* disable interrupts */
}

/*!
 * @brief  reset ALS sensor
 */
void als21c_reset() {
  /* clear */
  memset(&als21c_data, 0, sizeof(als21c_data));
  /* reset */
  als21c_data.swrst = 1;
  als21c_set_reg_sysm_ctrl();
  als21c_data.swrst = 0;
  /* default values after reset */
  als21c_data.en_aint = 0x1;
  als21c_data.pga_als = 0x1;
  als21c_data.int_time = 0x3;
  als21c_data.prs_als = 0x1;
}

/*!
 * @brief  switch ambient light sensor on/off
 * @param  onoff
 */
void als21c_enable(bool onoff) {
  als21c_data.en_als = onoff ? 0x1 : 0x0;
  als21c_set_reg_sysm_ctrl();
}

/*!
 * @brief  switch ambient light sensor on for a single measurement
 */
void als21c_enable_once(bool onoff) {
  als21c_data.en_once = onoff ? 0x1 : 0x0;
  als21c_set_reg_sysm_ctrl();
}

/*!
 * @brief  set the ambient light sensor gain
 * @param  pdsel new value for pd_sel
 * @param  pdals new value for pd_als
 */
void als21c_set_gain(uint8_t pdsel, als21c_gain_t pdals) {
  /* pd_sel == 1 for gain*2 */
  als21c_data.pd_sel = pdsel & 0x1;
  /* pd_als */
  als21c_data.pga_als = pdals;
  /* write gain */
  als21c_set_reg_als_gain();
}

/*!
 * @brief  set the ambient light sensor gain
 * @param  gain
 *         actual gain is power of two between 1 and 512
 */
void als21c_set_gain_value(uint32_t gain) {
  if (gain <= 1) als21c_set_gain(0, ALS21C_GAIN_1X);
  else if (gain <= 2) als21c_set_gain(1, ALS21C_GAIN_1X);
  else if (gain <= 4) als21c_set_gain(0, ALS21C_GAIN_4X);
  else if (gain <= 8) als21c_set_gain(1, ALS21C_GAIN_4X);
  else if (gain <= 16) als21c_set_gain(0, ALS21C_GAIN_16X);
  else if (gain <= 32) als21c_set_gain(1, ALS21C_GAIN_16X);
  else if (gain <= 64) als21c_set_gain(0, ALS21C_GAIN_64X);
  else if (gain <= 128) als21c_set_gain(1, ALS21C_GAIN_64X);
  else if (gain <= 256) als21c_set_gain(0, ALS21C_GAIN_256X);
  else als21c_set_gain(1, ALS21C_GAIN_256X);
  return;
}

/*!
 * @brief  returns the ambient light sensor gain
 * @return gain
 *         gain is power of two between 1 and 512
 */
uint32_t als21c_get_gain_value() {
  uint32_t gain_value;
  switch (als21c_data.pga_als) {
    case ALS21C_GAIN_1X: gain_value = 1; break;
    case ALS21C_GAIN_4X: gain_value = 4; break;
    case ALS21C_GAIN_16X: gain_value = 16; break;
    case ALS21C_GAIN_64X: gain_value = 64; break;
    case ALS21C_GAIN_256X: gain_value = 256; break;
    default: gain_value = 1; break; /* ought never to happen */
  }
  if (als21c_data.pd_sel) gain_value *= 2;
  return gain_value;
}

/*!
 * @brief  set the integration time for the ADC
 * @param  itime
 * @param  icount
 */
void als21c_set_integration(als21c_int_time_t itime, uint8_t icount) {
  als21c_data.int_time = itime;
  als21c_data.als_conv = icount;
  als21c_set_reg_als_time();
}

/*!
 * @brief  set the integration time for the ADC in units of 1.17ms
 * @param  count
 *         integration time in units of 1.17 milliseconds
 */
void als21c_set_integration_time(uint32_t count) {
  if (count == 0) als21c_set_integration(ALS21C_INT_TIME_1T, 0);
  else if (count <= 16) als21c_set_integration(ALS21C_INT_TIME_1T, count - 1);
  else if (count <= 4 * 16) als21c_set_integration(ALS21C_INT_TIME_4T, count / 4 - 1);
  else if (count <= 16 * 16) als21c_set_integration(ALS21C_INT_TIME_16T, count / 16 - 1);
  else if (count <= 64 * 16) als21c_set_integration(ALS21C_INT_TIME_64T, count / 64 - 1);
  else als21c_set_integration(ALS21C_INT_TIME_64T, 15); /* maximum value */
}

/*!
 * @brief  get the integration time for the ADC in units of 1.17ms
 * @param  none
 * @return integration time in units of 1.17 milliseconds
 */
uint32_t als21c_get_integration_time(void) {
  uint32_t itime;
  switch (als21c_data.int_time) {
    case ALS21C_INT_TIME_1T: itime = 1; break;
    case ALS21C_INT_TIME_4T: itime = 4; break;
    case ALS21C_INT_TIME_16T: itime = 16; break;
    case ALS21C_INT_TIME_64T: itime = 64; break;
    default: itime = 1; break; /* should never happen */
  }
  itime = itime * (als21c_data.als_conv + 1);
  return itime;
}

/*!
 * @brief  set the integration time for the ADC in milliseconds
 * @param  millisec
 *         integration time in milliseconds
 */
void als21c_set_integration_time_millisec(uint32_t millisec) {
  uint32_t count;
  count = (millisec * 416) / 487; /* 416/487 = 1.17ms */
  als21c_set_integration_time(count);
}

/*!
 * @brief  get the integration time for the ADC in milliseconds
 * @param  none
 * @return integration time in milliseconds
 */
uint32_t als21c_get_integration_time_millisec(void) {
  uint32_t count, millisec;
  count = als21c_get_integration_time();
  millisec = (count * 487) / 416; /* 416/487 = 1.17ms */
  return millisec;
}

/*!
 * @brief  get maximum value of als21c_read_als()
 * @param  none
 * @return maximum ALS count
 */
int32_t als21c_get_max_count(void) {
  int32_t max_count, integration_time;
  integration_time = als21c_get_integration_time();
  max_count = 1024 * integration_time - 1;
  if (max_count > 0xffff) max_count = 0xffff;
  return max_count;
}

/*!
 * @brief  set the wait time between two measurements
 * @param  millisec
 */
void als21c_set_wait(als21c_wait_time_t unit, uint8_t count) {
  als21c_data.wtime_unit = unit;
  als21c_data.wtime = count;
  als21c_set_reg_wait_time();
}

/*!
 * @brief  set the wait time between two measurements in milliseconds
 * @param  millisec
 *         wait time 0 switches waiting off
 */
void als21c_set_wait_time_millisec(uint16_t millisec) {

  if (millisec <= 8) als21c_set_wait(ALS21C_WAIT_TIME_1T, 0);
  else if (millisec <= 512) als21c_set_wait(ALS21C_WAIT_TIME_1T, millisec / 8 - 1);
  else if (millisec <= 1024) als21c_set_wait(ALS21C_WAIT_TIME_2T, millisec / 16 - 1);
  else if (millisec <= 2048) als21c_set_wait(ALS21C_WAIT_TIME_4T, millisec / 32 - 1);
  else if (millisec <= 4096) als21c_set_wait(ALS21C_WAIT_TIME_8T, millisec / 64 - 1);
  else als21c_set_wait(ALS21C_WAIT_TIME_8T, 0x3f); /* maximum value */

  /* disable wait if millisec == 0 */
  if (millisec == 0)
    als21c_data.en_wait = 0x0;
  else
    als21c_data.en_wait = 0x1;
  als21c_set_reg_sysm_ctrl();
}

/*!
 * @brief  return wait time in millis between two measurements
 * @return millisec
 */
uint32_t als21c_get_wait_time_millisec() {
  uint32_t wtime, millisec;
  millisec = 0;
  if (als21c_data.en_wait) {
    wtime = 8 << als21c_data.wtime_unit;
    millisec = wtime * (als21c_data.wtime + 1);
  }
  return millisec;
}

/*!
 * @brief  return total time in millis between two measurements
 * @return millisec
 *         sum of integration time and wait time
 */
uint32_t als21c_get_delay_millisec() {
  uint32_t millisec;
  millisec = als21c_get_integration_time_millisec() + als21c_get_wait_time_millisec();
  return millisec;
}

/*!
 * @brief  return raw ALS count
 * @return count
 *         positive or zero value is als count
 *         negative value is error condition
 *         ALS21C_ERR_SATURATION: error in the analog part (amplifier, comparator)
 *         ALS21C_ERR_NOT_READY: reading too soon (sensor still counting)
 */
int32_t als21c_read_als() {
  uint16_t count;
  als21c_get_reg_data_status();
  if (!als21c_data.data_ready) return ALS21C_ERR_NOT_READY;
  if (als21c_data.saturation_als || als21c_data.saturation_comp) return ALS21C_ERR_SATURATION;
  count = als21c_i2c_read16(ALS21C_REG_ALS_DATA);
  return count;
}

void als21c_increase_gain() {
  if (als21c_data.pd_sel == 0) {
    /* double gain */
    als21c_data.pd_sel = 1;
    als21c_set_reg_als_gain();
  } else if (als21c_data.pga_als != ALS21C_GAIN_256X) {
    /* more gain */
    als21c_data.pga_als <<= 1;
    als21c_data.pd_sel = 0;
    als21c_set_reg_als_gain();
  } else if (als21c_data.int_time != ALS21C_INT_TIME_64T) {
    /* increase int_time */
    als21c_data.int_time++;
    als21c_set_reg_als_time();
  } else if (als21c_data.als_conv < 15) {
    /* increase als_conv */
    als21c_data.als_conv++;
    als21c_set_reg_als_time();
  }
}

void als21c_decrease_gain() {
  if (als21c_data.als_conv > 0) {
    /* decrease als_conv */
    --als21c_data.als_conv;
    als21c_set_reg_als_time();
  } else if (als21c_data.int_time != ALS21C_INT_TIME_1T) {
    /* decrease int_time */
    --als21c_data.int_time;
    als21c_set_reg_als_time();
  } else if (als21c_data.pd_sel == 1) {
    /* halve gain */
    als21c_data.pd_sel = 0;
    als21c_set_reg_als_gain();
  } else if (als21c_data.pga_als != ALS21C_GAIN_1X) {
    /* less gain */
    als21c_data.pga_als >>= 1;
    als21c_data.pd_sel = 1;
    als21c_set_reg_als_gain();
  }
}

/*!
 * @brief  return ALS light intensity in lux
 * @return lux
 *         positive or zero value is lux
 *         negative value is error condition
 *         ALS21C_ERR_SATURATION: error in the analog part (amplifier, comparator)
 *         ALS21C_ERR_OVERFLOW: error in the digital part (counter)
 *         ALS21C_ERR_NOT_READY: reading too soon (sensor still counting)
 */
int32_t als21c_read_lux() {
  int32_t count, max_count;
  int32_t lux;

  als21c_get_reg_data_status();
  if (!als21c_data.data_ready) return ALS21C_ERR_NOT_READY;

  count = als21c_i2c_read16(ALS21C_REG_ALS_DATA);
  max_count = als21c_get_max_count();

  /* convert adc count to lux */
  lux = als21c_count_to_lux(count);

  /* automatic configuration of gain and integration time */
  if (als21c_data.auto_lux) {
    if (als21c_data.saturation_als || als21c_data.saturation_comp || (count > max_count - max_count / 4))
      als21c_decrease_gain();
    else if (count < max_count / 4)
      als21c_increase_gain();
  }

  if (als21c_data.saturation_als || als21c_data.saturation_comp)
    return ALS21C_ERR_SATURATION; /* analog */
  else if (count >= max_count)
    return ALS21C_ERR_OVERFLOW; /* digital */

  return lux;
}

/*!
 * @brief  set auto lux adjust
 * @param  onoff enable or disable
 *         when enabled, automatically adjusts ALS gain and integration time
 */
void als21c_set_auto_lux(bool onoff) {
  als21c_data.auto_lux = onoff;
}

/*!
 * @brief  enable or disable interrupt
 * @param  onoff
 */
void als21c_enable_interrupt(bool onoff) {
  als21c_data.en_aint = onoff ? 0x1 : 0x0;
  als21c_set_reg_int_ctrl();
}

/*!
 * @brief  set ALS synchronisation
 * @param  onoff
 *         when enabled, ALS light measurement waits until interrupt is cleared
 */
void als21c_enable_als_sync(bool onoff) {
  als21c_data.als_sync = onoff ? 0x1 : 0x0;
  als21c_set_reg_int_ctrl();
}

/*!
 * @brief  get ALS interrupt status
 * @return true if interrupt pending
 */
bool als21c_interrupt_status() {
  als21c_get_reg_int_flag();
  return als21c_data.int_por || als21c_data.int_als;
}

/*!
 * @brief  clear ALS interrupt
 */
void als21c_clear_interrupt() {
  als21c_i2c_write8(ALS21C_REG_INT_FLAG, 0x0);
}

/*!
 * @brief  set interrupt persistence
 * @param  pers
 *         sets number of consecutive measurements that have to be outside threshold to trigger an interrupt
 *         if pers is zero, thresholds are ignored and every measurement triggers an interrupt
 */
void als21c_set_persistence(uint8_t pers) {
  if (pers > 15) pers = 15;
  als21c_data.prs_als = pers;
  als21c_set_reg_persistence();
}

/*!
 * @brief  set lower threshold for ALS count to trigger an interrupt
 * @param  value
 */
void als21c_set_low_threshold(uint16_t value) {
  als21c_i2c_write16(ALS21C_REG_ALS_THRES_L, value);
}

/*!
 * @brief  set upper threshold for ALS count to trigger an interrupt
 * @param  value
 */
void als21c_set_high_threshold(uint16_t value) {
  als21c_i2c_write16(ALS21C_REG_ALS_THRES_H, value);
}

/*!
 * @brief  get ALS product id
 * @return product id
 */
uint16_t als21c_get_product_id() {
  return als21c_i2c_read16(ALS21C_REG_PROD_ID);
}

/* sysm_ctrl register */
void als21c_set_reg_sysm_ctrl() {
  uint8_t dta = als21c_data.swrst << 7 | als21c_data.en_wait << 6 | als21c_data.en_frst << 5 | als21c_data.en_once << 1 | als21c_data.en_als;
  als21c_i2c_write8(ALS21C_REG_SYSM_CTRL, dta);
}

/* int_ctrl register */
void als21c_set_reg_int_ctrl() {
  uint8_t dta = als21c_data.als_sync << 4 | als21c_data.en_aint;
  als21c_i2c_write8(ALS21C_REG_INT_CTRL, dta);
}

/* set interrupt flag register */
void als21c_set_reg_int_flag() {
  uint8_t dta = als21c_data.int_por << 7 | als21c_data.data_flag << 6 | als21c_data.int_als;
  als21c_i2c_write8(ALS21C_REG_INT_FLAG, dta);
}

/* get interrupt flag register */
void als21c_get_reg_int_flag() {
  uint8_t data = als21c_i2c_read8(ALS21C_REG_INT_FLAG);
  als21c_data.int_por = (data >> 7) & 0x1;
  als21c_data.data_flag = (data >> 6) & 0x1;
  als21c_data.int_als = data & 0x1;
}

/* wait_time register */
void als21c_set_reg_wait_time() {
  uint8_t dta = als21c_data.wtime_unit << 6 | als21c_data.wtime;
  als21c_i2c_write8(ALS21C_REG_WAIT_TIME, dta);
}

/* als_gain register */
void als21c_set_reg_als_gain() {
  uint8_t dta = als21c_data.pd_sel << 7 | als21c_data.pga_als;
  als21c_i2c_write8(ALS21C_REG_ALS_GAIN, dta);
}

/* als_time register */
void als21c_set_reg_als_time() {
  uint8_t dta = als21c_data.als_conv << 4 | als21c_data.int_time;
  als21c_i2c_write8(ALS21C_REG_ALS_TIME, dta);
}

/* persistence register */
void als21c_set_reg_persistence() {
  uint8_t dta = als21c_data.int_src << 4 | als21c_data.prs_als;
  als21c_i2c_write8(ALS21C_REG_PERSISTENCE, dta);
}

/* data status register */
void als21c_get_reg_data_status() {
  uint8_t data = als21c_i2c_read8(ALS21C_REG_DATA_STATUS);
  als21c_data.data_ready = (data >> 7) & 0x1;
  als21c_data.saturation_als = (data >> 1) & 0x1;
  als21c_data.saturation_comp = data & 0x1;
}

#ifdef __cplusplus
}; /* namespace als21c */
#endif
