/*!
 *
 * 	I2C Driver for NEWOPT XYC_ALS21C ambient light sensor
 *
 * 	This is a library for the NEWOPT XYC_ALS21C breakout:
 * 	http://oshwlab.com/koendv/xyc_als21c_k1
 *
 *      code for rt-thread
 */

#include <xyc_als21c_k1.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <rtconfig.h>

#define DBG_TAG "ALS21C"
#define DBG_LVL DBG_ERR
#include <rtdbg.h>

#ifndef ALS21C_I2C_BUS
#ifdef BSP_USING_HARD_I2C1
#define ALS21C_I2C_BUS "hwi2c1" /* i2c bus */
#else
#define ALS21C_I2C_BUS "i2c1"   /* i2c bus */
#endif
#endif

#ifdef __cplusplus

namespace als21c
{
#endif

/* I2C operations */

static struct rt_i2c_bus_device *i2c_bus = RT_NULL;

void als21c_i2c_write8(const uint8_t reg, const uint8_t data)
{
    struct rt_i2c_msg msg[1];
    uint8_t           buf[2];

    if (i2c_bus == RT_NULL)
    {
        i2c_bus = rt_i2c_bus_device_find(ALS21C_I2C_BUS);
        if (i2c_bus == RT_NULL)
        {
            LOG_E("i2c bus error");
            return;
        }
    }

    buf[0] = reg;
    buf[1] = data;

    msg[0].addr  = ALS21C_I2C_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 2;
    msg[0].buf   = buf;

    if (rt_i2c_transfer(i2c_bus, msg, 1) != 1)
        LOG_E("i2c write error");
    return;
}

void als21c_i2c_write16(const uint8_t reg, const uint16_t data)
{
    struct rt_i2c_msg msg[2];
    uint8_t           buf[3];

    if (i2c_bus == RT_NULL)
    {
        i2c_bus = rt_i2c_bus_device_find(ALS21C_I2C_BUS);
        if (i2c_bus == RT_NULL)
        {
            LOG_E("i2c bus error");
            return;
        }
    }

    buf[0] = reg;
    buf[1] = data & 0xff;
    buf[2] = data >> 8;

    msg[0].addr  = ALS21C_I2C_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 3;
    msg[0].buf   = buf;

    if (rt_i2c_transfer(i2c_bus, msg, 1) != 1)
        LOG_E("i2c write error");
    return;
}

uint8_t als21c_i2c_read8(const uint8_t reg)
{
    struct rt_i2c_msg msg[2];
    uint8_t           buf = 0;

    if (i2c_bus == RT_NULL)
    {
        i2c_bus = rt_i2c_bus_device_find(ALS21C_I2C_BUS);
        if (i2c_bus == RT_NULL)
        {
            LOG_E("i2c bus error");
            return 0;
        }
    }

    msg[0].addr  = ALS21C_I2C_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = (uint8_t *)&reg;
    msg[1].addr  = ALS21C_I2C_ADDR;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = 1;
    msg[1].buf   = &buf;

    if (rt_i2c_transfer(i2c_bus, msg, 2) != 2)
        LOG_E("i2c read error");
    return buf;
}

uint16_t als21c_i2c_read16(const uint8_t reg)
{
    struct rt_i2c_msg msg[2];
    uint8_t           buf[2] = {0};
    uint16_t          data;

    if (i2c_bus == RT_NULL)
    {
        i2c_bus = rt_i2c_bus_device_find(ALS21C_I2C_BUS);
        if (i2c_bus == RT_NULL)
        {
            LOG_E("i2c bus error");
            return 0;
        }
    }

    msg[0].addr  = ALS21C_I2C_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len   = 1;
    msg[0].buf   = (uint8_t *)&reg;
    msg[1].addr  = ALS21C_I2C_ADDR;
    msg[1].flags = RT_I2C_RD;
    msg[1].len   = 2;
    msg[1].buf   = buf;

    if (rt_i2c_transfer(i2c_bus, msg, 2) != 2)
        LOG_E("i2c read error");

    data = buf[0] | buf[1] << 8;
    return data;
}

void als21c_dump_regs()
{
    rt_kprintf("reg_sysm_ctrl 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_SYSM_CTRL));
    rt_kprintf("reg_int_ctrl 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_INT_CTRL));
    rt_kprintf("reg_int_flag 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_INT_FLAG));
    rt_kprintf("reg_wait_time 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_WAIT_TIME));
    rt_kprintf("reg_als_gain 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_ALS_GAIN));
    rt_kprintf("reg_als_time 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_ALS_TIME));
    rt_kprintf("reg_persistence 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_PERSISTENCE));
    rt_kprintf("reg_als_thres_l 0x%02X\r\n", als21c_i2c_read16(ALS21C_REG_ALS_THRES_L));
    rt_kprintf("reg_als_thres_h 0x%02X\r\n", als21c_i2c_read16(ALS21C_REG_ALS_THRES_H));
    rt_kprintf("reg_data_status 0x%02X\r\n", als21c_i2c_read8(ALS21C_REG_DATA_STATUS));
    rt_kprintf("reg_als_data 0x%02X\r\n", als21c_i2c_read16(ALS21C_REG_ALS_DATA));
    rt_kprintf("reg_prod_id 0x%02X\r\n", als21c_i2c_read16(ALS21C_REG_PROD_ID));
}

#ifdef RT_USING_FINSH
#include <finsh.h>

/* example code for polling */

/*! rt-thread shell command : als32c_lux */
void als21c_lux(void)
{
    static bool first_time = true;
    int32_t     lux;
    if (first_time)
    {
        als21c_begin();
        als21c_set_gain_value(256);
        als21c_set_integration_time(64);
        als21c_set_wait_time_millisec(250);
        als21c_set_auto_lux(true);
        als21c_enable(true);
        rt_kprintf("xyc-als21c-k1 init\r\n");
        first_time = false;
    }
    lux = als21c_read_lux();
    while (lux < 0)
    {
        /* lighting conditions have changed.
           e.g. change from shadow to bright sunlight.
           repeat measurement until auto_lux has found
           appropriate gain and integration time */
        if (lux == ALS21C_ERR_NOT_READY)
            rt_kprintf("WAIT\r\n");
        else if (lux == ALS21C_ERR_SATURATION)
            rt_kprintf("SATURATION\r\n");
        else if (lux == ALS21C_ERR_OVERFLOW)
            rt_kprintf("OVERFLOW\r\n");
        rt_thread_mdelay(als21c_get_delay_millisec() + 10);
        lux = als21c_read_lux();
    }
    rt_kprintf("lux: %d\r\n", lux);
    return;
}

MSH_CMD_EXPORT(als21c_lux, print light intensity in lux);
#endif /* RT_USING_FINSH & FINSH_USING_MSH */

#ifdef __cplusplus
}; /* namespace als21c */
#endif
