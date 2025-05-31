/* 
 * This is a simple adaptive brightness for u8g2 and xyc-als21c-k1.
 *
 * Light is measured and screen brightness adjusted.
 * If, during 5 seconds, there is 10% more light or 20% less light, 
 * an interrupt is received and screen brightness is adjusted.
 * No polling.
 */

#include <stdlib.h>
#include <rtthread.h>
#include <drv_gpio.h>
#include <math.h>
#include <xyc_als21c_k1.h>
#include <u8g2.h>

#define ALS21C_INT_PIN GET_PIN(C, 13)
#define PERSISTENCE    15

const int32_t max_brightness     = 255;
const int32_t min_brightness     = 30;
const int32_t max_brightness_als = 1000; // als count for maximum brightness

extern u8g2_t      u8g2;
static rt_thread_t auto_bright_threadid = RT_NULL;
static rt_sem_t    als21c_irq_sem       = RT_NULL;

void set_display_brightness(int32_t als)
{
    if (als < 1) als = 1;
    /* for more speed, replace the next formula with lookup table and interpolate */
    int32_t brightness = (max_brightness - min_brightness) * logf((float)als * 2.71828 / (float)max_brightness_als) + min_brightness;
    if (brightness < 1) brightness = 1;
    if (brightness > 255) brightness = 255;
    rt_kprintf("brightness: %d\r\n", brightness);
    u8g2_SetContrast(&u8g2, brightness);
}

/* interrupt handler */
static void als21c_handler(void *ptr)
{
    rt_sem_release(als21c_irq_sem);
}

void auto_bright_func()
{
    rt_pin_mode(ALS21C_INT_PIN, PIN_MODE_INPUT_PULLUP);
    als21c_enable_interrupt(false);
    als21c_clear_interrupt();
    /* watch als21c interrupt pin */
    rt_pin_attach_irq(ALS21C_INT_PIN, PIN_IRQ_MODE_FALLING, als21c_handler, RT_NULL);
    rt_pin_irq_enable(ALS21C_INT_PIN, PIN_IRQ_ENABLE);
    als21c_set_gain_value(256);
    als21c_set_integration_time(64);
    als21c_set_wait_time_millisec(250);
    als21c_set_persistence(PERSISTENCE);
    als21c_set_low_threshold(0xffff);
    als21c_set_high_threshold(0x0);
    als21c_clear_interrupt();
    als21c_enable_interrupt(true);
    als21c_enable(true);
    rt_kprintf("brightness update after %d ms\r\n", PERSISTENCE * als21c_get_delay_millisec());
    while (1)
    {
        rt_sem_take(als21c_irq_sem, RT_WAITING_FOREVER);
        int32_t als = als21c_read_als();
        rt_kprintf("als: %d\r\n", als);
        if (als == ALS21C_ERR_NOT_READY)
        {
            als21c_clear_interrupt();
            continue;
        }
        if ((als == ALS21C_ERR_SATURATION) || (als == ALS21C_ERR_OVERFLOW))
            als = 65535;
        if (als < 10) als = 10;
        int32_t als_low_limit = als - als / 5;   // -20%
        if (als_low_limit < 0) als_low_limit = 0;
        int32_t als_high_limit = als + als / 10; // +10%
        if (als_high_limit > 65535) als_high_limit = 65535;
        als21c_set_low_threshold(als_low_limit);
        als21c_set_high_threshold(als_high_limit);
        /* new brightness setting */
        set_display_brightness(als);
        als21c_clear_interrupt();
    }
}

int auto_bright()
{
    als21c_irq_sem       = rt_sem_create("auto bright", 0, RT_IPC_FLAG_FIFO);
    auto_bright_threadid = rt_thread_create("auto bright", auto_bright_func, RT_NULL, 1024, 5, 10);
    if (auto_bright_threadid != RT_NULL)
        return rt_thread_startup(auto_bright_threadid);
    else
        return -RT_ERROR;
}

MSH_CMD_EXPORT(auto_bright, set auto brightness);
