#include "bsp_io_init.h"

void LED2_Toggle(void *args)
{
    rt_pin_write(LED_RUN_PIN, PIN_TOGGLE);
    rt_kprintf( "LED_RUN_PIN" );
}

int bsp_io_init(void)
{
    PORT_DebugPortSetting(TDO_SWO,Disable); //关闭JTDI 调试管脚
    PORT_DebugPortSetting(TDI,Disable); //关闭JTDI 调试管脚
    PORT_DebugPortSetting(TRST,Disable); //关闭JTDI 调试管脚
    EFM_Unlock();
    bM4_EFM_FRMC_CACHE = 1;//开启flash加速
    EFM_Lock();
//    rt_pin_mode(LED_RUN_PIN, PIN_MODE_OUTPUT);
//    rt_pin_mode(LED_STA_PIN, PIN_MODE_OUTPUT);
//    
//    rt_pin_mode(KEY1_PIN, PIN_MODE_INPUT_PULLUP);
//    rt_pin_mode(KEY2_PIN, PIN_MODE_INPUT_PULLUP);
//    rt_pin_mode(KEY3_PIN, PIN_MODE_INPUT_PULLUP);
//    rt_pin_mode(KEY4_PIN, PIN_MODE_INPUT_PULLUP);
//    
//    rt_pin_mode(IPS1_PIN, PIN_MODE_OUTPUT);
//    rt_pin_mode(IPS2_PIN, PIN_MODE_OUTPUT);
//    rt_pin_mode(IPS3_PIN, PIN_MODE_OUTPUT);
//    rt_pin_mode(IPS4_PIN, PIN_MODE_OUTPUT);
//    rt_pin_mode(IPS5_PIN, PIN_MODE_OUTPUT);
//    rt_pin_mode(IPS6_PIN, PIN_MODE_OUTPUT);
//    
//    /*attach KEY_PIN irq*/
//    rt_pin_attach_irq(KEY4_PIN, PIN_IRQ_MODE_FALLING, LED2_Toggle, RT_NULL); //绑定外部中断
//    /*enable KEY_PIN irq*/
//    rt_pin_irq_enable(KEY4_PIN, PIN_IRQ_ENABLE);
    return 0;
}
INIT_BOARD_EXPORT(bsp_io_init);
