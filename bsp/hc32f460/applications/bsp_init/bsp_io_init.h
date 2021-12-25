#ifndef __BSP_KEY_INIT_H__
#define __BSP_KEY_INIT_H__
        
#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>

/* defined the LED1 pin: PH2 */
#define LED_RUN_PIN              GET_PIN(H,2)



int bsp_io_init(void);//初始化普通IO口




#endif /*endif __BSP_KEY_INIT_H__*/
