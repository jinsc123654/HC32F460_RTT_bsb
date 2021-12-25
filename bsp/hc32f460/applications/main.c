/*
 * Copyright (C) 2021, lizhengyang
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author          Notes
 * 2021-09-02      lizhengyang     first version
 */
#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "lvgl.h"
#include "bsp_io_init.h"

void SystemLight_thread_init(void);//空闲中断监测CPU的线程

#define DELAY_MS                (1)    /* 1s */


void lv_example_meter_2(void);
int32_t main(void)
{

    SystemLight_thread_init();
    lv_example_meter_2 ();
    while (1)
    {    
        lv_task_handler();
        rt_thread_mdelay(DELAY_MS);
    }
}

