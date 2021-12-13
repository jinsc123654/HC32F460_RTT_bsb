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

#include "bsp_io_init.h"

void SystemLight_thread_init(void);//空闲中断监测CPU的线程

#define DELAY_MS                (RT_TICK_PER_SECOND)    /* 1s */



int32_t main(void)
{
    bsp_io_init();
    SystemLight_thread_init();

    while (1)
    {
        rt_thread_delay(DELAY_MS);
    }
}

