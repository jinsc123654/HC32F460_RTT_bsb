#include "rtthread.h"
#include "bsp_io_init.h"

#define CPU_USAGE_CALC_TICK    10
#define CPU_USAGE_LOOP        100
#define RUN_LED               GET_PIN(H,2)

static rt_uint8_t  cpu_usage_major = 0, cpu_usage_minor= 0;
static rt_uint8_t cpu_usage_major_max = 0, cpu_usage_minor_max = 0;
static rt_uint32_t total_count = 0;

static void cpu_usage_idle_hook(void);                                          //空闲钩子函数本体
void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor);                       //获取CPU使用率

void SystemLight(void *parameter)
{
    rt_int8_t LedCount = 0,LedCounFlag = 0;

    rt_thread_idle_sethook(cpu_usage_idle_hook);                                /* 设置空闲线程钩子 */

//    Bt_M23_Run(TIM2);                                                           //TIM0 运行
    while(1)
    {
        if( LedCounFlag == 0 )
        {
            LedCount++;
        }
        else
        {
            LedCount--;
        }
        if( LedCount >= 108 )
        {
            LedCounFlag = 1;
            LedCount = 99;
            //Gpio_Toggle_list( B_PIN_INIT, RUN_LED );
            rt_pin_write(RUN_LED, PIN_TOGGLE);
            //rt_kprintf("cpu usage: %d.%d%\n", major, minor);
//            LCD_printf( 48,48," CPU:%2d.%2d", major, minor );
        }
        else if( LedCount <= -10 )
        {
            LedCounFlag = 0;
            LedCount = 103;
            //Gpio_Toggle_list( B_PIN_INIT, RUN_LED );
            rt_pin_write(RUN_LED, PIN_TOGGLE);
            //rt_kprintf("cpu usage: %d.%d%\n", major, minor);
//            LCD_printf( 48,48," CPU:%2d.%2d", major, minor );
        }
//        Def_Bt_M23_CCR_Set( TIM2, BtCCR0A, LedCount );
        rt_thread_mdelay(20);
        rt_thread_yield();                                                      /* 放弃剩余时间片，进行一次线程切换 */
    }
}

void SystemLight_thread_init(void)
{
    /* 创建线程 */
    rt_thread_t SystemLight_task_tid = rt_thread_create( "SysBlink",             /* 线程名称 */
                                                          SystemLight, RT_NULL,
                                                          280, RT_THREAD_PRIORITY_MAX - 2, 10);
    if(SystemLight_task_tid != RT_NULL)
    {
        /* 启动线程 */
        //My_Gpio_Init( B_PIN_INIT, RUN_LED );
        rt_pin_mode(RUN_LED, PIN_MODE_OUTPUT);
        rt_pin_write(RUN_LED, PIN_HIGH);
        rt_thread_startup(SystemLight_task_tid);
        rt_kprintf("Blink thread is already started\r\n");
    }
    else
    {
        rt_kprintf("Blink thread is not started\r\n");
    }
}

static void cpu_usage_idle_hook(void)                                           /* 线程钩子函数 */
{
    rt_tick_t tick;
    rt_uint32_t count;
    volatile rt_uint32_t loop;

    if (total_count == 0)
    {
        /* get total count */
        rt_enter_critical();
        tick = rt_tick_get();
        while(rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
        {
            total_count ++;
            loop = 0;

            while (loop < CPU_USAGE_LOOP) loop ++;
        }
        rt_exit_critical();
    }

    count = 0;
    /* get CPU usage */
    tick = rt_tick_get();
    while (rt_tick_get() - tick < CPU_USAGE_CALC_TICK)
    {
        count ++;
        loop  = 0;
        while (loop < CPU_USAGE_LOOP) loop ++;
    }

    /* calculate major and minor */
    if (count < total_count)
    {
        count = total_count - count;
        cpu_usage_major = (count * 100) / total_count;
        cpu_usage_minor = ((count * 100) % total_count) * 100 / total_count;
        if( cpu_usage_major_max < cpu_usage_major || ( cpu_usage_major_max == cpu_usage_major && cpu_usage_minor_max < cpu_usage_minor )  )
        {
            cpu_usage_major_max = cpu_usage_major;
            cpu_usage_minor_max = cpu_usage_minor;
        }
    }
    else
    {
        total_count = count;

        /* no CPU usage */
        cpu_usage_major = 0;
        cpu_usage_minor = 0;
    }
}

void cpu_usage_get(rt_uint8_t *major, rt_uint8_t *minor)                        /* 获得使用率 */
{
    RT_ASSERT(major != RT_NULL);
    RT_ASSERT(minor != RT_NULL);

    *major = cpu_usage_major;
    *minor = cpu_usage_minor;
}
void cpuget( void )
{
    rt_uint8_t major, minor;
                           
    cpu_usage_get(&major, &minor); 
    rt_kprintf("cpu speed: %dMhz\n", SystemCoreClock / 1000000);
    rt_kprintf("cpu usage: %d.%d%\n", major, minor);
    rt_kprintf("cpu usage max: %d.%d%\n", cpu_usage_major_max, cpu_usage_minor_max);
}
MSH_CMD_EXPORT( cpuget, Get CPU usage );
void cpuforget( void )
{
    rt_uint8_t major, minor;
    
    cpu_usage_get(&major, &minor);
    rt_kprintf("cpu usage: %d.%d%\n", major, minor);
    cpu_usage_major_max = cpu_usage_minor_max = 0;
    rt_kprintf("cpu usage max: %d.%d%\n", cpu_usage_major_max, cpu_usage_minor_max);
}
MSH_CMD_EXPORT( cpuforget, Forget CPU maxusage );
