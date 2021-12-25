#ifndef __LCD_INIT_H
#define __LCD_INIT_H

#include "board.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "lcd.h"
#define USE_HORIZONTAL 2  //设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏
#define SPI1_ReadWrite(x)  SPI_SendData8(SPI_UNIT, (x))

#define _320_172 0
#define _240_240 1

#define LCD_V _240_240

#if LCD_V == _320_172
#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
#define LCD_W 172	
#define LCD_H 320

#else
#define LCD_W 320
#define LCD_H 172
#endif
#elif LCD_V == _240_240
#define LCD_W 240
#define LCD_H 240
#endif
//extern uint16_t IPS_ram[LCD_H][LCD_W];//LCD缓存

#define SPI_HW
/* SPI_SCK Port/Pin definition */
#define SPI_SCK_PORT                    (PortA)
#define SPI_SCK_PIN                     (Pin02)
#define SPI_SCK_FUNC                    (Func_Spi1_Sck)

/* SPI_MOSI Port/Pin definition */
#define SPI_MOSI_PORT                   (PortA)
#define SPI_MOSI_PIN                    (Pin00)
#define SPI_MOSI_FUNC                   (Func_Spi1_Mosi)

///* SPI_MISO Port/Pin definition */
//#define SPI_MISO_PORT                   (PortA)
//#define SPI_MISO_PIN                    (Pin03)
//#define SPI_MISO_FUNC                   (Func_Spi1_Miso)

/* SPI unit and clock definition */
#define SPI_UNIT                        (M4_SPI1)
#define SPI_UNIT_CLOCK                  (PWC_FCG1_PERIPH_SPI1)

/* SPI DMA unit and channel definition */
#define SPI_DMA_UNIT                    (M4_DMA1)
#define SPI_DMA_CLOCK_UNIT              (PWC_FCG0_PERIPH_DMA1)
#define SPI_DMA_TX_CHANNEL              (DmaCh0)
#define SPI_DMA_TX_TRIG_SOURCE          (EVT_SPI1_SPTI)
#define SPI_DMA_RX_TRIG_SOURCE          (EVT_SPI1_SPRI)

#define SPI_MASTER_MODE
//#define SPI_SLAVE_MODE


/* defined the BLK pin: PE13 */
#define IPS1_PIN                 GET_PIN(A,8)
/* defined the CS pin: PE13 */
#define IPS2_PIN                 GET_PIN(C,13)
/* defined the DC pin: PE13 */
#define IPS3_PIN                 GET_PIN(A,1)
/* defined the RES pin: PE13 */
#define IPS4_PIN                 GET_PIN(B,15)
/* defined the MOSI pin: PE13 */
#define IPS5_PIN                 GET_PIN(A,0)
/* defined the SCLK pin: PE13 */
#define IPS6_PIN                 GET_PIN(A,2)

//-----------------LCD端口定义---------------- 

#define LCD_SCLK_Clr() rt_pin_write(IPS6_PIN,0)//SCL=SCLK
#define LCD_SCLK_Set() rt_pin_write(IPS6_PIN,1)

#define LCD_MOSI_Clr() rt_pin_write(IPS5_PIN,0)//SDA=MOSI
#define LCD_MOSI_Set() rt_pin_write(IPS5_PIN,1)

#define LCD_RES_Clr()  rt_pin_write(IPS4_PIN,0)//RES
#define LCD_RES_Set()  rt_pin_write(IPS4_PIN,1)

#define LCD_DC_Clr()   rt_pin_write(IPS3_PIN,0)//DC
#define LCD_DC_Set()   rt_pin_write(IPS3_PIN,1)
 		     
#define LCD_CS_Clr()   rt_pin_write(IPS2_PIN,0)//CS
#define LCD_CS_Set()   rt_pin_write(IPS2_PIN,1)

#define LCD_BLK_Clr()  rt_pin_write(IPS1_PIN,0)//BLK
#define LCD_BLK_Set()  rt_pin_write(IPS1_PIN,1)




void LCD_GPIO_Init(void);//初始化GPIO
void LCD_Writ_Bus(u8 dat);//模拟SPI时序
void LCD_WR_DATA8(u8 dat);//写入一个字节
void LCD_WR_DATA(u16 dat);//写入两个字节
void LCD_WR_REG(u8 dat);//写入一个指令
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2);//设置坐标函数
void LCD_Init(void);//LCD初始化
#endif




