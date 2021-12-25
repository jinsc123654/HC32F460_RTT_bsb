#include "lcd_init.h"


//uint16_t IPS_ram[LCD_H][LCD_W] = {0};//LCD����


void LCD_GPIO_Init(void)
{
    rt_pin_mode(IPS1_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(IPS2_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(IPS3_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(IPS4_PIN, PIN_MODE_OUTPUT);
#ifndef SPI_HW
    rt_pin_mode(IPS5_PIN, PIN_MODE_OUTPUT);
    rt_pin_mode(IPS6_PIN, PIN_MODE_OUTPUT);
#endif
}
static void Spi1_Config(void)
{
    stc_spi_init_t stcSpiInit;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcSpiInit);

    /* Configuration peripheral clock */
    PWC_Fcg1PeriphClockCmd(SPI_UNIT_CLOCK, Enable);

    /* Configuration SPI pin */
    PORT_SetFunc(SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_FUNC, Disable);
    PORT_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_FUNC, Disable);

    /* Configuration SPI structure */
    stcSpiInit.enClkDiv = SpiClkDiv2;
    stcSpiInit.enFrameNumber = SpiFrameNumber1;
    stcSpiInit.enDataLength = SpiDataLengthBit8;
    stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
    stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    stcSpiInit.enReadBufferObject = SpiReadReceiverBuffer;
    stcSpiInit.enWorkMode = SpiWorkMode3Line;
    stcSpiInit.enTransMode = SpiTransOnlySend;
    stcSpiInit.enCommAutoSuspendEn = Disable;
    stcSpiInit.enModeFaultErrorDetectEn = Disable;
    stcSpiInit.enParitySelfDetectEn = Disable;
    stcSpiInit.enParityEn = Disable;
    stcSpiInit.enParity = SpiParityEven;

    /* SPI master mode */
    stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    stcSpiInit.stcDelayConfig.enSsSetupDelayOption = SpiSsSetupDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsSetupDelayTime = SpiSsSetupDelaySck1;
    stcSpiInit.stcDelayConfig.enSsHoldDelayOption = SpiSsHoldDelayCustomValue;
    stcSpiInit.stcDelayConfig.enSsHoldDelayTime = SpiSsHoldDelaySck1;
    stcSpiInit.stcDelayConfig.enSsIntervalTimeOption = SpiSsIntervalCustomValue;
    stcSpiInit.stcDelayConfig.enSsIntervalTime = SpiSsIntervalSck6PlusPck2;

    SPI_Init(SPI_UNIT, &stcSpiInit);
    SPI_Cmd(SPI_UNIT, Enable);

}
void SPI_DAM_ON(void)//ǿ�п���һ��DAM
{
    uint16_t null_temp = 0;
    SPI_Cmd(SPI_UNIT, Disable);//�ȹر�SPI ����DMA���ٿ���

    DMA_SetSrcAddress(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, (uint32_t)(null_temp));
    DMA_SetTransferCnt(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, 1 );
    DMA_ChannelCmd(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, Enable);
    SPI_Cmd(SPI_UNIT, Enable);//����SPI DMA��������
}
void InitSPI1_dma(void)
{
    stc_dma_config_t stcDmaCfg;

    /* configuration structure initialization */
    MEM_ZERO_STRUCT(stcDmaCfg);

    /* Configuration peripheral clock */
    PWC_Fcg0PeriphClockCmd(SPI_DMA_CLOCK_UNIT, Enable);
    PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);

    /* Configure TX DMA */
    stcDmaCfg.u16BlockSize = 1;
    stcDmaCfg.u16TransferCnt = 1;
    stcDmaCfg.u32SrcAddr = (uint32_t)&stcDmaCfg;
    stcDmaCfg.u32DesAddr = (uint32_t)(&SPI_UNIT->DR);
    stcDmaCfg.stcDmaChCfg.enSrcInc = AddressIncrease;
    stcDmaCfg.stcDmaChCfg.enDesInc = AddressFix;
    stcDmaCfg.stcDmaChCfg.enTrnWidth = Dma8Bit;
    stcDmaCfg.stcDmaChCfg.enIntEn = Disable;
    DMA_InitChannel(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, &stcDmaCfg);

    DMA_SetTriggerSrc(SPI_DMA_UNIT, SPI_DMA_TX_CHANNEL, SPI_DMA_TX_TRIG_SOURCE);

    /* Enable DMA. */
    DMA_Cmd(SPI_DMA_UNIT, Enable);
    
    /* ����һ��DMA  ʹDAMflag����*/
    SPI_DAM_ON();
}
/******************************************************************************
      ����˵����LCD��������д�뺯��
      ������ݣ�dat  Ҫд��Ĵ�������
      ����ֵ��  ��
******************************************************************************/

void LCD_Writ_Bus(u8 dat) 
{    
    LCD_CS_Clr();
#ifndef SPI_HW
    u8 i;
    for(i=0;i<8;i++)
    {
        LCD_SCLK_Clr();
        if(dat&0x80)
        {
            LCD_MOSI_Set();
        }
        else
        {
            LCD_MOSI_Clr();
        }
        LCD_SCLK_Set();
        dat<<=1;
    }
#else
    SPI_SetDataLength(SPI_UNIT,SpiDataLengthBit8);
    SPI1_ReadWrite(dat);
#endif
  LCD_CS_Set();    
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA8(u8 dat)
{
    LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_DATA(u16 dat)
{
    LCD_Writ_Bus(dat>>8);
    LCD_Writ_Bus(dat);
}


/******************************************************************************
      ����˵����LCDд������
      ������ݣ�dat д�������
      ����ֵ��  ��
******************************************************************************/
void LCD_WR_REG(u8 dat)
{
    LCD_DC_Clr();//д����
    LCD_Writ_Bus(dat);
    LCD_DC_Set();//д����
}

#if LCD_V == _320_172
/******************************************************************************
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
    if(USE_HORIZONTAL==0)
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1+34);
        LCD_WR_DATA(x2+34);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//������д
    }
    else if(USE_HORIZONTAL==1)
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1+34);
        LCD_WR_DATA(x2+34);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//������д
    }
    else if(USE_HORIZONTAL==2)
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1+34);
        LCD_WR_DATA(y2+34);
        LCD_WR_REG(0x2c);//������д
    }
    else
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1+34);
        LCD_WR_DATA(y2+34);
        LCD_WR_REG(0x2c);//������д
    }
}
void LCD_Init(void)
{
    LCD_GPIO_Init();//��ʼ��GPIO
#ifdef SPI_HW
    Spi1_Config();
    InitSPI1_dma();
#endif
    
    LCD_RES_Clr();//��λ
    delay_ms(30);
    LCD_RES_Set();
    delay_ms(100);
    LCD_BLK_Set();//�򿪱���
    delay_ms(100);

    LCD_WR_REG(0x11); 
 
    LCD_WR_REG(0x36); 
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
    else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
    else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
    else LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A);
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33); 

    LCD_WR_REG(0xB7); 
    LCD_WR_DATA8(0x35);  

    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x35);

    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x2C);

    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x13);   

    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);  

    LCD_WR_REG(0xC6); 
    LCD_WR_DATA8(0x0F);    

    LCD_WR_REG(0xD0); 
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);

    LCD_WR_REG(0xD6); 
    LCD_WR_DATA8(0xA1);

    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xF0);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x05);
    LCD_WR_DATA8(0x29);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x3E);
    LCD_WR_DATA8(0x38);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x12);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x30);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xF0);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x0A);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x0B);
    LCD_WR_DATA8(0x07);
    LCD_WR_DATA8(0x28);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x3E);
    LCD_WR_DATA8(0x36);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x14);
    LCD_WR_DATA8(0x29);
    LCD_WR_DATA8(0x32);

    LCD_WR_REG(0x21); 

    LCD_WR_REG(0x11);
    delay_ms(120);
    LCD_WR_REG(0x29); 
}

#elif LCD_V == _240_240
/******************************************************************************
      ����˵����������ʼ�ͽ�����ַ
      ������ݣ�x1,x2 �����е���ʼ�ͽ�����ַ
                y1,y2 �����е���ʼ�ͽ�����ַ
      ����ֵ��  ��
******************************************************************************/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
    if(USE_HORIZONTAL==0)
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//������д
    }
    else if(USE_HORIZONTAL==1)
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1+80);
        LCD_WR_DATA(y2+80);
        LCD_WR_REG(0x2c);//������д
    }
    else if(USE_HORIZONTAL==2)
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1);
        LCD_WR_DATA(x2);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//������д
    }
    else
    {
        LCD_WR_REG(0x2a);//�е�ַ����
        LCD_WR_DATA(x1+80);
        LCD_WR_DATA(x2+80);
        LCD_WR_REG(0x2b);//�е�ַ����
        LCD_WR_DATA(y1);
        LCD_WR_DATA(y2);
        LCD_WR_REG(0x2c);//������д
    }
}

void LCD_Init(void)
{
    LCD_GPIO_Init();//��ʼ��GPIO
#ifdef SPI_HW
    Spi1_Config();
    InitSPI1_dma();
#endif
    
    LCD_RES_Clr();
    delay_ms(20);
    LCD_RES_Set();
    delay_ms(20);
    LCD_BLK_Set();
    LCD_WR_REG(0x36);
    if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
    else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
    else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
    else LCD_WR_DATA8(0xA0);

    LCD_WR_REG(0x3A); 
    LCD_WR_DATA8(0x05);

    LCD_WR_REG(0xB2);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x00);
    LCD_WR_DATA8(0x33);
    LCD_WR_DATA8(0x33); 

    LCD_WR_REG(0xB7); 
    LCD_WR_DATA8(0x35);  

    LCD_WR_REG(0xBB);
    LCD_WR_DATA8(0x37);

    LCD_WR_REG(0xC0);
    LCD_WR_DATA8(0x2C);

    LCD_WR_REG(0xC2);
    LCD_WR_DATA8(0x01);

    LCD_WR_REG(0xC3);
    LCD_WR_DATA8(0x12);   

    LCD_WR_REG(0xC4);
    LCD_WR_DATA8(0x20);  

    LCD_WR_REG(0xC6); 
    LCD_WR_DATA8(0x0F);    

    LCD_WR_REG(0xD0); 
    LCD_WR_DATA8(0xA4);
    LCD_WR_DATA8(0xA1);

    LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x2B);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x4C);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x0B);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x23);

    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x51);
    LCD_WR_DATA8(0x2F);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x20);
    LCD_WR_DATA8(0x23);

    LCD_WR_REG(0x21); 

    LCD_WR_REG(0x11); 
    //Delay (120); 

    LCD_WR_REG(0x29); 
} 
#endif







