/*
 * Copyright (C) 2020, Huada Semiconductor Co., Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-10-30     CDT          first version
 */


/*******************************************************************************
 * Include files
 ******************************************************************************/
#include <rtthread.h>
#include "drv_spi.h"


#if defined(RT_USING_SPI) && defined(RT_USING_PIN)
#include <rtdevice.h>

#if !defined(BSP_USING_SPI1) && !defined(BSP_USING_SPI2) && \
    !defined(BSP_USING_SPI3) && !defined(BSP_USING_SPI4) && \
    !defined(BSP_USING_SPI5) && !defined(BSP_USING_SPI6)
#error "Please define at least one SPIx"
#endif

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/* #define DEBUG */
#ifndef HC32_SPI_DEBUG
#define SPI_PRINT_DBG(fmt, args...)
#define SPI_PRINT_ERR(fmt, args...) rt_kprintf(fmt, ##args);
#else
#define SPI_PRINT_DBG(fmt, args...) rt_kprintf(fmt, ##args);
#define SPI_PRINT_ERR(fmt, args...) rt_kprintf(fmt, ##args);
#endif

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/* private rt-thread spi ops function */
static rt_err_t hc32_spi_configure(struct rt_spi_device* device,
                                    struct rt_spi_configuration* configuration);
static rt_uint32_t hc32_spi_xfer(struct rt_spi_device* device,
                                     struct rt_spi_message* message);

#if defined(BSP_USING_SPI1)
static void spi1_rx_dma_irq_handle(void);
static void spi1_tx_dma_irq_handle(void);
#endif  /* BSP_USING_SPI1 */

#if defined(BSP_USING_SPI2)
static void spi2_rx_dma_irq_handle(void);
static void spi2_tx_dma_irq_handle(void);
#endif  /* BSP_USING_SPI2 */

#if defined(BSP_USING_SPI3)
static void spi3_rx_dma_irq_handle(void);
static void spi3_tx_dma_irq_handle(void);
#endif  /* BSP_USING_SPI3 */

#if defined(BSP_USING_SPI4)
static void spi4_rx_dma_irq_handle(void);
static void spi4_tx_dma_irq_handle(void);
#endif  /* BSP_USING_SPI4 */

#if defined(BSP_USING_SPI5)
static void spi5_rx_dma_irq_handle(void);
static void spi5_tx_dma_irq_handle(void);
#endif  /* BSP_USING_SPI5 */

#if defined(BSP_USING_SPI6)
static void spi6_rx_dma_irq_handle(void);
static void spi6_tx_dma_irq_handle(void);
#endif  /* BSP_USING_SPI6 */

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
enum
{
#ifdef BSP_USING_SPI1
    SPI1_INDEX,
#endif
#ifdef BSP_USING_SPI2
    SPI2_INDEX,
#endif
#ifdef BSP_USING_SPI3
    SPI3_INDEX,
#endif
#ifdef BSP_USING_SPI4
    SPI4_INDEX,
#endif
#ifdef BSP_USING_SPI5
    SPI5_INDEX,
#endif
#ifdef BSP_USING_SPI6
    SPI6_INDEX,
#endif
    SPI_INDEX_MAX,
};

static const struct spi_index spi_map[] =
{
#ifdef BSP_USING_SPI1
    {SPI1_INDEX, M4_SPI1},
#endif
#ifdef BSP_USING_SPI2
    {SPI2_INDEX, M4_SPI2},
#endif
#ifdef BSP_USING_SPI3
    {SPI3_INDEX, M4_SPI3},
#endif
#ifdef BSP_USING_SPI4
    {SPI4_INDEX, M4_SPI4},
#endif
#ifdef BSP_USING_SPI5
    {SPI5_INDEX, M4_SPI5},
#endif
#ifdef BSP_USING_SPI6
    {SPI6_INDEX, M4_SPI6},
#endif
};

static struct hc32_spi_config spi_config[] =
{
#ifdef BSP_USING_SPI1
    SPI1_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI2
    SPI2_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI3
    SPI3_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI4
    SPI4_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI5
    SPI5_BUS_CONFIG,
#endif

#ifdef BSP_USING_SPI6
    SPI6_BUS_CONFIG,
#endif
};

static const struct spi_irq_handler spi_irq_handlers[] =
{
#ifdef BSP_USING_SPI1
    {spi1_rx_dma_irq_handle, spi1_tx_dma_irq_handle},
#endif

#ifdef BSP_USING_SPI2
    {spi2_rx_dma_irq_handle, spi2_tx_dma_irq_handle},
#endif

#ifdef BSP_USING_SPI3
    {spi3_rx_dma_irq_handle, spi3_tx_dma_irq_handle},
#endif

#ifdef BSP_USING_SPI4
    {spi4_rx_dma_irq_handle, spi4_tx_dma_irq_handle},
#endif

#ifdef BSP_USING_SPI5
    {spi5_rx_dma_irq_handle, spi5_tx_dma_irq_handle},
#endif

#ifdef BSP_USING_SPI6
    {spi6_rx_dma_irq_handle, spi6_tx_dma_irq_handle},
#endif
};

static struct hc32_spi spi_bus_obj[sizeof(spi_config) / sizeof(spi_config[0])] = {0};

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
extern void hc32_board_spi_init(M4_SPI_TypeDef *M4_SPIx, rt_uint8_t mode);


/*SPI 发送及接收 该函数根据F4A0修改来的*/
static en_result_t SPI_TxRx(M4_SPI_TypeDef *SPIx, const void *pvTxBuf, void *pvRxBuf, uint32_t u32Length)
{
    uint32_t u32BitSize;
    __IO uint32_t u32Timecount;
    __IO uint32_t u32Count = 0U;
    en_result_t enRet = Ok;
    uint32_t u32Tmp;
    __UNUSED __IO uint32_t u32Read;

    /* Get data bit size, SPI_DATA_SIZE_4BIT ~ SPI_DATA_SIZE_32BIT */
    u32BitSize = READ_REG32_BIT(SPIx->CFG2, (0x00000F00UL)) >> 8;//取得spi设置的数据长度

    while (u32Count < u32Length)
    {
        if (pvTxBuf != NULL)
        {
            if (u32BitSize <= SpiDataLengthBit8)
            {
                /* SPI_DATA_SIZE_4BIT ~ SPI_DATA_SIZE_8BIT */
                WRITE_REG32(SPIx->DR, ((const uint8_t *)pvTxBuf)[u32Count]);
            }
            else if(u32BitSize <= SpiDataLengthBit16)
            {
                /* SPI_DATA_SIZE_9BIT ~ SPI_DATA_SIZE_16BIT */
                WRITE_REG32(SPIx->DR, ((const uint16_t *)pvTxBuf)[u32Count]);
            }
            else
            {
                /* SPI_DATA_SIZE_20BIT ~ SPI_DATA_SIZE_32BIT */
                WRITE_REG32(SPIx->DR, ((const uint32_t *)pvTxBuf)[u32Count]);
            }
        }
        else
        {
            WRITE_REG32(SPIx->DR, 0xFFFFFFFFUL);
        }

        /* Delay about 10ms */
        u32Timecount = 1000000UL; //因为开启了flash加速 这里也只是移植时自己写的一个数
        do
        {
            if(Reset != SPI_GetFlag(SPIx, SpiFlagReceiveBufferFull))
            {
                break;
            }
            u32Timecount--;
        } while (u32Timecount != 0U);

        if (u32Timecount == 0U)
        {
            enRet = ErrorTimeout;
            break;
        }

        u32Tmp = READ_REG32(SPIx->DR);
        if (pvRxBuf != NULL)
        {
            if (u32BitSize <= SpiDataLengthBit8)
            {
                /* SPI_DATA_SIZE_4BIT ~ SPI_DATA_SIZE_8BIT */
                ((uint8_t *)pvRxBuf)[u32Count] = (uint8_t)u32Tmp;
            }
            else if(u32BitSize <= SpiDataLengthBit16)
            {
                /* SPI_DATA_SIZE_9BIT ~ SPI_DATA_SIZE_16BIT */
                ((uint16_t *)pvRxBuf)[u32Count] = (uint16_t)u32Tmp;
            }
            else
            {
                /* SPI_DATA_SIZE_20BIT ~ SPI_DATA_SIZE_32BIT */
                ((uint32_t *)pvRxBuf)[u32Count] = (uint32_t)u32Tmp;
            }
        }
        else
        {
            /* Dummy read */
            u32Read = READ_REG32(SPIx->DR);
        }

        u32Count++;
    }
    return enRet;
}
/*SPI 发送及接收 该函数根据F4A0修改来的*/
en_result_t SPI_TransmitReceive(M4_SPI_TypeDef *SPIx, const void *pvTxBuf, void *pvRxBuf, uint32_t u32Length)
{
    en_result_t enRet = ErrorInvalidParameter;

    if ((pvTxBuf != NULL) && (pvRxBuf != NULL) && (u32Length != 0U))
    {
        /* Transmit and receive data in full duplex master mode. */
        enRet = SPI_TxRx(SPIx, pvTxBuf, pvRxBuf, u32Length);
    }
    return enRet;
}
/*SPI 发送 该函数根据F4A0修改来的*/
static en_result_t SPI_Tx(M4_SPI_TypeDef *SPIx, const void *pvTxBuf, uint32_t u32Length)
{
    __IO uint32_t u32Count = 0U;
    __IO uint32_t u32Timecount;
    uint32_t u32BitSize;
    en_result_t enRet = Ok;

    /* Get data bit size, SPI_DATA_SIZE_4BIT ~ SPI_DATA_SIZE_32BIT */
    u32BitSize = READ_REG32_BIT(SPIx->CFG2, (0x00000F00UL)) >> 8;

    while (u32Count < u32Length)
    {
        if (u32BitSize <= SpiDataLengthBit8)
        {
            /* SPI_DATA_SIZE_4BIT ~ SPI_DATA_SIZE_8BIT */
            WRITE_REG32(SPIx->DR, ((const uint8_t *)pvTxBuf)[u32Count]);
        }
        else if(u32BitSize <= SpiDataLengthBit16)
        {
            /* SPI_DATA_SIZE_9BIT ~ SPI_DATA_SIZE_16BIT */
            WRITE_REG32(SPIx->DR, ((const uint16_t *)pvTxBuf)[u32Count]);
        }
        else
        {
            /* SPI_DATA_SIZE_20BIT ~ SPI_DATA_SIZE_32BIT */
            WRITE_REG32(SPIx->DR, ((const uint32_t *)pvTxBuf)[u32Count]);
        }

        /* Delay about 10ms */
        u32Timecount = 1000000UL; //因为开启了flash加速 这里也只是移植时自己写的一个数
        do
        {
            if(Reset != SPI_GetFlag(SPIx, SpiFlagSendBufferEmpty))
            {
                break;
            }
            u32Timecount--;
        } while (u32Timecount != 0U);

        if (u32Timecount == 0U)
        {
            enRet = ErrorTimeout;
        }

        u32Count++;
    }
    return enRet;
}
/*SPI 发送 该函数根据F4A0修改来的*/
en_result_t SPI_Transmit(M4_SPI_TypeDef *SPIx, const void *pvTxBuf, uint32_t u32TxLength)
{
    uint32_t u32Flags;
    en_result_t enRet = ErrorInvalidParameter;

    if ((pvTxBuf != NULL) && (u32TxLength != 0U))
    {
        u32Flags = READ_REG32_BIT(SPIx->CR1, (0x00000002UL)) >> 1;
        if (u32Flags == SpiTransOnlySend)
        {
            /* Transmit data in send only mode. */
            enRet = SPI_Tx(SPIx, pvTxBuf, u32TxLength);
        }
        else
        {
            /* Transmit data in full duplex mode. */
            enRet = SPI_TxRx(SPIx, pvTxBuf, NULL, u32TxLength);
        }
    }
    return enRet;
}
/*SPI 接收 该函数根据F4A0修改来的*/
en_result_t SPI_Receive(M4_SPI_TypeDef *SPIx, void *pvRxBuf, uint32_t u32RxLength)
{
    en_result_t enRet = ErrorInvalidParameter;

    if ((pvRxBuf != NULL) && (u32RxLength != 0U))
    {
        /* Receives data in full duplex master mode. */
        enRet = SPI_TxRx(SPIx, NULL, pvRxBuf, u32RxLength);
    }
    return enRet;
}

/**
  * @brief  This function gets index for specific SPI_Instance.
  * @param  Instance
  * @retval index
  */
static uint32_t get_spi_index(M4_SPI_TypeDef *Instance)
{
    uint32_t index = SPI_INDEX_MAX;

    for (uint8_t i = 0U; i < ARRAY_SZ(spi_map); i++)
    {
        if (spi_map[i].Instance == Instance)
        {
            index = spi_map[i].index;
            RT_ASSERT(index < SPI_INDEX_MAX)
            break;
        }
    }

    return index;
}

static uint32_t get_spi_fcg(M4_SPI_TypeDef *Instance)
{
//    return (PWC_FCG1_SPI1 << get_spi_index(Instance));
    return (uint32_t)-1;
}

rt_err_t hc32_spi_init(struct hc32_spi *spi_drv, struct rt_spi_configuration *cfg)
{
    RT_ASSERT(spi_drv != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    stc_spi_init_t stcSpiInit;
    stc_clk_freq_t stcClkFreq;

    M4_SPI_TypeDef *spi_handle = spi_drv->handle.Instance;

    /* Enable spi clock gate */
    PWC_Fcg1PeriphClockCmd(get_spi_fcg(spi_handle), Enable);

    /* Init spi struct as default value */
    MEM_ZERO_STRUCT(stcSpiInit);
    

    /* Slave or master mode */
    if (cfg->mode & RT_SPI_SLAVE)
    {
        stcSpiInit.enMasterSlaveMode = SpiModeSlave;
    }
    else
    {
        stcSpiInit.enMasterSlaveMode = SpiModeMaster;
    }
    /* 3 wire or 4 wire mode */
    if (cfg->mode & RT_SPI_3WIRE)
    {
        stcSpiInit.enWorkMode = SpiWorkMode3Line;
    }
    else
    {
        stcSpiInit.enWorkMode = SpiWorkMode4Line;
    }
    /* spi mode */
    if (0x00 == (cfg->mode & (RT_SPI_CPHA | RT_SPI_CPOL)))                      //空闲SCK为0 奇数采样 偶数变化
    {
        stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
        stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    }
    else if (0x01 == (cfg->mode & (RT_SPI_CPHA | RT_SPI_CPOL)))                 //空闲SCK为0 偶数采样 奇数变化
    {
        stcSpiInit.enSckPolarity = SpiSckIdleLevelLow;
        stcSpiInit.enSckPhase = SpiSckOddChangeEvenSample;
    }
    else if (0x02 == (cfg->mode & (RT_SPI_CPHA | RT_SPI_CPOL)))                 //空闲SCK为1 奇数采样 偶数变化
    {
        stcSpiInit.enSckPolarity = SpiSckIdleLevelHigh;
        stcSpiInit.enSckPhase = SpiSckOddSampleEvenChange;
    }
    else if (0x03 == (cfg->mode & (RT_SPI_CPHA | RT_SPI_CPOL)))                 //空闲SCK为1 偶数采样 奇数变化
    {
        stcSpiInit.enSckPolarity = SpiSckIdleLevelHigh;
        stcSpiInit.enSckPhase = SpiSckOddChangeEvenSample;
    }
    /* LSB or MSB */
    if (cfg->mode & RT_SPI_MSB)
    {
        stcSpiInit.enFirstBitPosition = SpiFirstBitPositionMSB;
    }
    else
    {
        stcSpiInit.enFirstBitPosition = SpiFirstBitPositionLSB;
    }

    /* config data width 4~16, 20, 24, 32 */
    if (4u > cfg->data_width)
    {
        return RT_EIO;
    }
    else if (16u >= cfg->data_width)
    {
        stcSpiInit.enDataLength = (en_spi_data_length_t)((cfg->data_width - 4u) << 8u);
    }
    else if (20u == cfg->data_width)
    {
        stcSpiInit.enDataLength = SpiDataLengthBit20;
    }
    else if (24u == cfg->data_width)
    {
        stcSpiInit.enDataLength = SpiDataLengthBit24;
    }
    else if (32u == cfg->data_width)
    {
        stcSpiInit.enDataLength = SpiDataLengthBit32;
    }
    else
    {
        return RT_EIO;
    }

    /* Get APB clock */
    CLK_GetClockFreq(&stcClkFreq);
    if (cfg->max_hz >= stcClkFreq.pclk1Freq / 2u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv2;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 4u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv4;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 8u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv8;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 16u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv16;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 32u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv32;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 64u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv64;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 128u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv128;
    }
    else if (cfg->max_hz >= stcClkFreq.pclk1Freq / 256u)
    {
        stcSpiInit.enClkDiv = SpiClkDiv256;
    }

    /* spi port init */
    hc32_board_spi_init(spi_handle, cfg->mode);

    if (Ok != SPI_Init(spi_handle, &stcSpiInit))
    {
        return RT_EIO;
    }

    /* DMA configuration */
    if (spi_drv->spi_dma_flag & RT_DEVICE_FLAG_DMA_RX)
    {
        struct dma_config *spi_dma;
        stc_dma_config_t  stcDmaInit;

        /* Get spi dma_rx */
        spi_dma = spi_drv->config->dma_rx;

        /* Config Dma */
        MEM_ZERO_STRUCT(stcDmaInit);

        stcDmaInit.stcDmaChCfg.enIntEn     = Disable;
        stcDmaInit.u16BlockSize = 1UL;
        stcDmaInit.u16TransferCnt  = 0;
        stcDmaInit.u32SrcAddr   = 0;
        stcDmaInit.u32DesAddr  = (uint32_t)(&spi_handle->DR);
        stcDmaInit.stcDmaChCfg.enSrcInc   = AddressFix;
        stcDmaInit.stcDmaChCfg.enDesInc   = AddressIncrease;

        if (8u >= cfg->data_width)
        {
            stcDmaInit.stcDmaChCfg.enTrnWidth = Dma8Bit;
        }
        else if (16u >= cfg->data_width)
        {
            stcDmaInit.stcDmaChCfg.enTrnWidth = Dma16Bit;
        }
        else
        {
            stcDmaInit.stcDmaChCfg.enTrnWidth = Dma32Bit;
        }
        /* Enable Dma clock gate */
        if (M4_DMA1 == spi_dma->Instance)
        {
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1, Enable);
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
        }
        else
        {
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable);
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
        }
        /* Init Dma */
//        if (Ok != DMA_Init(spi_dma->Instance, spi_dma->channel, &stcDmaInit))
//        {
//            return RT_EIO;
//        }
        DMA_InitChannel(spi_dma->Instance, spi_dma->channel, &stcDmaInit);

        /* register interrupt */
        hc32_install_irq_handler(&spi_dma->irq_config,
                                spi_irq_handlers[get_spi_index(spi_handle)].rx_dma_irq_handler,
                                RT_TRUE);

        /* Enable Dma */
        DMA_Cmd(spi_dma->Instance, Enable);
    }
    if (spi_drv->spi_dma_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        struct dma_config *spi_dma;
        stc_dma_config_t  stcDmaInit;

        /* Get spi dma_tx */
        spi_dma = spi_drv->config->dma_tx;

        /* Config Dma */
        MEM_ZERO_STRUCT(stcDmaInit);

        stcDmaInit.stcDmaChCfg.enIntEn     = Disable;
        stcDmaInit.u16BlockSize = 1UL;
        stcDmaInit.u16TransferCnt  = 0;
        stcDmaInit.u32SrcAddr   = (uint32_t)(&spi_handle->DR);
        stcDmaInit.u32DesAddr  = 0;
        stcDmaInit.stcDmaChCfg.enSrcInc   = AddressIncrease;
        stcDmaInit.stcDmaChCfg.enDesInc   = AddressFix;

        if (8u >= cfg->data_width)
        {
            stcDmaInit.stcDmaChCfg.enTrnWidth = Dma8Bit;
        }
        else if (16u >= cfg->data_width)
        {
            stcDmaInit.stcDmaChCfg.enTrnWidth = Dma16Bit;
        }
        else
        {
            stcDmaInit.stcDmaChCfg.enTrnWidth = Dma32Bit;
        }
        /* Enable Dma clock gate */
        if (M4_DMA1 == spi_dma->Instance)
        {
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA1, Enable);
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
        }
        else
        {
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_DMA2, Enable);
            PWC_Fcg0PeriphClockCmd(PWC_FCG0_PERIPH_AOS, Enable);
        }
        /* Init Dma */
//        if (Ok != DMA_Init(spi_dma->Instance, spi_dma->channel, &stcDmaInit))
//        {
//            return RT_EIO;
//        }
        DMA_InitChannel(spi_dma->Instance, spi_dma->channel, &stcDmaInit);

        /* register interrupt */
        hc32_install_irq_handler(&spi_dma->irq_config,
                                spi_irq_handlers[get_spi_index(spi_handle)].tx_dma_irq_handler,
                                RT_TRUE);

        /* Enable Dma */
        DMA_Cmd(spi_dma->Instance, Enable);
    }

    return RT_EOK;
}

static void hc32_spi_deinit(struct hc32_spi *spi_drv)
{
    M4_SPI_TypeDef *spi_handle = spi_drv->handle.Instance;

    SPI_DeInit(spi_handle);

    /* Disable spi clock gate */
    PWC_Fcg1PeriphClockCmd(get_spi_fcg(spi_handle), Disable);
}

static rt_err_t hc32_spi_dma(struct hc32_spi_config *spi_dma, const uint8_t *pvTxBuf, void *pvRxBuf, uint32_t u32Length)
{
    if (RT_NULL == pvTxBuf)
    {
        DMA_SetDesAddress(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel, (uint32_t)pvRxBuf);
        DMA_SetTransferCnt(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel, u32Length);
        DMA_SetTriggerSrc(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel,spi_dma->dma_rx->trigger_evt_src);
        DMA_ChannelCmd(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel, Enable);
    }
    else if (RT_NULL == pvRxBuf)
    {
        DMA_SetSrcAddress(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel, (uint32_t)pvTxBuf);
        DMA_SetTransferCnt(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel, u32Length);
        DMA_SetTriggerSrc(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel,spi_dma->dma_tx->trigger_evt_src);
        DMA_ChannelCmd(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel, Enable);
    }
    else
    {
        DMA_SetDesAddress(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel, (uint32_t)pvRxBuf);
        DMA_SetTransferCnt(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel, u32Length);
        DMA_SetTriggerSrc(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel,spi_dma->dma_rx->trigger_evt_src);

        DMA_SetSrcAddress(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel, (uint32_t)pvTxBuf);
        DMA_SetTransferCnt(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel, u32Length);
        DMA_SetTriggerSrc(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel,spi_dma->dma_tx->trigger_evt_src);

        DMA_ChannelCmd(spi_dma->dma_tx->Instance, spi_dma->dma_tx->channel, Enable);
        DMA_ChannelCmd(spi_dma->dma_rx->Instance, spi_dma->dma_rx->channel, Enable);
    }

    SPI_Cmd(spi_dma->Instance, Enable);

    return RT_EOK;
}

static struct rt_spi_ops hc32_spi_ops =
{
    .configure = hc32_spi_configure,
    .xfer = hc32_spi_xfer,
};

static rt_err_t hc32_spi_configure(struct rt_spi_device *device,
                                struct rt_spi_configuration *configuration)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(configuration != RT_NULL);

    struct hc32_spi *spi_drv =  rt_container_of(device->bus, struct hc32_spi, spi_bus);
    spi_drv->cfg = configuration;
    return hc32_spi_init(spi_drv, configuration);
}

static rt_uint32_t hc32_spi_xfer(struct rt_spi_device *device, struct rt_spi_message *message)
{
    rt_uint8_t state;
    rt_size_t message_length, already_send_length;
    rt_uint16_t send_length;
    rt_uint8_t *recv_buf;
    const rt_uint8_t *send_buf;

    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(device->bus != RT_NULL);
    RT_ASSERT(device->bus->parent.user_data != RT_NULL);
    RT_ASSERT(message != RT_NULL);

    struct hc32_spi *spi_drv =  rt_container_of(device->bus, struct hc32_spi, spi_bus);
    SPI_HandleType *spi_handle = &spi_drv->handle;
    struct hc32_hw_spi_cs *cs = device->parent.user_data;

    if (message->cs_take)
    {
        PORT_ResetPortData((en_port_t)cs->port, (uint16_t)cs->pin);
    }
    message_length = message->length;
    recv_buf = message->recv_buf;
    send_buf = message->send_buf;
    while (message_length)
    {
        if (message_length > 65535)
        {
            send_length = 65535;
            message_length = message_length - 65535;
        }
        else
        {
            send_length = message_length;
            message_length = 0;
        }

        /* calculate the start address */
        already_send_length = message->length - send_length - message_length;
        send_buf = (rt_uint8_t *)message->send_buf + already_send_length;
        recv_buf = (rt_uint8_t *)message->recv_buf + already_send_length;

        if(message->send_buf && message->recv_buf)
        {
            if ((spi_drv->spi_dma_flag & RT_DEVICE_FLAG_DMA_TX) && (spi_drv->spi_dma_flag & RT_DEVICE_FLAG_DMA_RX))
            {
                state = hc32_spi_dma(spi_drv->config, send_buf, recv_buf, send_length);
            }
            else
            {
                SPI_Cmd(spi_handle->Instance, Enable);
                state = SPI_TransmitReceive(spi_handle->Instance, send_buf, recv_buf, send_length);
            }
        }
        else if(message->send_buf)
        {
            if (spi_drv->spi_dma_flag & RT_DEVICE_FLAG_DMA_TX)
            {
                state = hc32_spi_dma(spi_drv->config, send_buf, RT_NULL, send_length);
            }
            else
            {
                SPI_Cmd(spi_handle->Instance, Enable);
                state = SPI_Transmit(spi_handle->Instance, send_buf, send_length);
            }
            if (message->cs_release && (device->config.mode & RT_SPI_3WIRE))
            {
                SPI_Cmd(spi_handle->Instance, Disable);
            }
        }
        else
        {
            rt_memset((uint8_t *)recv_buf, 0xff, send_length);
            if (spi_drv->spi_dma_flag & RT_DEVICE_FLAG_DMA_RX)
            {
                state = hc32_spi_dma(spi_drv->config, RT_NULL, recv_buf, send_length);
            }
            else
            {
                SPI_Cmd(spi_handle->Instance, Enable);
                state = SPI_Receive(spi_handle->Instance, recv_buf, send_length);
            }
        }
        if(state != RT_EOK)
        {
            message->length = 0;
        }
        /* wait spi transfer complete */
        while(Reset != SPI_GetFlag(spi_handle->Instance, SpiFlagSpiIdle));
    }

    if (message->cs_release)
    {
        PORT_ResetPortData((en_port_t)cs->port, (uint16_t)cs->pin);
    }

    return message->length;
}

/**
  * Attach the spi device to SPI bus, this function must be used after initialization.
  */
rt_err_t hc32_hw_spi_device_attach(const char *bus_name,
                                   const char *device_name,
                                   uint8_t    cs_gpio_port,
                                   uint16_t   cs_gpio_pin)
{
    RT_ASSERT(bus_name != RT_NULL);
    RT_ASSERT(device_name != RT_NULL);

    rt_err_t result;
    struct rt_spi_device *spi_device;
    struct hc32_hw_spi_cs *cs_pin;
    stc_port_init_t stcGpioInit;

    MEM_ZERO_STRUCT(stcGpioInit);

//    stcGpioInit.u16PinState = PIN_STATE_SET;
    stcGpioInit.enPinMode = Pin_Mode_Out;
    stcGpioInit.enPullUp = Enable;
    stcGpioInit.enPinDrv = Pin_Drv_H;
    stcGpioInit.enPinOType = Pin_OType_Cmos;
    PORT_Init(SPI1_NSS_PORT, SPI1_NSS_PIN, &stcGpioInit);

    /* attach the device to spi bus*/
    spi_device = (struct rt_spi_device *)rt_malloc(sizeof(struct rt_spi_device));
    RT_ASSERT(spi_device != RT_NULL);
    cs_pin = (struct hc32_hw_spi_cs *)rt_malloc(sizeof(struct hc32_hw_spi_cs));
    RT_ASSERT(cs_pin != RT_NULL);
    cs_pin->port = cs_gpio_port;
    cs_pin->pin = cs_gpio_pin;
    result = rt_spi_bus_attach_device(spi_device, device_name, bus_name, (void *)cs_pin);

    return result;
}

static int hc32_hw_spi_bus_init(void)
{
    rt_err_t result;

    for (int i = 0; i < sizeof(spi_config) / sizeof(spi_config[0]); i++)
    {
        spi_bus_obj[i].config = &spi_config[i];
        spi_bus_obj[i].spi_bus.parent.user_data = &spi_config[i];
        spi_bus_obj[i].handle.Instance = spi_config[i].Instance;
        result = rt_spi_bus_register(&spi_bus_obj[i].spi_bus, spi_config[i].bus_name, &hc32_spi_ops);
    }
    return result;
}

/**
  * @brief  Clear DMA transfer complete flag.
  * @param  dma     specific dam witch spi used.
  * @retval None
  */
static void hc32_dma_irq_handle(struct dma_config *dma)
{
    dma->Instance->INTCLR1 |= (1u << dma->channel);
}


#if defined(BSP_USING_SPI1)
/**
  * @brief  This function handles DMA Rx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi1_rx_dma_irq_handle(void)
{
    #if defined(BSP_SPI1_RX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI1_INDEX].dma_rx);
    SPI_FunctionCmd(spi_config[SPI1_INDEX].Instance, Disable);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif /* BSP_SPI1_RX_USING_DMA */
}

/**
  * @brief  This function handles DMA Tx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi1_tx_dma_irq_handle(void)
{
    #if defined(BSP_SPI1_TX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI1_INDEX].dma_tx);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI1_TX_USING_DMA */
}
#endif  /* BSP_USING_SPI1 */

#if defined(BSP_USING_SPI2)
/**
  * @brief  This function handles DMA Rx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi2_rx_dma_irq_handle(void)
{
    #if defined(BSP_SPI2_RX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI2_INDEX].dma_rx);
    SPI_FunctionCmd(spi_config[SPI2_INDEX].Instance, Disable);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI2_RX_USING_DMA */
}

/**
  * @brief  This function handles DMA Tx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi2_tx_dma_irq_handle(void)
{
    #if defined(BSP_SPI2_TX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI2_INDEX].dma_tx);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI2_TX_USING_DMA */
}
#endif  /* BSP_USING_SPI2 */

#if defined(BSP_USING_SPI3)
/**
  * @brief  This function handles DMA Rx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi3_rx_dma_irq_handle(void)
{
    #if defined(BSP_SPI3_RX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI3_INDEX].dma_rx);
    SPI_FunctionCmd(spi_config[SPI3_INDEX].Instance, Disable);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI3_RX_USING_DMA */
}

/**
  * @brief  This function handles DMA Tx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi3_tx_dma_irq_handle(void)
{
    #if defined(BSP_SPI3_TX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI3_INDEX].dma_tx);


    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI3_TX_USING_DMA */
}
#endif  /* BSP_USING_SPI3 */

#if defined(BSP_USING_SPI4)
/**
  * @brief  This function handles DMA Rx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi4_rx_dma_irq_handle(void)
{
    #if defined(BSP_SPI4_RX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI4_INDEX].dma_rx);
    SPI_FunctionCmd(spi_config[SPI4_INDEX].Instance, Disable);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI4_RX_USING_DMA */
}

/**
  * @brief  This function handles DMA Tx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi4_tx_dma_irq_handle(void)
{
    #if defined(BSP_SPI4_TX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI4_INDEX].dma_tx);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI4_TX_USING_DMA */
}
#endif  /* BSP_USING_SPI4 */

#if defined(BSP_USING_SPI5)
/**
  * @brief  This function handles DMA Rx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi5_rx_dma_irq_handle(void)
{
    #if defined(BSP_SPI5_RX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI5_INDEX].dma_rx);
    SPI_FunctionCmd(spi_config[SPI5_INDEX].Instance, Disable);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI5_RX_USING_DMA */
}

/**
  * @brief  This function handles DMA Tx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi5_tx_dma_irq_handle(void)
{
    #if defined(BSP_SPI5_TX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI5_INDEX].dma_tx);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI5_TX_USING_DMA */
}
#endif  /* BSP_USING_SPI5 */

#if defined(BSP_USING_SPI6)
/**
  * @brief  This function handles DMA Rx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi6_rx_dma_irq_handle(void)
{
    #if defined(BSP_SPI6_RX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI6_INDEX].dma_rx);
    SPI_FunctionCmd(spi_config[SPI6_INDEX].Instance, Disable);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI6_RX_USING_DMA */
}

/**
  * @brief  This function handles DMA Tx complete interrupt request.
  * @param  None
  * @retval None
  */
static void spi6_tx_dma_irq_handle(void)
{
    #if defined(BSP_SPI6_TX_USING_DMA)
    /* enter interrupt */
    rt_interrupt_enter();

    hc32_dma_irq_handle(spi_config[SPI6_INDEX].dma_tx);

    /* leave interrupt */
    rt_interrupt_leave();
    #endif  /* BSP_SPI6_TX_USING_DMA */
}
#endif  /* BSP_USING_SPI6 */

/**
  * @brief  This function gets dma witch spi used infomation include unit,
  *         channel, interrupt etc.
  * @param  None
  * @retval None
  */
static void hc32_get_dma_info(void)
{
#ifdef BSP_SPI1_RX_USING_DMA
    spi_bus_obj[SPI1_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config spi1_dma_rx = SPI1_RX_DMA_CONFIG;
    spi_config[SPI1_INDEX].dma_rx = &spi1_dma_rx;
#endif
#ifdef BSP_SPI1_TX_USING_DMA
    spi_bus_obj[SPI1_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config spi1_dma_tx = SPI1_TX_DMA_CONFIG;
    spi_config[SPI1_INDEX].dma_tx = &spi1_dma_tx;
#endif

#ifdef BSP_SPI2_RX_USING_DMA
    spi_bus_obj[SPI2_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config spi2_dma_rx = SPI2_RX_DMA_CONFIG;
    spi_config[SPI2_INDEX].dma_rx = &spi2_dma_rx;
#endif
#ifdef BSP_SPI2_TX_USING_DMA
    spi_bus_obj[SPI2_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config spi2_dma_tx = SPI2_TX_DMA_CONFIG;
    spi_config[SPI2_INDEX].dma_tx = &spi2_dma_tx;
#endif

#ifdef BSP_SPI3_RX_USING_DMA
    spi_bus_obj[SPI3_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config spi3_dma_rx = SPI3_RX_DMA_CONFIG;
    spi_config[SPI3_INDEX].dma_rx = &spi3_dma_rx;
#endif
#ifdef BSP_SPI3_TX_USING_DMA
    spi_bus_obj[SPI3_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config spi3_dma_tx = SPI3_TX_DMA_CONFIG;
    spi_config[SPI3_INDEX].dma_tx = &spi3_dma_tx;
#endif

#ifdef BSP_SPI4_RX_USING_DMA
    spi_bus_obj[SPI4_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config spi4_dma_rx = SPI4_RX_DMA_CONFIG;
    spi_config[SPI4_INDEX].dma_rx = &spi4_dma_rx;
#endif
#ifdef BSP_SPI4_TX_USING_DMA
    spi_bus_obj[SPI4_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config spi4_dma_tx = SPI4_TX_DMA_CONFIG;
    spi_config[SPI4_INDEX].dma_tx = &spi4_dma_tx;
#endif

#ifdef BSP_SPI5_RX_USING_DMA
    spi_bus_obj[SPI5_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config spi5_dma_rx = SPI5_RX_DMA_CONFIG;
    spi_config[SPI5_INDEX].dma_rx = &spi5_dma_rx;
#endif
#ifdef BSP_SPI5_TX_USING_DMA
    spi_bus_obj[SPI5_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config spi5_dma_tx = SPI5_TX_DMA_CONFIG;
    spi_config[SPI5_INDEX].dma_tx = &spi5_dma_tx;
#endif

#ifdef BSP_SPI6_RX_USING_DMA
    spi_bus_obj[SPI6_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config spi6_dma_rx = SPI6_RX_DMA_CONFIG;
    spi_config[SPI6_INDEX].dma_rx = &spi6_dma_rx;
#endif
#ifdef BSP_SPI6_TX_USING_DMA
    spi_bus_obj[SPI6_INDEX].spi_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config spi6_dma_tx = SPI6_TX_DMA_CONFIG;
    spi_config[SPI6_INDEX].dma_tx = &spi6_dma_tx;
#endif
}

int hc32_hw_spi_init(void)
{
    hc32_get_dma_info();
    return hc32_hw_spi_bus_init();
}

INIT_BOARD_EXPORT(hc32_hw_spi_init);

#endif /* BSP_USING_SPI */
