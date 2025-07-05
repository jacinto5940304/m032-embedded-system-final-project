/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Perform A/D Conversion with ADC single cycle scan mode (3 channels).
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define PLL_CLOCK       50000000

/******************************************************************
 * dataset format setting
 ******************************************************************/

#define train_data_num  			180	//Total number of training data
#define test_data_num 				0	//Total number of testing data

/******************************************************************
 * Network Configuration - customized per network
 ******************************************************************/
#define input_length                    6// The number of input 
#define HiddenNodes_1                    12 // The number of neurons in hidden layer
#define HiddenNodes_2                    12 // The number of neurons in hidden layer

#define target_num                     4 // The number of output 

#define TEST_LENGTH    256
#define ADXL345_ADDR 0x53

volatile uint32_t g_u32AdcIntFlag;
const float LearningRate =       1e-3   ;    // Learning Rate
const float Momentum = 0.9;
const float InitialWeightMax = 0.5;     // Maximum initial weight
const float goal_acc =  0.95          ;    // Target accuracy



volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;
volatile uint8_t g_u8MstTxAbortFlag = 0;
volatile uint8_t g_u8MstRxAbortFlag = 0;
volatile uint8_t g_u8MstReStartFlag = 0;
volatile uint8_t g_u8TimeoutFlag = 0;

volatile uint8_t g_u8DeviceAddr_1;
volatile uint8_t g_au8MstTxData_1[3];
volatile uint8_t g_u8MstRxData_1;
volatile uint8_t g_u8MstDataLen_1;
volatile uint8_t g_u8MstEndFlag_1 = 0;
volatile uint8_t g_u8MstTxAbortFlag_1 = 0;
volatile uint8_t g_u8MstRxAbortFlag_1 = 0;
volatile uint8_t g_u8MstReStartFlag_1 = 0;
volatile uint8_t g_u8TimeoutFlag_1 = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
volatile static I2C_FUNC s_I2C0HandlerFn = NULL;
volatile static I2C_FUNC s_I2C1HandlerFn = NULL;
void I2C0_Init(void);
void I2C1_Init(void);
void ADXL345_WriteRegister_0(uint8_t reg, uint8_t data);
void ADXL345_WriteRegister_1(uint8_t reg, uint8_t data);
void I2C_MasterRx(uint32_t u32Status);
void I2C_MasterTx(uint32_t u32Status);
void I2C_MasterRx_1(uint32_t u32Status);
void I2C_MasterTx_1(uint32_t u32Status);
int16_t x_end0,y_end0,z_end0;
int16_t x_end1,y_end1,z_end1;

volatile uint32_t g_timer0Count = 0;  
volatile uint32_t g_timer1Count = 0;  // Global counter
volatile uint8_t g_paused = 0;  // state change


void TMR0_IRQHandler(void)
{
    if (TIMER_GetIntFlag(TIMER0))
    {
        TIMER_ClearIntFlag(TIMER0);
        g_timer0Count += 1;
    }
}



/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

void I2C1_IRQHandler(void)
{
    uint32_t u32Status = I2C_GET_STATUS(I2C1);
    if (I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        I2C_ClearTimeoutFlag(I2C1);
        g_u8TimeoutFlag_1 = 1;
    }
    else if (s_I2C1HandlerFn)
    {
        s_I2C1HandlerFn(u32Status);
    }
}



/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 2)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, ((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData = (unsigned char) I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;
        while(I2C0->CTL0 & I2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}

void I2C_MasterRx_1(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C1, (g_u8DeviceAddr_1 << 1));    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C1, g_au8MstTxData_1[g_u8MstDataLen_1++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen_1 != 2)
        {
            I2C_SET_DATA(I2C1, g_au8MstTxData_1[g_u8MstDataLen_1++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C1, ((g_u8DeviceAddr_1 << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData_1 = (unsigned char) I2C_GET_DATA(I2C1);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
        g_u8MstEndFlag_1 = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag_1 = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;
        while(I2C1->CTL0 & I2C_CTL0_SI_Msk)//***
            if(--u32TimeOutCnt == 0) break;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen != 3)
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            I2C_SET_DATA(I2C0, (uint32_t)((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;
        while(I2C0->CTL0 & I2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}

void I2C_MasterTx_1(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C1, g_u8DeviceAddr_1 << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C1, g_au8MstTxData_1[g_u8MstDataLen_1++]);
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C1);
        I2C_START(I2C1);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MstDataLen_1 != 3)
        {
            I2C_SET_DATA(I2C1, g_au8MstTxData_1[g_u8MstDataLen_1++]);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            g_u8MstEndFlag_1 = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Press any key to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            I2C_SET_DATA(I2C1, (uint32_t)((g_u8DeviceAddr_1 << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag_1 = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        u32TimeOutCnt = SystemCoreClock;
        while(I2C1->CTL0 & I2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
}




void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable HIRC clock (Internal RC 48MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK source divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Enable UART0 clock */
CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    CLK_EnableModuleClock(TMR0_MODULE);
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD=PB.12 and TXD=PB.13 */
    SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
                    (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);

    /* Set I2C0 multi-function pins */
    SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk)) |
                    (SYS_GPB_MFPL_PB4MFP_I2C0_SDA | SYS_GPB_MFPL_PB5MFP_I2C0_SCL);

    SYS->GPB_MFPL = (SYS->GPB_MFPL
                     & ~(SYS_GPB_MFPL_PB2MFP_Msk
                        | SYS_GPB_MFPL_PB3MFP_Msk))
                    | (SYS_GPB_MFPL_PB2MFP_I2C1_SDA 
                     |  SYS_GPB_MFPL_PB3MFP_I2C1_SCL); 

    /* Lock protected registers */
    SYS_LockReg();
}


void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);
    /* Enable I2C interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Set I2C 4 Slave Addresses */
    //I2C_SetSlaveAddr(I2C0, 0, 0x53, 0);   /* Slave Address : 0x15 */
    ADXL345_WriteRegister_0(0x2D,0x08);
    ADXL345_WriteRegister_0(0x31,0x0B);
    ADXL345_WriteRegister_0(0x38,0x80);
    printf("hefushkldas");
    // I2C_SetSlaveAddr(I2C0, 1, 0x35, 0);   /* Slave Address : 0x35 */
    // I2C_SetSlaveAddr(I2C0, 2, 0x55, 0);   /* Slave Address : 0x55 */
    // I2C_SetSlaveAddr(I2C0, 3, 0x75, 0);   /* Slave Address : 0x75 */
}

void I2C1_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C1, 100000);
    /* Enable I2C interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C 4 Slave Addresses */
    //I2C_SetSlaveAddr(I2C1, 0, 0x53, 0);   /* Slave Address : 0x15 */
    ADXL345_WriteRegister_1(0x2D,0x08);
    ADXL345_WriteRegister_1(0x31,0x0B);
    ADXL345_WriteRegister_1(0x38,0x80);
    printf("jhfjwef\n");
    // I2C_SetSlaveAddr(I2C1, 1, 0x35, 0);   /* Slave Address : 0x35 */
    // I2C_SetSlaveAddr(I2C1, 2, 0x55, 0);   /* Slave Address : 0x55 */
    // I2C_SetSlaveAddr(I2C1, 3, 0x75, 0);   /* Slave Address : 0x75 */
}



void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}

void I2C1_Close(void)
{
    /* Disable I2C1 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C1);
    NVIC_DisableIRQ(I2C1_IRQn);

    /* Disable I2C1 and close I2C1 clock */
    I2C_Close(I2C1);
    CLK_DisableModuleClock(I2C1_MODULE);

}

int32_t I2C0_Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    do
    {
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C0, 0);
        g_u8MstReStartFlag = 0;
        g_u8DeviceAddr = slvaddr;

        g_u8TimeoutFlag = 0;
        for(i = 0; i < TEST_LENGTH; i++)
        {
            g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);

            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;

            /* I2C function to write data to slave */
            s_I2C0HandlerFn = I2C_MasterTx;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    printf(" MasterTx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterTx abort flag*/
                    g_u8MstTxAbortFlag = 1;
                }
            }
            while(g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);
            g_u8MstEndFlag = 0;

            if(g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag*/
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* I2C function to read data from slave */
            s_I2C0HandlerFn = I2C_MasterRx;

            g_u8MstDataLen = 0;
            g_u8DeviceAddr = slvaddr;

            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterRx abort flag*/
                    g_u8MstRxAbortFlag = 1;
                }
            }
            while(g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstRxAbortFlag)
            {
                /* Clear MasterRx abort flag*/
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }
            /* Compare data */
            if(g_u8MstRxData != g_au8MstTxData[2])
            {
                /* Disable I2C timeout */
                I2C_DisableTimeout(I2C0);
                printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
                return -1;
            }
        }
    }
    while(g_u8MstReStartFlag);   /*If unexpected abort happens, re-start the transmition*/

    /* Disable I2C timeout */
    I2C_DisableTimeout(I2C0);
    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

int32_t I2C1_Read_Write_SLAVE(uint8_t slvaddr)
{
    uint32_t i;

    do
    {
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C1, 0);
        g_u8MstReStartFlag_1 = 0;
        g_u8DeviceAddr_1 = slvaddr;

        g_u8TimeoutFlag_1 = 0;
        for(i = 0; i < TEST_LENGTH; i++)
        {
            g_au8MstTxData_1[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData_1[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData_1[2] = (uint8_t)(g_au8MstTxData_1[1] + 3);

            g_u8MstDataLen_1 = 0;
            g_u8MstEndFlag_1 = 0;

            /* I2C function to write data to slave */
            s_I2C1HandlerFn = I2C_MasterTx_1;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag_1)
                {
                    printf(" MasterTx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C1RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C1_Init();
                    /* Set MasterTx abort flag*/
                    g_u8MstTxAbortFlag_1 = 1;
                }
            }
            while(g_u8MstEndFlag_1 == 0 && g_u8MstTxAbortFlag_1 == 0);
            g_u8MstEndFlag_1 = 0;

            if(g_u8MstTxAbortFlag_1)
            {
                /* Clear MasterTx abort flag*/
                g_u8MstTxAbortFlag_1 = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag_1 = 1;
                break;
            }

            /* I2C function to read data from slave */
            s_I2C1HandlerFn = I2C_MasterRx_1;

            g_u8MstDataLen_1 = 0;
            g_u8DeviceAddr_1 = slvaddr;

            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag_1)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out!! Press any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C1RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C1_Init();
                    /* Set MasterRx abort flag*/
                    g_u8MstRxAbortFlag_1 = 1;
                }
            }
            while(g_u8MstEndFlag_1 == 0 && g_u8MstRxAbortFlag_1 == 0);

            g_u8MstEndFlag_1 = 0;

            if(g_u8MstRxAbortFlag_1)
            {
                /* Clear MasterRx abort flag*/
                g_u8MstRxAbortFlag_1 = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag_1 = 1;
                break;
            }
            /* Compare data */
            if(g_u8MstRxData_1 != g_au8MstTxData_1[2])
            {
                /* Disable I2C timeout */
                I2C_DisableTimeout(I2C1);
                printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData_1);
                return -1;
            }
        }
    }
    while(g_u8MstReStartFlag_1);   /*If unexpected abort happens, re-start the transmition*/

    /* Disable I2C timeout */
    I2C_DisableTimeout(I2C1);
    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

void ADXL345_WriteRegister_0(uint8_t reg, uint8_t data)
{
    g_u8DeviceAddr = ADXL345_ADDR;
    g_au8MstTxData[0] = reg;
    g_au8MstTxData[1] = data;
    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;
    s_I2C0HandlerFn = I2C_MasterTx;
   // printf("wefbjkef");

    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);
   // printf("wefbjkesdsf");
    while(g_u8MstEndFlag == 0);
   // printf("wefbjkesdsf");
}

void ADXL345_WriteRegister_1(uint8_t reg, uint8_t data)
{
    g_u8DeviceAddr_1 = ADXL345_ADDR;
    g_au8MstTxData_1[0] = reg;
    g_au8MstTxData_1[1] = data;
    g_u8MstDataLen_1 = 0;
    g_u8MstEndFlag_1 = 0;
    s_I2C1HandlerFn = I2C_MasterTx_1;
   // printf("wefbjkef");

    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);
   // printf("wefbjkesdsf");
    while(g_u8MstEndFlag_1 == 0);
   // printf("wefbjkesdsf");
}

uint8_t ADXL345_ReadRegister_0(uint8_t reg)
{
    g_au8MstTxData[0] = reg;
    g_u8MstDataLen = 0;
    g_u8MstEndFlag = 0;
    s_I2C0HandlerFn = I2C_MasterRx;
    I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);
    while(g_u8MstEndFlag == 0);
    return g_u8MstRxData;
}

uint8_t ADXL345_ReadRegister_1(uint8_t reg)
{
    g_au8MstTxData_1[0] = reg;
    g_u8MstDataLen_1 = 0;
    g_u8MstEndFlag_1 = 0;
    s_I2C1HandlerFn = I2C_MasterRx_1;
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STA);
    while(g_u8MstEndFlag_1 == 0);
    return g_u8MstRxData_1;
}

void ACC_VALUE_0(void){
    uint8_t x0,x1,y0,y1,z0,z1;
    x0 = ADXL345_ReadRegister_0(0x32);
    x1 = ADXL345_ReadRegister_0(0x33);
    y0 = ADXL345_ReadRegister_0(0x34);
    y1 = ADXL345_ReadRegister_0(0x35);
    z0 = ADXL345_ReadRegister_0(0x36);
    z1 = ADXL345_ReadRegister_0(0x37);
    x_end0 = (int16_t)((x1 << 8) | x0);
    y_end0 = (int16_t)((y1 << 8) | y0);
    z_end0 = (int16_t)((z1 << 8) | z0);
    return;
}

void ACC_VALUE_1(void){
    uint8_t x0,x1,y0,y1,z0,z1;
    x0 = ADXL345_ReadRegister_1(0x32);
    x1 = ADXL345_ReadRegister_1(0x33);
    y0 = ADXL345_ReadRegister_1(0x34);
    y1 = ADXL345_ReadRegister_1(0x35);
    z0 = ADXL345_ReadRegister_1(0x36);
    z1 = ADXL345_ReadRegister_1(0x37);
    x_end1 = (int16_t)((x1 << 8) | x0);
    y_end1 = (int16_t)((y1 << 8) | y0);
    z_end1 = (int16_t)((z1 << 8) | z0);
    return;
}

// Create train dataset/output
float train_data_input[train_data_num][input_length] = {
    4.07 , -0.56 , 0.56 , 3.88 , -0.37 , 0.55 , 
 4.47 , -1.59 , 1.13 , 3.11 , 1.30 , -1.67 , 
 3.78 , 0.98 , 0.05 , 4.02 , -2.13 , 0.80 , 
 3.94 , -1.50 , 0.77 , 3.91 , -0.80 , 0.29 , 
 3.93 , 0.30 , 0.30 , 3.42 , -1.05 , 0.70 ,
 3.64 , 0.02 , 0.48 , 3.05 , -0.35 , 0.58 , 
 5.49 , 0.79 , 2.80 , 2.57 , -1.16 , -0.14 ,
 3.45 , 0.05 , -1.23 , 3.33 , -0.68 , 0.88 , 
 3.76 , -1.31 , -0.76 , 2.92 , 1.36 , 0.59 ,
 3.91 , -0.39 , 0.10 , 2.91 , -0.79 , 0.31 ,
 3.53 , -0.57 , 0.48 , 3.79 , 0.59 , 1.36 , 
 5.04 , 1.60 , -0.21 , 3.64 , 1.69 , -3.26 ,
 5.66 , 1.85 , -1.45 , 3.05 , -0.70 , 0.75 , 
 3.60 , -1.27 , -1.25 , 3.59 , -0.74 , 0.43 ,
 3.93 , -0.71 , 0.41 , 3.95 , -0.21 , 0.99 , 
 4.87 , -0.36 , 2.31 , 3.63 , -0.23 , 2.57 , 
 3.43 , -0.75 , -1.94 , 3.04 , 0.22 , 0.15 , 
 3.23 , -0.34 , 5.23 , 3.32 , -1.33 , 0.52 , 
 3.38 , -1.27 , 0.40 , 3.06 , 0.01 , 0.67 , 
 4.40 , -0.14 , 0.36 , 4.24 , -1.41 , 0.79 , 
 5.91 , 1.57 , 4.86 , 6.65 , -2.39 , 2.72 , 
 4.58 , 0.05 , 0.64 , 2.66 , 0.70 , -0.75 , 
 4.71 , 0.03 , 3.77 , 5.21 , -2.24 , 4.84 , 
 5.17 , 0.28 , 4.70 , 4.80 , -2.83 , 3.38 , 
 3.52 , -0.39 , -0.35 , 2.84 , 0.67 , -0.59 , 
 3.42 , -0.83 , 0.13 , 3.25 , -2.64 , 2.55 ,
 3.91 , -0.63 , 0.83 , 3.86 , 0.17 , 0.75 , 
 5.82 , 1.70 , 1.60 , 3.72 , -0.30 , -0.51 , 
 3.92 , -0.63 , 0.32 , 3.57 , -0.22 , 0.50 , 
 5.72 , 2.74 , 2.60 , 4.32 , -2.54 , 1.75 , 
 3.43 , -0.38 , -1.41 , 2.49 , 0.46 , 0.19 ,
 3.26 , -0.04 , 0.86 , 3.66 , 0.79 , -0.04 , 
 5.32 , 0.59 , 2.11 , 3.77 , -1.00 , -0.73 , 
 5.20 , 1.67 , -0.40 , 10.21 , -3.02 , -0.30 ,
 4.32 , -0.57 , 0.51 , 2.84 , 0.69 , 0.18 , 
 4.67 , 0.62 , 2.83 , 4.04 , -2.36 , 0.65 , 
 4.38 , 0.52 , -2.66 , 2.40 , -0.23 , -1.13 , 
 5.43 , -0.79 , 3.77 , 5.00 , -1.73 , 4.90 ,
 3.50 , -1.59 , 0.83 , 3.92 , 1.00 , 1.48 , 
 3.58 , -1.88 , -0.38 , 3.56 , 0.75 , 0.12 , 
 5.39 , -0.21 , 0.47 , 4.25 , -1.07 , 0.60 , 
 4.11 , -0.72 , -0.36 , 4.22 , -0.39 , 0.32 , 
 5.15 , 0.41 , -0.02 , 5.40 , -0.76 , 0.71 , 
 3.94 , -1.33 , -5.22 , 2.16 , 2.13 , -0.95 , 
 3.53 , 0.27 , 1.22 , 2.11 , 0.07 , 0.04 ,  //45 jump data

 4.55, -1.07, 1.73, 3.50, -0.01, 0.57,
4.09, -1.70, 0.43, 4.14, 0.13, 0.53,
3.82, 0.04, 3.05, 5.03, -2.96, 0.84,
4.45, 0.08, 0.82, 4.62, -0.88, 1.38,
5.35, -0.62, 0.41, 3.86, 0.62, 0.50,
3.88, -0.00, -1.10, 2.53, 1.10, -0.49,
4.38, -1.99, 1.09, 4.91, 1.09, 0.80,
3.11, -2.12, -0.29, 3.57, 0.35, 0.49,
4.61, -0.98, 1.03, 5.10, 0.86, 0.69,
4.59, -0.50, -0.79, 4.44, 0.25, 1.48,
3.93, -1.26, 0.12, 4.70, 0.56, 0.63,
3.49, -0.50, 0.80, 3.50, -0.16, 1.79,
3.52, -0.10, 0.95, 4.97, -5.66, 3.73,
4.70, 0.88, 2.35, 4.42, -1.94, 1.66,
4.21, -1.39, 2.14, 4.55, -0.07, 1.11,
3.36, -0.36, 0.35, 2.35, 0.23, 0.41,
5.79, -0.12, -0.88, 4.34, -0.04, 0.51,
4.01, -1.82, 0.29, 5.54, 0.88, 0.68,
4.71, -0.85, -0.02, 3.46, -0.78, 3.95,
3.72, -2.27, -0.23, 3.93, -0.02, 0.23,
4.47, 1.07, -1.07, 3.85, 1.85, 2.01,
5.24, -1.66, 0.74, 5.03, 0.64, 0.84,
4.79, -2.24, 1.60, 4.28, -0.76, 1.33,
4.54, -0.87, 1.08, 5.25, -0.25, 0.89,
4.94, -1.77, 1.71, 3.48, 0.04, 0.87,
3.57, -2.02, -0.67, 2.80, 0.40, 0.18,
7.31, 1.89, 3.37, 3.30, -1.21, 1.88,
3.96, -0.13, 0.51, 2.99, -0.51, 0.58,
4.37, 0.22, 0.21, 5.25, -1.08, 3.48,
4.45, -1.18, 0.39, 5.39, 0.18, 0.30,
4.95, -1.32, 1.49, 4.36, -0.30, 1.00,
5.49, -1.61, 1.61, 4.49, -0.24, 1.45,
5.55, -2.33, 2.48, 6.50, -1.79, 1.71,
5.67, -0.66, 0.18, 5.50, -0.24, 2.39,
4.91, -0.57, 0.75, 4.48, 0.48, 0.76,
3.76, -2.61, -0.98, 2.89, 0.95, -0.68,
5.60, -1.31, 0.95, 3.73, -0.86, 1.39,
3.41, -0.76, 0.79, 5.48, -0.47, 2.51,
4.32, -1.57, 1.29, 4.49, 0.49, -0.18,
4.08, 0.86, -2.24, 3.22, 0.95, -0.50,
5.42, -2.01, 1.73, 4.23, -0.23, 2.27,
3.88, 0.27, 0.24, 3.50, 1.93, -0.05,
3.04, 1.00, -1.43, 5.63, -2.01, 2.51,
4.35, -2.54, 2.25, 2.65, 1.19, 0.64,
3.54, -0.34, -0.32, 1.95, 1.12, 1.03,// 45 open_jump_data

1.88 , -0.80 , 1.70 , 0.42 , 0.84 , 0.25 ,
1.70 , 0.30 , -0.38 , 0.25 , 1.57 , -1.10 ,
2.02 , -1.02 , 1.36 , -0.04 , 0.63 , -0.31 ,
2.61 , 0.04 , 1.43 , 0.39 , 1.13 , -0.33 ,
3.11 , 0.58 , 0.85 , 0.82 , -0.19 , -0.80 ,
3.32 , 0.02 , 0.88 , 2.23 , -1.77 , 0.98 ,
3.74 , 2.17 , 0.85 , 0.93 , -0.72 , 0.50 ,
3.64 , 0.93 , 0.45 , 0.52 , 1.82 , -1.00 ,
3.88 , 2.07 , 0.83 , 1.20 , -0.89 , 0.38 ,
2.58 , -0.15 , 0.68 , 3.59 , -0.41 , 1.02 ,
2.59 , 0.26 , -0.02 , 0.93 , -0.30 , -0.06 ,
2.87 , 0.52 , 0.37 , 4.65 , -1.38 , 3.85 ,
2.32 , 0.46 , 1.75 , 1.24 , 0.36 , -0.05 ,
2.77 , 0.96 , -0.18 , 4.61 , -1.01 , 4.45 ,
2.43 , 0.29 , 0.18 , 2.57 , -1.46 , 1.95 ,
2.45 , 0.29 , 0.71 , 2.11 , -1.27 , 0.18 ,
3.09 , 1.52 , 1.29 , 0.13 , 1.07 , -0.16 ,
3.99 , 1.57 , 1.17 , 1.13 , -0.59 , 0.38 ,
2.54 , 0.75 , 0.66 , 4.07 , -1.82 , 1.82 ,
2.45 , 0.29 , 0.71 , 2.11 , -1.27 , 0.18 ,
2.89 , -0.63 , 0.46 , 1.65 , -0.77 , 0.40 ,
2.51 , 0.23 , 0.73 , 3.91 , -1.36 , 0.42 ,
3.30 , 1.27 , 0.04 , 0.82 , -0.05 , 0.66 ,
3.02 , 0.32 , 0.49 , 2.87 , -0.12 , 1.61 ,
2.70 , 0.91 , 0.38 , 2.86 , 0.98 , 2.32 ,
2.10 , 0.95 , 0.67 , 0.50 , 0.50 , -0.30 ,
2.31 , 0.77 , 0.05 , 1.88 , 0.15 , -1.82 ,
2.21 , 1.31 , -0.27 , 0.88 , 0.82 , -1.82 ,
2.52 , 0.28 , 0.23 , 1.12 , 0.97 , 0.74 ,
2.46 , 0.61 , 0.53 , 4.78 , -1.73 , 0.64 ,
2.79 , 1.17 , 0.18 , 1.23 , -0.45 , 0.67 ,
3.07 , -0.17 , 0.47 , 1.70 , -1.97 , -0.17 ,
2.86 , 1.24 , -0.15 , 1.43 , -1.14 , 0.69 ,
2.88 , 0.73 , 0.13 , 2.60 , -0.71 , -0.45 ,
3.46 , 0.98 , 0.88 , 1.38 , -0.62 , -0.29 ,
3.36 , 1.54 , -0.63 , 1.55 , -0.71 , 0.63 ,
2.75 , 0.67 , 0.43 , 4.12 , -1.18 , 2.41 ,
4.36 , 0.71 , 2.07 , 1.70 , -0.52 , 0.11 ,
2.69 , 0.94 , 0.46 , 1.31 , -0.73 , 0.79 ,
2.57 , 1.20 , 0.34 , 2.29 , -1.08 , -0.72 ,
3.35 , 0.39 , 2.24 , 1.88 , -0.55 , 0.38 ,
2.43 , 1.00 , 0.55 , 2.72 , -1.86 , 0.39 ,
2.18 , 1.12 , 1.13 , 1.99 , -0.80 , -0.27 ,
1.91 , -0.44 , 1.16 , 0.64 , -0.22 , 0.11 ,
2.52 , 0.73 , 0.02 , 3.58 , -0.11 , 0.79 ,//45 run

 1.28 ,0.29 ,  0.98  , 1.09 , -0.21 ,  -0.01 , 
 1.14 ,0.23 ,  -0.00  , 0.56 , 0.09 ,  -0.07 , 
 1.20 ,0.28 ,  0.26  , 0.89 , -0.15 ,  0.82 , 
 1.32 ,0.32 ,  -0.12  , 1.01 , -0.02 ,  0.18 , 
 2.21 ,0.32 ,  1.66  , 1.16 , -0.51 ,  0.11 , 
 1.95 ,-0.36 ,  0.19  , 1.16 , -0.58 ,  0.27 , 
 1.45 ,0.89 ,  -0.05  , 1.23 , 0.06 ,  -0.16 , 
 1.46 ,0.13 ,  1.20  , 1.13 , -0.28 ,  0.13 , 
 1.28 ,-0.01 ,  0.09  , 1.62 , -0.20 ,  1.11 , 
 1.65 ,-0.17 ,  1.14  , 1.10 , -0.30 ,  0.14 ,
 1.23 ,0.03 ,  0.19  , 1.76 , -0.31 ,  1.33 , 
 1.30 ,0.21 ,  0.02  , 1.32 , -0.52 ,  -0.70 ,
 1.34 ,0.13 ,  1.15  , 1.15 , -0.41 ,  0.21 ,
 1.48 ,0.22 ,  1.13  , 1.11 , -0.30 ,  0.23 ,
 1.38 ,-0.26 ,  -0.10  , 1.15 , -0.54 ,  0.21 ,
 1.29 ,0.10 ,  0.26  , 0.27 , 0.20 ,  -0.05 , 
 1.29 ,-0.12 ,  -0.53  , 0.78 , 0.02 ,  -0.53 , 
 1.57 ,-0.07 ,  -0.02  , 1.20 , -0.40 ,  0.34 ,
 1.38 ,0.23 ,  -0.69  , 0.88 , -0.58 ,  -0.17 ,
 1.27 ,0.13 ,  0.18  , 2.30 , -0.71 ,  1.45 ,
 1.23 ,0.16 ,  0.49  , 1.21 , 0.12 ,  -0.66 ,
 1.46 ,0.24 ,  -0.36  , 1.17 , -0.96 ,  -0.18 ,
 1.01 ,-0.20 ,  0.45  , 1.08 , -0.41 ,  0.02 ,
 1.09 ,0.17 ,  0.11  , 0.71 , -0.13 ,  -0.08 ,
 1.31 ,0.14 ,  0.16  , 2.12 , -0.86 ,  1.42 ,
 1.51 ,0.18 ,  0.31  , 1.46 , -0.46 ,  0.13 ,
 1.51 ,0.18 ,  0.31  , 1.46 , -0.46 ,  0.13 , 
 1.29 ,-0.41 ,  -0.07  , 1.12 , -0.51 ,  0.24 , 
 1.32 ,0.34 ,  0.15  , 0.98 , 0.29 ,  -0.18 ,
 1.24 ,0.07 ,  0.07  , 1.85 , -0.48 ,  1.45 ,
 1.32 ,0.15 ,  0.43  , 1.59 , -0.16 ,  0.25 , 
 1.37 ,0.19 ,  -0.17  , 1.04 , -0.84 ,  -0.05 , 
 1.34 ,0.24 ,  0.46  , 1.25 , 0.27 ,  -0.62 ,
 1.45 ,0.14 ,  -0.49  , 0.97 , -0.79 ,  -0.25 , 
 1.32 ,0.23 ,  0.40  , 1.52 , 0.09 ,  -0.27 , 
 1.32 ,0.02 ,  -0.25  , 1.08 , -0.86 ,  0.08 ,  
 1.32 ,0.29 ,  0.33  , 1.45 , 0.11 ,  -0.46 ,
 1.34 ,-0.13 ,  0.00  , 1.06 , -0.85 ,  -0.18 ,
 1.32 ,0.39 ,  0.29  , 1.14 , 0.45 ,  -0.75 ,
 1.32 ,0.09 ,  -0.24  , 1.01 , -0.84 ,  -0.01 ,
 1.34 ,0.20 ,  0.32  , 0.52 , 0.21 ,  -0.80 , 
 1.55 ,-0.25 ,  0.10  , 1.09 , -0.62 ,  0.22 ,  
 1.40 ,-0.09 ,  -0.98  , 0.57 , 0.17 ,  -0.72 , 
 1.29 ,0.36 ,  0.25  , 1.36 , 0.03 ,  -0.87 , 
 1.43 ,-0.01 ,  -0.63  , 1.31 , -0.38 ,  0.12 ,  //45 walk



};	    // You can put your train dataset here
int train_data_output[train_data_num ][target_num] = {
    1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
1,0,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,1,0,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,1,0,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
0,0,0,1,
};      // Label of the train data

// Create test dataset/output
float test_data_input[test_data_num][input_length] = {
	
    

};		// You can put your test dataset here

int test_data_output[test_data_num][target_num] = {
 

};		// Label of the test data

int ReportEvery10;
int RandomizedIndex[train_data_num];
long  TrainingCycle;
float Rando;
float Error;
float Accum;

float data_mean[6] ={0};
float data_std[6] ={0};

float Hidden_1[HiddenNodes_1];
float Hidden_2[HiddenNodes_2];
float Output[target_num+1];

float HiddenWeights_1[input_length+1][HiddenNodes_1] = {
{-2.938277, 0.297808, 2.797866, -4.075537, -3.221987, -1.811113, 0.640244, 0.235774, 2.327744, 0.951585, 0.529274, -2.058649},
  {-0.866018, -1.041687, -0.370362, -0.149291, 1.877218, 0.542929, 0.269909, -1.131223, -1.800509, 1.178880, -2.058179, -0.076976},
  {-0.608723, 1.158003, 2.980884, 0.063008, 0.167454, 0.355564, -0.761800, -1.064947, 0.244102, 0.981965, 1.253184, 1.536057},
  {-2.234524, 0.266681, 0.748719, 0.531063, -2.095236, -1.178408, 0.381948, -0.166158, 1.261756, -0.099976, -0.018220, -0.680066},
  {-0.088770, 1.356299, -1.244663, -1.993874, -0.517851, 0.477676, -0.951627, 2.243024, -0.120158, -1.884620, -1.816771, -1.106045},
  {-0.673815, 2.177164, 0.012333, -0.272116, 0.370012, 0.859234, -1.810903, 0.474684, -0.835056, -0.352109, 1.861688, 0.776864},
  {3.085373, 0.184098, 1.547206, -2.925614, -0.382503, -0.005035, 1.844707, -1.657319, 0.687421, -0.184592, -2.307621, -0.997294}
};
float HiddenWeights_2[HiddenNodes_1+1][HiddenNodes_2] = {
{-1.048570, 3.607911, -1.047962, 0.320694, -1.868399, -0.636462, -0.274419, -0.350094, 0.551368, -1.750556, 1.351897, -0.879272},
  {0.299616, -1.380875, -0.187560, 0.947003, 1.298981, 0.314472, 0.150031, 1.264864, -0.039148, 0.676923, -0.955873, 0.297914},
  {-0.337484, 0.979884, -2.118822, 2.534082, -1.347135, 0.832778, 0.588981, 3.068900, -0.217018, -0.519016, 0.333238, -0.371751},
  {0.967278, -1.060420, 2.519133, -2.656328, 1.446489, -1.703484, -0.590488, -2.618700, 0.979543, 0.502207, 0.956629, 1.529404},
  {-0.874435, 0.056897, -1.182478, 0.861585, 0.613817, -2.768748, 1.386211, 0.070188, 1.372884, 0.121656, 2.288256, -0.133718},
  {-0.715809, 1.032698, -0.261932, -0.410392, 0.819260, -1.990218, 0.119088, -0.371220, 1.262361, -0.304601, 1.207018, 0.332374},
  {0.577947, -1.487651, 0.964957, -0.224503, 1.298798, 0.590680, -0.610186, 0.217807, -0.491955, 1.056496, -1.110446, 1.009253},
  {0.510623, -1.906802, 0.658795, 0.466495, 1.486986, -0.211191, 0.138277, 1.104189, -0.452190, 1.407808, -0.516778, -0.227417},
  {0.120682, 0.462152, 0.652377, 1.190699, -0.893581, 3.180247, -0.210149, 2.916551, -1.451315, -0.828469, -2.407559, -0.363709},
  {-0.548992, 1.379167, -1.078008, 0.867406, -0.980590, 0.297030, -0.222462, 0.948851, -0.152628, -0.663213, -0.327277, -0.598174},
  {0.719320, -2.979386, 0.597422, 0.032758, 0.880424, 1.069648, 0.494375, 0.997175, -0.811578, 1.030620, -0.848235, 0.173962},
  {-0.693344, 1.933278, 0.443566, -0.960418, -0.057887, -1.866686, -0.777750, -2.342816, 0.429733, -0.426685, 0.726216, -0.316694},
  {-0.096976, 0.952420, 0.272132, 0.479331, -0.028195, 0.626290, 0.209008, 1.840000, -0.380648, -0.273566, -0.620369, 0.126994}
};
float OutputWeights[HiddenNodes_2+1][target_num] = {
  {-1.166333, 0.946206, -2.435313, -0.324787},
  {4.071312, -4.785336, 0.808176, 0.512022},
  {-0.955549, 1.515448, -4.156406, 1.239232},
  {-0.248558, -0.498034, 2.521367, -2.978022},
  {-2.530963, 2.876942, -1.239670, 0.805891},
  {3.134419, -0.274091, -5.151176, -2.033908},
  {-1.705303, -0.680969, 0.606007, -1.017475},
  {0.844776, 0.956127, 3.360234, -5.058348},
  {-1.894481, -0.375928, 0.920012, 1.502339},
  {-2.434183, 1.663117, -0.902788, -0.135441},
  {-3.772162, -1.901227, 1.323177, 2.025384},
  {-1.719835, 0.317395, -1.577534, 1.233346},
  {-0.713479, -0.493903, -2.171858, -0.979779}
};

float HiddenDelta_1[HiddenNodes_1];
float HiddenDelta_2[HiddenNodes_2];

float OutputDelta[target_num];
float ChangeHiddenWeights_1[input_length+1][HiddenNodes_1];
float ChangeHiddenWeights_2[HiddenNodes_1+1][HiddenNodes_2];
float ChangeOutputWeights[HiddenNodes_2+1][target_num];

int target_value;
int out_value;
int max;

// void SYS_Init(void)
// {
//     SYS_UnlockReg();
//     CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
//     CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);
//     CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
//     CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);
//     CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
//     CLK_EnableModuleClock(UART0_MODULE);
//     CLK_EnableModuleClock(ADC_MODULE);
//     CLK_SetModuleClock(ADC_MODULE, CLK_CLKSEL2_ADCSEL_PCLK1, CLK_CLKDIV0_ADC(1));
//     SystemCoreClockUpdate();
//     SYS->GPB_MFPH = (SYS->GPB_MFPH & ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk)) |
//                     (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
//     GPIO_SetMode(PB, BIT0 | BIT1 | BIT2, GPIO_MODE_INPUT);
//     SYS->GPB_MFPL = (SYS->GPB_MFPL & ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk | SYS_GPB_MFPL_PB2MFP_Msk)) |
//                     (SYS_GPB_MFPL_PB0MFP_ADC0_CH0 | SYS_GPB_MFPL_PB1MFP_ADC0_CH1 | SYS_GPB_MFPL_PB2MFP_ADC0_CH2);
//     GPIO_DISABLE_DIGITAL_PATH(PB, BIT0 | BIT1 | BIT2);
//     SYS_LockReg();
// }

// void ADC_IRQHandler(void)
// {
//     g_u32AdcIntFlag = 1;
//     ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
// }

// void UART0_Init(void)
// {
//     SYS_ResetModule(UART0_RST);
//     UART_Open(UART0, 115200);
// }

void scale_data()
{
		float sum[input_length] = {0};
		int i, j;

		// Compute Data Mean
		for(i = 0; i < train_data_num; i++){
			for(j = 0; j < input_length; j++){
				sum[j] += train_data_input[i][j];
			}
		}
		for(j = 0; j < input_length ; j++){
			data_mean[j] = sum[j] / train_data_num;
			printf("MEAN: %.2f\n", data_mean[j]);
			sum[j] = 0.0;
		}

		// Compute Data STD
		for(i = 0; i < train_data_num; i++){
			for(j = 0; j < input_length ; j++){
				sum[j] += pow(train_data_input[i][j] - data_mean[j], 2);
			}
		}
		for(j = 0; j < input_length; j++){
			data_std[j] = sqrt(sum[j]/train_data_num);
			printf("STD: %.2f\n", data_std[j]);
			sum[j] = 0.0;
		}
}

void normalize(float *data)
{
		int i;
        data_mean[0] = 3.22;
        data_mean[1] = -0.05;
        data_mean[2] = 0.53;
        data_mean[3] = 2.75;
        data_mean[4] = -0.34;
        data_mean[5] = 0.56;
        data_std[0] = 3.52;
        data_std[1] = 0.98;
        data_std[2] = 1.33;
        data_std[3] = 3.21;
        data_std[4] = 1.10;
        data_std[5] = 1.30;

		for(i = 0; i < input_length; i++){
			data[i] = (data[i] - data_mean[i]) / data_std[i];
		}
}

int train_preprocess()
{
    int i;

    for(i = 0 ; i < train_data_num ; i++)
    {
        normalize(train_data_input[i]);
    }

    return 0;
}

int test_preprocess()
{
    int i;

    for(i = 0 ; i < test_data_num ; i++)
    {
        normalize(test_data_input[i]);
    }

    return 0;
}




int data_setup()
{
    int i;
		//int j;
		int p, ret;
	    uint32_t u32ChannelCount;
    int32_t i32ConversionData[3];
		unsigned int seed = 1;
    printf("ADC started...\n");
	ADC_POWER_ON(ADC);
				ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, BIT0 | BIT1 | BIT2);
        ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);
        ADC_ENABLE_INT(ADC, ADC_ADF_INT);
        NVIC_EnableIRQ(ADC_IRQn);
        g_u32AdcIntFlag = 0;
        ADC_START_CONV(ADC);
        while (g_u32AdcIntFlag == 0);
        ADC_DISABLE_INT(ADC, ADC_ADF_INT);
        for (u32ChannelCount = 0; u32ChannelCount < 3; u32ChannelCount++)
        {
            i32ConversionData[u32ChannelCount] = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
					seed *=i32ConversionData[u32ChannelCount];
        }
        printf("{%d, %d, %d}\n", i32ConversionData[0], i32ConversionData[1], i32ConversionData[2]);
        CLK_SysTickDelay(500000);

		printf("ADC conversion done!\n");
		for(i = 0; i < 3; i++)
    {
				seed *= ADC_GET_CONVERSION_DATA(ADC, i);
    }
		seed *= 1000;
		printf("\nRandom seed: %d\n", seed);
    srand(seed);

    ReportEvery10 = 1;
    for( p = 0 ; p < train_data_num ; p++ )
    {
        RandomizedIndex[p] = p ;
    }

	scale_data();
    ret = train_preprocess();
    ret |= test_preprocess();
    if(ret) //Error Check
        return 1;

    return 0;
}


void run_train_data()
{
    int i, j, p;
    int correct=0;
    float accuracy = 0;
    printf("Train result:\n");
    for( p = 0 ; p < train_data_num ; p++ )
    {
        max = 0;
        for (i = 1; i < target_num; i++)
        {
            if (train_data_output[p][i] > train_data_output[p][max]) {
                max = i;
            }
        }
        target_value = max;

    /******************************************************************
    * Compute hidden layer activations
    ******************************************************************/

        for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
            Accum = HiddenWeights_1[input_length][i] ;
            for( j = 0 ; j < input_length ; j++ ) {
                Accum += train_data_input[p][j] * HiddenWeights_1[j][i] ;
            }
            Hidden_1[i] = 1.0/(1.0 + exp(-Accum)) ;
        }
        for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
            Accum = HiddenWeights_2[HiddenNodes_1][i] ;
            for( j = 0 ; j < HiddenNodes_1; j++ ) {
                Accum += Hidden_1[j] * HiddenWeights_2[j][i] ;
            }
            Hidden_2[i] = 1.0/(1.0 + exp(-Accum)) ;
        }

    /******************************************************************
    * Compute output layer activations and calculate errors
    ******************************************************************/

        for( i = 0 ; i < target_num ; i++ ) {
            Accum = OutputWeights[HiddenNodes_2][i] ;
            for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
                Accum += Hidden_2[j] * OutputWeights[j][i] ;
            }
            Output[i] = 1.0/(1.0 + exp(-Accum)) ;
        }

        max = 0;
        for (i = 1; i < target_num; i++)
        {
            if (Output[i] > Output[max]) {
                max = i;
            }
        }
        out_value = max;

        if(out_value!=target_value)
            printf("Error --> Training Pattern: %d,Target : %d, Output : %d\n", p, target_value, out_value);
        else
            correct++;
        }


        // Calculate accuracy
        accuracy = (float)correct / train_data_num;
        printf ("Accuracy1 = %.2f /100 \n",accuracy*100);

}
void run_test_data()
{
    int i, j, p;
    int correct=0;
    float accuracy = 0;
    printf("Test result:\n");
    for( p = 0 ; p < test_data_num ; p++ )
    {
        max = 0;
        for (i = 1; i < target_num; i++)
        {
            if (test_data_output[p][i] > test_data_output[p][max]) {
                max = i;
            }
        }
        target_value = max;

    /******************************************************************
    * Compute hidden layer activations
    ******************************************************************/

        for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
            Accum = HiddenWeights_1[input_length][i] ;
            for( j = 0 ; j < input_length ; j++ ) {
                Accum += test_data_input[p][j] * HiddenWeights_1[j][i] ;
            }
            Hidden_1[i] = 1.0/(1.0 + exp(-Accum)) ;
        }
        for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
            Accum = HiddenWeights_2[HiddenNodes_1][i] ;
            for( j = 0 ; j < HiddenNodes_1 ; j++ ) {
                Accum += Hidden_1[j] * HiddenWeights_2[j][i] ;
            }
            Hidden_2[i] = 1.0/(1.0 + exp(-Accum)) ;
        }

    /******************************************************************
    * Compute output layer activations and calculate errors
    ******************************************************************/

        for( i = 0 ; i < target_num ; i++ ) {
            Accum = OutputWeights[HiddenNodes_2][i] ;
            for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
                Accum += Hidden_2[j] * OutputWeights[j][i] ;
            }
            Output[i] = 1.0/(1.0 + exp(-Accum)) ;
        }
        max = 0;
        for (i = 1; i < target_num; i++)
        {
            if (Output[i] > Output[max]) {
                max = i;
            }
        }
        out_value = max;

        if(out_value!=target_value)
            printf("Error --> Training Pattern: %d,Target : %d, Output : %d\n", p, target_value, out_value);
        else
            correct++;
        }
        // Calculate accuracy
        accuracy = (float)correct / test_data_num;

        printf ("Accuracy2 = %.2f /100 \n",accuracy*100);
}

float Get_Train_Accuracy()
{
    int i, j, p;
    int correct = 0;
		float accuracy = 0;
    for (p = 0; p < train_data_num; p++)
    {
/******************************************************************
* Compute hidden layer activations
******************************************************************/

        for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
            Accum = HiddenWeights_1[input_length][i] ;
            for( j = 0 ; j < input_length ; j++ ) {
                Accum += train_data_input[p][j] * HiddenWeights_1[j][i] ;
            }
            Hidden_1[i] = 1.0/(1.0 + exp(-Accum)) ;
        }
        for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
            Accum = HiddenWeights_2[HiddenNodes_1][i] ;
            for( j = 0 ; j < HiddenNodes_1 ; j++ ) {
                Accum += Hidden_1[j] * HiddenWeights_2[j][i] ;
            }
            Hidden_2[i] = 1.0/(1.0 + exp(-Accum)) ;
        }
/******************************************************************
* Compute output layer activations
******************************************************************/

        for( i = 0 ; i < target_num ; i++ ) {
            Accum = OutputWeights[HiddenNodes_2][i] ;
            for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
                Accum += Hidden_2[j] * OutputWeights[j][i] ;
            }
            Output[i] = 1.0/(1.0 + exp(-Accum)) ;
        }


        //get target value
        max = 0;
        for (i = 1; i < target_num; i++)
        {
            if (train_data_output[p][i] > train_data_output[p][max]) {
                max = i;
            }
        }
        target_value = max;
        //get output value
        max = 0;
        for (i = 1; i < target_num; i++)
        {
            if (Output[i] > Output[max]) {
                max = i;
            }
        }
        out_value = max;
        //compare output and target
        if (out_value==target_value)
        {
            correct++;
        }
    }

    // Calculate accuracy
    accuracy = (float)correct / train_data_num;
		//printf("correct %d\n",correct);
    return accuracy;
}

void load_weight()
{
    int i, j;

    // Input → Hidden-1
    printf("\n======= HiddenWeights_1 (input→Hidden1) =======\n");
    printf("{\n");
    for (i = 0; i <= input_length; i++) {
        printf("  {");
        for (j = 0; j < HiddenNodes_1; j++) {
            printf("%f%s", HiddenWeights_1[i][j],
                   (j < HiddenNodes_1 - 1) ? ", " : "");
        }
        printf("}%s\n", (i < input_length) ? "," : "");
    }
    printf("}\n");

    // Hidden-1 → Hidden-2
    printf("\n======= HiddenWeights_2 (Hidden1→Hidden2) =======\n");
    printf("{\n");
    for (i = 0; i <= HiddenNodes_1; i++) {
        printf("  {");
        for (j = 0; j < HiddenNodes_2; j++) {
            printf("%f%s", HiddenWeights_2[i][j],
                   (j < HiddenNodes_2 - 1) ? ", " : "");
        }
        printf("}%s\n", (i < HiddenNodes_1) ? "," : "");
    }
    printf("}\n");

    // Hidden-2 → Output
    printf("\n======= OutputWeights (Hidden2→output) =======\n");
    printf("{\n");
    for (i = 0; i <= HiddenNodes_2; i++) {
        printf("  {");
        for (j = 0; j < target_num; j++) {
            printf("%f%s", OutputWeights[i][j],
                   (j < target_num - 1) ? ", " : "");
        }
        printf("}%s\n", (i < HiddenNodes_2) ? "," : "");
    }
    printf("}\n");
}

void AdcSingleCycleScanModeTest()
{
		int i, j;
    uint32_t u32ChannelCount;
    float single_data_input[6];
		char output_string[10] = {NULL};
    float fx0, fy0, fz0;    
    float fx1, fy1, fz1;
    float fx0_old,fy0_old,fz0_old;
    float fx1_old,fy1_old,fz1_old;
    float fx1_diff,fy1_diff,fz1_diff;
    float fx0_diff,fy0_diff,fz0_diff;
    uint32_t last0 = 0;
    printf("\n");
		printf("[Phase 3] Start Prediction ...\n\n");
		PB2=1;
    while(1)
    {
		printf(">>>>>>>>>>>>>><<<<<<<<<<<<<<<<<\n\n");

				/* Set the ADC operation mode as single-cycle, input mode as single-end and
                 enable the analog input channel 0, 1, 2 and 3 */
        // ADC_Open(ADC, ADC_ADCR_DIFFEN_SINGLE_END, ADC_ADCR_ADMD_SINGLE_CYCLE, 0x7);

        // /* Power on ADC module */
        // ADC_POWER_ON(ADC);

        // /* Clear the A/D interrupt flag for safe */
        // ADC_CLR_INT_FLAG(ADC, ADC_ADF_INT);

        // /* Start A/D conversion */
        // ADC_START_CONV(ADC);

        // /* Wait conversion done */
        // while(!ADC_GET_INT_FLAG(ADC, ADC_ADF_INT));
        printf(">>>>>>>>>>>>>><<<<<<<<<<<<<<<<<\n\n");

        // for(u32ChannelCount = 0; u32ChannelCount < 6; u32ChannelCount++)
        // {
        //     single_data_input[u32ChannelCount] = ADC_GET_CONVERSION_DATA(ADC, u32ChannelCount);
        // }
                ACC_VALUE_0();
                ACC_VALUE_1();
                fx0 = (float)x_end0/ 256.0f;
                fy0 = (float)y_end0/ 256.0f;
                fz0 = (float)z_end0/ 256.0f;
                fx1 = (float)x_end1/ 256.0f;
                fy1 = (float)y_end1/ 256.0f;
                fz1 = (float)z_end1/ 256.0f;
                fx1_diff = fx1 - fx1_old;
                fy1_diff = fy1 - fy1_old;
                fz1_diff = fz1 - fz1_old;
                fx0_diff = fx0 - fx0_old;
                fy0_diff = fy0 - fy0_old;
                fz0_diff = fz0 - fz0_old;

                //printf("x = %d g, y = %d g, z = %d g\n",x,y,z);
                // printf("X0 = %.2f g, Y0 = %.2f g, Z0 = %.2f g X1 = %.2f g, Y1 = %.2f g, Z1 = %.2f g\n", fx0, fy0, fz0, fx1, fy1, fz1);
                // printf("dX0 = %.2f g, dY0 = %.2f g, dZ0 = %.2f g dX1 = %.2f g, dY1 = %.2f g, dZ1 = %.2f g\n", fx0_diff, fy0_diff, fz0_diff, fx1_diff, fy1_diff, fz1_diff);
                fx1_old = fx1;
                fy1_old = fy1;
                fz1_old = fz1;
                fx0_old = fx0;
                fy0_old = fy0;
                fz0_old = fz0;
                
                // if (g_timer0Count != last0)
                // {

                //     last0 = g_timer0Count;
                //     // last1 = g_timer1Count;
                // }
            single_data_input[0] = fx0;
            single_data_input[1] = fy0;
            single_data_input[2] = fz0;
            single_data_input[3] = fx1;
            single_data_input[4] = fy1;
            single_data_input[5] = fz1;

            normalize(single_data_input);

            printf("output: %f %f %f %f %f %f\n",single_data_input[0],single_data_input[1],single_data_input[2],single_data_input[3],single_data_input[4],single_data_input[5]);
            printf("std:%lf,mean%lf\n",data_mean[0],data_std[0]);
            // Compute hidden layer activations
            for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
                    Accum = HiddenWeights_1[input_length][i] ;
                    for( j = 0 ; j < input_length ; j++ ) {
                            Accum += single_data_input[j] * HiddenWeights_1[j][i] ;
                    }
                    Hidden_1[i] = 1.0/(1.0 + exp(-Accum)) ;
            }
            for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
                Accum = HiddenWeights_2[HiddenNodes_1][i];          // bias term
                for( j = 0 ; j < HiddenNodes_1 ; j++ ) {
                    Accum += Hidden_1[j] * HiddenWeights_2[j][i];   // feed in Hidden_1, not single_data_input
                }
                Hidden_2[i] = 1.0/(1.0 + exp(-Accum));
            }

            // Compute output layer activations
            for( i = 0 ; i < target_num ; i++ ) {
                    Accum = OutputWeights[HiddenNodes_2][i] ;
                    for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
                            Accum += Hidden_2[j] * OutputWeights[j][i] ;
                    }
                    Output[i] = 1.0/(1.0 + exp(-Accum)) ;
            }

            max = 0;

            for (i = 1; i < target_num; i++)
            {
                    if (Output[i] > Output[max]) {
                            max = i;
                            printf("efhwiefhbued\n");
                    }
                printf("max: %doutput: %d  :%f\n",max ,i, Output[i]);
            }
            out_value = max;
            printf("max====== %d",max);

            switch(out_value){
                    case 0:
                            strcpy(output_string, "jump");
                            break;
                    case 1:
                            strcpy(output_string, "open_close_jump");
                            break;
                    case 2:
                            strcpy(output_string, "run");
                            break;
                    case 3:
                            strcpy(output_string, "walk");
                            break;
                    default:
                            strcpy(output_string, "idle");
                            break;
            
            }
            printf(" %.2f, %.2f, %.2f, %.2f, %.2f, %.2f\n", fx0, fy0, fz0, fx1, fy1, fz1);
            printf("\rPrediction output: %-8s", output_string);
            CLK_SysTickDelay(500000);
    }
}



int32_t main(void)
{
		int i, j, p, q, r;
    float accuracy=0;
	  uint32_t u32ChannelCount;
    int32_t i32ConversionData[3];
   SYS_Init();

    /* Init UART0 to 115200-8n1 for print message */
    UART_Open(UART0, 115200);
    uint32_t last0 = 0;


    /* Init I2C0 */
    I2C0_Init();
    I2C1_Init();
    ADXL345_WriteRegister_0(0x1E,0);
    ADXL345_WriteRegister_0(0x1F,-1);
    ADXL345_WriteRegister_0(0x20,2);

    ADXL345_WriteRegister_1(0x1E,0);
    ADXL345_WriteRegister_1(0x1F,-1);
    ADXL345_WriteRegister_1(0x20,2);

    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 20);
    TIMER_EnableInt(TIMER0);
    NVIC_EnableIRQ(TMR0_IRQn);
    
    float fx0, fy0, fz0;
    float fx1, fy1, fz1;
    float fx0_old,fy0_old,fz0_old;
    float fx1_old,fy1_old,fz1_old;
    float fx1_diff,fy1_diff,fz1_diff;
    float fx0_diff,fy0_diff,fz0_diff;
    TIMER_Start(TIMER0);
	  printf("\n+-----------------------------------------------------------------------+\n");
    printf("|                        00LAB8 - Machine Learning                        |\n");
    printf("+-----------------------------------------------------------------------+\n");

	//     printf("\n[Phase 1] Initialize DataSet ...");
	//   /* Data Init (Input / Output Preprocess) */
	// 	if(data_setup()){
    //     printf("[Error] Datasets Setup Error\n");
    //     return 0;
    // }else
	// 			printf("Done!\n\n");

	// 	printf("[Phase 2] Start Model Training ...\n");
	// 	// Initialize HiddenWeights and ChangeHiddenWeights
    // for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
    //     for( j = 0 ; j <= input_length ; j++ ) {
    //         ChangeHiddenWeights_1[j][i] = 0.0 ;
    //         Rando = (float)((rand() % 100))/100;
    //         HiddenWeights_1[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    //     }
    // }
    // for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
    //     for( j = 0 ; j <= HiddenNodes_1 ; j++ ) {
    //         ChangeHiddenWeights_2[j][i] = 0.0 ;
    //         Rando = (float)((rand() % 100))/100;
    //         HiddenWeights_2[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    //     }
    // }

    // // Initialize OutputWeights and ChangeOutputWeights
    // for( i = 0 ; i < target_num ; i ++ ) {
    //     for( j = 0 ; j <= HiddenNodes_2 ; j++ ) {
    //         ChangeOutputWeights[j][i] = 0.0 ;
    //         Rando = (float)((rand() % 100))/100;
    //         OutputWeights[j][i] = 2.0 * ( Rando - 0.5 ) * InitialWeightMax ;
    //     }
    // }

    // // Begin training
    // for(TrainingCycle = 1 ; TrainingCycle < 2147483647 ; TrainingCycle++)
    // {
    //     Error = 0.0 ;

    //     // Randomize order of training patterns
    //     for( p = 0 ; p < train_data_num ; p++) {
    //         q = rand()%train_data_num;
    //         r = RandomizedIndex[p] ;
    //         RandomizedIndex[p] = RandomizedIndex[q] ;
    //         RandomizedIndex[q] = r ;
    //     }

    //     // Cycle through each training pattern in the randomized order
    //     for( q = 0 ; q < train_data_num ; q++ )
    //     {
    //         p = RandomizedIndex[q];

    //         /* --------------------------------------------------------- */
    //         /*  Forward pass                                            */
    //         /* --------------------------------------------------------- */

    //         /* Compute Hidden-1 activations */
    //         for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
    //             Accum = HiddenWeights_1[input_length][i] ;
    //             for( j = 0 ; j < input_length ; j++ ) {
    //                 Accum += train_data_input[p][j] * HiddenWeights_1[j][i] ;
    //             }
    //             Hidden_1[i] = 1.0/(1.0 + exp(-Accum)) ;
    //         }

    //         /* Compute Hidden-2 activations */
    //         for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
    //             Accum = HiddenWeights_2[HiddenNodes_1][i] ;
    //             for( j = 0 ; j < HiddenNodes_1 ; j++ ) {
    //                 Accum += Hidden_1[j] * HiddenWeights_2[j][i] ;
    //             }
    //             Hidden_2[i] = 1.0/(1.0 + exp(-Accum)) ;
    //         }

    //         /* Compute Output layer activations and deltas */
    //         for( i = 0 ; i < target_num ; i++ ) {
    //             Accum = OutputWeights[HiddenNodes_2][i] ;
    //             for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
    //                 Accum += Hidden_2[j] * OutputWeights[j][i] ;
    //             }
    //             Output[i] = 1.0/(1.0 + exp(-Accum)) ;
    //             OutputDelta[i] = (train_data_output[p][i] - Output[i]) * Output[i] * (1.0 - Output[i]) ;
    //             Error += 0.5 * (train_data_output[p][i] - Output[i]) * (train_data_output[p][i] - Output[i]) ;
    //         }

    //         /* --------------------------------------------------------- */
    //         /*  Back-propagation                                        */
    //         /* --------------------------------------------------------- */

    //         /* Hidden-2 deltas */
    //         for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
    //             Accum = 0.0 ;
    //             for( j = 0 ; j < target_num ; j++ ) {
    //                 Accum += OutputWeights[i][j] * OutputDelta[j] ;
    //             }
    //             HiddenDelta_2[i] = Accum * Hidden_2[i] * (1.0 - Hidden_2[i]) ;
    //         }

    //         /* Hidden-1 deltas */
    //         for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
    //             Accum = 0.0 ;
    //             for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
    //                 Accum += HiddenWeights_2[i][j] * HiddenDelta_2[j] ;
    //             }
    //             HiddenDelta_1[i] = Accum * Hidden_1[i] * (1.0 - Hidden_1[i]) ;
    //         }

    //         /* --------------------------------------------------------- */
    //         /*  Weight updates (Momentum + SGD)                         */
    //         /* --------------------------------------------------------- */

    //         /* Input  -> Hidden-1 */
    //         for( i = 0 ; i < HiddenNodes_1 ; i++ ) {
    //             ChangeHiddenWeights_1[input_length][i] = LearningRate * HiddenDelta_1[i] + Momentum * ChangeHiddenWeights_1[input_length][i] ;
    //             HiddenWeights_1[input_length][i] += ChangeHiddenWeights_1[input_length][i] ;
    //             for( j = 0 ; j < input_length ; j++ ) {
    //                 ChangeHiddenWeights_1[j][i] = LearningRate * train_data_input[p][j] * HiddenDelta_1[i] + Momentum * ChangeHiddenWeights_1[j][i] ;
    //                 HiddenWeights_1[j][i] += ChangeHiddenWeights_1[j][i] ;
    //             }
    //         }

    //         /* Hidden-1 -> Hidden-2 */
    //         for( i = 0 ; i < HiddenNodes_2 ; i++ ) {
    //             ChangeHiddenWeights_2[HiddenNodes_1][i] = LearningRate * HiddenDelta_2[i] + Momentum * ChangeHiddenWeights_2[HiddenNodes_1][i] ;
    //             HiddenWeights_2[HiddenNodes_1][i] += ChangeHiddenWeights_2[HiddenNodes_1][i] ;
    //             for( j = 0 ; j < HiddenNodes_1 ; j++ ) {
    //                 ChangeHiddenWeights_2[j][i] = LearningRate * Hidden_1[j] * HiddenDelta_2[i] + Momentum * ChangeHiddenWeights_2[j][i] ;
    //                 HiddenWeights_2[j][i] += ChangeHiddenWeights_2[j][i] ;
    //             }
    //         }

    //         /* Hidden-2 -> Output */
    //         for( i = 0 ; i < target_num ; i++ ) {
    //             ChangeOutputWeights[HiddenNodes_2][i] = LearningRate * OutputDelta[i] + Momentum * ChangeOutputWeights[HiddenNodes_2][i] ;
    //             OutputWeights[HiddenNodes_2][i] += ChangeOutputWeights[HiddenNodes_2][i] ;
    //             for( j = 0 ; j < HiddenNodes_2 ; j++ ) {
    //                 ChangeOutputWeights[j][i] = LearningRate * Hidden_2[j] * OutputDelta[i] + Momentum * ChangeOutputWeights[j][i] ;
    //                 OutputWeights[j][i] += ChangeOutputWeights[j][i] ;
    //             }
    //         }
    //     }
    //     accuracy = Get_Train_Accuracy();

    //     // Every 10 cycles send data to terminal for display
    //     ReportEvery10 = ReportEvery10 - 1;
    //     if (ReportEvery10 == 0)
    //     {

    //         printf ("\nTrainingCycle: %ld\n",TrainingCycle);
    //         printf ("Error = %.5f\n",Error);
    //         printf ("Accuracy3 = %.2f /100 \n",accuracy*100);
    //         load_weight();
    //         //run_train_data();

    //         if (TrainingCycle==1)
    //         {
    //             ReportEvery10 = 9;
    //         }
    //         else
    //         {
    //             ReportEvery10 = 10;
    //         }
    //     }

    //     // If error rate is less than pre-determined threshold then end
    //     if( accuracy >= goal_acc ) break ;
    // }

    printf ("\nTrainingCycle: %ld\n",TrainingCycle);
    printf ("Error = %.5f\n",Error);
    //run_train_data();
    printf ("Training Set Solved!\n");
    printf ("--------\n");
    printf ("Testing Start!\n ");
    //run_test_data();
    printf ("--------\n");
    ReportEvery10 = 1;
    load_weight();

		printf("\nModel Training Phase has ended.\n");
    //scale_data();
    /* Start prediction */
    AdcSingleCycleScanModeTest();

    while(1);
	
}




/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/

