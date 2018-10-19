//-----------------------------------------------------------------------------
// Drv_Spi.c
//
//  Created on	: 2018-5-08
//      Author	: Divenire
//		version	: V1.0
//		brief	:
//-----------------------------------------------------------------------------
// Attention:
//
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "BSP\Drv_include.h"

/*****************************************************************************
* Function Name   : Drv_SPI2_Init
* Input           : None
* Output          : None
* Return          : None
* Description     :
*						SSI2Clk		SPI2_CLK	PD3
*						SS2_Fss		SPI2_CS		Soft
*						SSI2XDAT1	SPI2_MISO	PD0
*						SSI2XDAT0	SPI2_MOSI	PD1
*****************************************************************************/
void Drv_SPI2_Init(void)
{
    uint32_t ui32Data;
    //
    // The SSI2 peripheral must be enabled for use.
    //

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    //
    // The GPIOD peripheral must be enabled for use.
    //

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for SSI2 functions on port PD0 PD1 PD3
    //

    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    GPIOPinConfigure(GPIO_PD3_SSI2CLK);

    //
    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.
    //

    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_0);

    //
    // Configure and enable the SSI2 port for SPI2 master mode.
    // Moto fmt, CPOL 1, CPHA 1
    // SPI_CLK  10MHZ	8bit (21MHZ???)
    SSIConfigSetExpClk(SSI2_BASE, Tiva_SysClock, SSI_FRF_MOTO_MODE_3,
                       SSI_MODE_MASTER, 1000000, 8);

    //
    // Enable the SSI2 module.
    //

    SSIEnable(SSI2_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while (SSIDataGetNonBlocking(SSI2_BASE, &ui32Data))
        ;
}
void Drv_SPI2_Transmit(uint8_t *pData, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        Drv_SPI2_RW(pData[i]);
    }
}

void Drv_SPI2_Receive(uint8_t *pData, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++)
    {
        pData[i] = Drv_SPI2_RW(0);
    }
}

uint8_t Drv_SPI2_RW(uint8_t value)
{
    uint32_t ui32Data;
    uint8_t ui8Data;
    SSIDataPut(SSI2_BASE, value);
    while (SSIBusy(SSI2_BASE))
        ;
    SSIDataGet(SSI2_BASE, &ui32Data);
    ui8Data = ui32Data & 0xff;
    return (ui8Data);
}
