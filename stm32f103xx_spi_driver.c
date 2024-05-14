/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jan 1, 2024
 *      Author: khater
 */


#include "stm32f103xx_spi_driver.h"


/*****************************************************************************
 * @fn				-SPI_PeriClockControl
 *
 * @brief			- this fn enables or disables the peripheral clk of a given SPI
 *
 * @param[]			- BASE ADDR of the SPI peripheral
 * @param[]			- enable or disable macros
 *
 * @note			- none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();

		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}

}

/*****************************************************************************
 * @fn					- SPI_Init
 *
 * @brief				- intializes the SPI
 *
 * @param[1]			- pointer to the struct of the spi handle for the mode and cfg
 *
 * @note				- none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// first lets configure the SPI_CR1 register

	uint32_t tempreg =0;

	// enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// 1. configure the device mode -> if the device is master (1) or slave (0) then it would be shifted by 2
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// 2. configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// bidi mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// bidi mode should be enabled
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		// RXONLY should be set
		tempreg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clk speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DDF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;


	// 6. configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// 7. save the tempreg in CR1
	pSPIHandle->pSPIx->CR1 = tempreg;

	/*if(pSPIHandle->pSPIx == SPI1)
	{
		pSPIHandle->pAFIO->MAPR &= ~(1<<0);
	}
	else if(pSPIHandle->pSPIx == SPI3)
	{

	}
	*/
}

/*****************************************************************************
 * @fn					- SPI_DeInit
 *
 * @brief				- Reset the SPI
 *
 * @param[1]			-ptr to SPI register of a particular SPI
 *
 * @note				- none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET; // Flag reset ezay ???? flag set el mafrood? garabha ma3 abdulla
	}
	return FLAG_RESET;
}
/*****************************************************************************
 * @fn					-SPI_SendData
 *
 * @brief				- configures the master spi to send data to the slave
 *
 * @param[1]			- ptr to SPI register of a particular SPI
 * @param[2]			- msg to be transmitted in the Tx buffer
 * @param[3]			- msg length
 *
 * @note				- This is a blocking call
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		// 1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ) )
		{
			// 16 bit DFF
			// 1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;

		}
		else
		{
			// 8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*****************************************************************************
 * @fn					-SPI_ReceiveData
 *
 * @brief				- configures the master spi to send data to the slave
 *
 * @param[1]			- ptr to SPI register of a particular SPI
 * @param[2]			- msg to be Received in the Rx buffer
 * @param[3]			- msg length
 *
 * @note				- none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}


/*****************************************************************************
 * @fn					-SPI_IRQConfig
 *
 * @brief				- configures the interrupt traits of a given SPI
 *
 * @param[1]			- interrupt number
 * @param[2]			- interrupt priority
 * @param[3]			- Enable or disable macros
 *
 * @note				- none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI)
{

}
/*****************************************************************************
 * @fn					-SPI_IRQPriorityConfig
 *
 * @brief				- configures the interrupt priority of a given SPI
 *
 * @param[1]			- interrupt number
 * @param[2]			- interrupt priority
 * @param[3]			- Enable or disable macros
 *
 * @note				- none
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQpriority)
{

}
/*****************************************************************************
 * @fn				-SPI_IRQHandling
 *
 * @brief			- this function takes the pin that would work as an interrupt
 *
 * @param[1]		- pointer to the SPI handle struct
 *
 * @note			- none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

/*****************************************************************************
 * @fn				-SPI_PeripheralControl
 *
 * @brief			- this function enables or disables the SPI peripheral
 *
 * @param[1]		- ptr to SPI register of a particular SPI
 *
 * @note			- none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*****************************************************************************
 * @fn				-SPI_SSIConfig
 *
 * @brief			-
 *
 * @param[1]		-
 *
 * @note			-
 */

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SSI);
	}
}

/*****************************************************************************
 * @fn				-SPI_SSOEConfig
 *
 * @brief			-
 *
 * @param[1]		-
 *
 * @note			- none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR2 |= (1<< SPI_CR2_SSOE);
	}
	else
	{
		pSPIx->CR2 &= ~(1<< SPI_CR2_SSOE);
	}
}

