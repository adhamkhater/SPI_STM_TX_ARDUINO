/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: Jan 1, 2024
 *      Author: khater
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_
#include "stm32f10xx.h"

/**************************** PERIPHERAL REGISTER Configuration STRUCTURE ********************************/

/*
 * PERIPHERAL REGISTER Configuration STRUCTURE for SPI
 */
typedef struct
{
	uint8_t SPI_DeviceMode;				//@SPI_DeviceMode Modes
	uint8_t SPI_BusConfig;				//@SPI_BusConfig Modes
	uint8_t SPI_SclkSpeed;				//@SPI_SclkSpeed Modes
	uint8_t SPI_DFF;					//@SPI_DFF Modes
	uint8_t SPI_CPOL;					//@SPI_CPOL Modes
	uint8_t SPI_CPHA;					//@SPI_CPHA Modes
	uint8_t SPI_SSM;					//@SPI_SSM Modes

}SPI_Config_t;

/*
 * Handling structure for the SPI PERIPHERAL
 */

typedef struct
{
	SPI_RegDef_t		*pSPIx;
	SPI_Config_t		SPIConfig;
	AFIO_RegDef_t		*pAFIO;

}SPI_Handle_t;

/********************************************
 *		 @SPI_DeviceMode Modes
 ********************************************/
#define SPI_DEVICE_MODE_MASTER				1
#define SPI_DEVICE_MODE_SLAVE				0

/********************************************
 *		 @SPI_BusConfig Modes
 ********************************************/
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/********************************************
 *		 @SPI_SclkSpeed Modes
 ********************************************/
#define SPI_CLK_SPEED_DIV2					0
#define SPI_CLK_SPEED_DIV4					1
#define SPI_CLK_SPEED_DIV8					2
#define SPI_CLK_SPEED_DIV16					3
#define SPI_CLK_SPEED_DIV32					4
#define SPI_CLK_SPEED_DIV64					5
#define SPI_CLK_SPEED_DIV128				6
#define SPI_CLK_SPEED_DIV256				7

/********************************************
 *		 @SPI_DFF Modes
 ********************************************/
#define SPI_DFF_8BITS						0
#define SPI_DFF_16BITS						1

/********************************************
 *		 @SPI_CPOL Modes
 ********************************************/
#define SPI_CPOL_HIGH						1
#define SPI_CPOL_LOW						0
/********************************************
 *		 @SPI_CPHA Modes
 ********************************************/
#define SPI_CPHA_HIGH						1
#define SPI_CPHA_LOW						0

/********************************************
 *		 @SPI_SSM Modes
 ********************************************/
#define SPI_SSM_EN							1
#define SPI_SSM_DI							0

/********************************************
 *		 SPI related status flags definitions
 ********************************************/
#define SPI_FLAG_TXE						( 1 << SPI_SR_TXE)
#define SPI_FLAG_RXNE						( 1 << SPI_SR_RXNE)
#define SPI_FLAG_BSY						( 1 << SPI_SR_BSY)
/**************************************************************************************
 *
 * 								APIs Supported by this driver
 *
 **************************************************************************************/
/*
 * Peripheral Clk setup
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorD);

/*
 * Init and De-init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * Data send and receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQpriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * Other peripheral controls and APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*******************************************************************************************************/
#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
