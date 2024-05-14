/*
 * 007spi_txonly_arduino.c
 *
 *  Created on: Jan 23, 2024
 *      Author: khater
 */

/*
 * 004spi_tx_testing.c
 *
 *  Created on: Jan 1, 2024
 *      Author: khater
 */

#include<string.h>
#include "stm32f10xx.h"

/*
 * 				SPI1 PINS				*
 *PA7 -- SPI1_MOSI
 *PA6 -- SPI1_MISO
 *PA5 -- SPI1_SCK
 *PA4 -- SPI1_NSS
 ************************************************/
void SPI_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OP2;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_AF_PP;
	//SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_OP_AF_PP;
	GPIO_PeriClockControl(GPIOB, ENABLE);

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);


	AFIO_PCLK_EN();
}
void delay(void)
{
	for(uint32_t i = 0; i< 500000 /2; i++);

}


void SPI_Inits(void)
{
	SPI_Handle_t SPIHandle;
	SPIHandle.pSPIx = SPI2;
	SPIHandle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPIHandle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPIHandle.SPIConfig.SPI_SclkSpeed = SPI_CLK_SPEED_DIV16;
	SPIHandle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPIHandle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPIHandle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPIHandle.SPIConfig.SPI_SSM = SPI_SSM_DI;
	AFIO_PCLK_EN();
	SPIHandle.pAFIO = AFIO;
	SPIHandle.pAFIO->MAPR |=0;

	SPI_Init(&SPIHandle);


}
void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn;
	GPIOBtn.pGPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_IN_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = GPIO_IN_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GPIOBtn);
}
int main(void)
{
	char user_data[]= "Khater";
	// this func is to make the GPIO pin behave as SPI
	SPI_GPIOInits();
	// this func is to intialize the SPI peripheral params
	SPI_Inits();

	GPIO_ButtonInit();

	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while( GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_0) );

		delay();
		// enable the SPI peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//first lets send length of the data info
		uint8_t datalen = strlen(user_data);
		SPI_SendData(SPI2, &datalen, 1);

		// to send the data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		while( SPI_GetFlagStatus(SPI2, SPI_FLAG_BSY) );

		SPI_PeripheralControl(SPI2, DISABLE);


	}

	return 0;
}


