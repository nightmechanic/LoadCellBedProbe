#include "main.h"
#include "ads1256.h"


extern TIM_HandleTypeDef htim3;

//HAL funtions for ADC

__STATIC_INLINE uint8_t ads1256_HAL_is_DRDY_high(void) {
	return (uint8_t)LL_GPIO_IsInputPinSet(ADC_DRDY_GPIO_Port, ADC_DRDY_Pin);
}



void ads1256_HAL_reset(void){
	LL_GPIO_ResetOutputPin(ADC_Reset_GPIO_Port, ADC_Reset_Pin);

	delay_us(ADS_RESET_LENGTH_US);

	LL_GPIO_SetOutputPin(ADC_Reset_GPIO_Port, ADC_Reset_Pin);

}

void ads1256_HAL_Setup_Buffers(uint8_t *TXBuff, uint8_t *RXBuff){
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_3, LL_SPI_DMA_GetRegAddr(SPI2),
				  	  	  	  	  (uint32_t)RXBuff, LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_3)); //RX
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_STREAM_4, (uint32_t)TXBuff,
				  	  	  	  	  LL_SPI_DMA_GetRegAddr(SPI2), LL_DMA_GetDataTransferDirection(DMA1, LL_DMA_STREAM_4));  //TX
}

ADS1256C_StatusTypeDef ads1256_HAL_SPI_Transfer(uint8_t bytecount){

	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_3, bytecount);
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_4, bytecount);


	//set CS low
	LL_GPIO_ResetOutputPin(ADC_CS_GPIO_Port, ADC_CS_Pin);

	// Enable SPI
	LL_SPI_Enable(SPI2);

	/* Enable DMA Channels */
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_3); //RX
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_4); //TX


	//wait for (RX) transfer completion
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while (!LL_DMA_IsActiveFlag_TC3(DMA1)) {
		if ( (__HAL_TIM_GET_COUNTER(&htim3)) > (bytecount * SPI_TIMEOUT_US) ) {
					return ADS1256_SPI_TIMEOUT;
				}
	}


	// Set CS High
	LL_GPIO_SetOutputPin(ADC_CS_GPIO_Port, ADC_CS_Pin);

	// clear DMA flags
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_ClearFlag_TC4(DMA1);

	//Disabe SPI
	LL_SPI_Disable(SPI2);
	//Disable DMA channels
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4); //TX
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3); //RX


	return ADS1256_OK;
}




uint8_t RX_Buffer[8];
uint8_t TX_Buffer[8];

ADS1256C_StatusTypeDef ads1256_init(void){

	ADS1256C_StatusTypeDef status;

	//setup buffers
	ads1256_HAL_Setup_Buffers(TX_Buffer, RX_Buffer);

	//ads reset
	ads1256_HAL_reset();

	//wait for DRDY to go low after reset calibration

	__HAL_TIM_SET_COUNTER(&htim3,0);
	while (ads1256_HAL_is_DRDY_high()){
		if ( (__HAL_TIM_GET_COUNTER(&htim3)) > DRDY_CAL_TIMEOUT_US ) {
			return ADS1256_DRDY_TIMEOUT;
		}

	}

	// read DRATE register to verify we are communicating

	TX_Buffer[0] = CMD_RREG | REG_STATUS;
	TX_Buffer[1] = 0x00;

	status = ads1256_HAL_SPI_Transfer(2);
	if ( status != ADS1256_OK ) {
		return status;
	}

	delay_us(CMD_TO_DATA_US);
	TX_Buffer[0] = 0x00;

	status = ads1256_HAL_SPI_Transfer(1);
	if ( status != ADS1256_OK ) {
			return status;
		}

	if (RX_Buffer[0] != ADS1256_30000_SPS) {
		return ADS1256_MISMATCH;
	}
	return ADS1256_OK;

}

ADS1256C_StatusTypeDef ads1256_configure(ADS1256_DRATE ads_rate, ADS1256_GAIN ads_gain,
										ADS1256_CHANNELS ads_pchannel, ADS1256_CHANNELS ads_nchannel,
										uint8_t buf_en)
{
	ADS1256C_StatusTypeDef status;

	//setup buffers
	ads1256_HAL_Setup_Buffers(TX_Buffer, RX_Buffer);

	//set configuration registers
	TX_Buffer[0] = CMD_WREG | REG_STATUS;
	TX_Buffer[1] = 0x03; //writing 4 registers
	TX_Buffer[2] = 0x00 | (buf_en<<1);  // buf_en into status reg
	TX_Buffer[3] = (ads_pchannel<<4) | ads_nchannel; //active channels
	TX_Buffer[4] = 0x20 | ads_gain;
	TX_Buffer[5] = ads_rate;

	status = ads1256_HAL_SPI_Transfer(6);
	if ( status != ADS1256_OK ) {
		return status;
	}

	//perform self calibration
	TX_Buffer[0] = CMD_SELFCAL;

	status = ads1256_HAL_SPI_Transfer(1);
	if ( status != ADS1256_OK ) {
			return status;
	}

	//wait for DRDY to go low after calibration

	__HAL_TIM_SET_COUNTER(&htim3,0);
	while (ads1256_HAL_is_DRDY_high()){
		if ( (__HAL_TIM_GET_COUNTER(&htim3)) > DRDY_CAL_TIMEOUT_US ) {
			return ADS1256_DRDY_TIMEOUT;
		}
	}

	return ADS1256_OK;
}

ADS1256C_StatusTypeDef ads1256_get_samples(float32_t * sample_buffer, uint16_t sample_count, float32_t conversion_factor) {

	ADS1256C_StatusTypeDef status;
	int32_t NewDataInt;

	//setup buffers
	ads1256_HAL_Setup_Buffers(TX_Buffer, RX_Buffer);
	TX_Buffer[0] = 0;
	TX_Buffer[1] = 0;
	TX_Buffer[2] = 0;


	while (sample_count > 0) {
		TX_Buffer[0] = CMD_RDATA;
		//wait for DRDY to go low
		__disable_irq();
		__HAL_TIM_SET_COUNTER(&htim3,0);
		while ( ads1256_HAL_is_DRDY_high() ){
			if ( (__HAL_TIM_GET_COUNTER(&htim3)) > DRDY_SHORT_TIMEOUT_US ) {
				return ADS1256_DRDY_TIMEOUT;
			}
		}

		status = ads1256_HAL_SPI_Transfer(1);
		if ( status != ADS1256_OK ) {
			__enable_irq();
			return status;
		}

		delay_us(CMD_TO_DATA_US);

		TX_Buffer[0] = 0x00;

		status = ads1256_HAL_SPI_Transfer(3);
		if ( status != ADS1256_OK ) {
				__enable_irq();
				return status;
			}
		__enable_irq();
		//we have data :)
		NewDataInt = (RX_Buffer[0]<<16) + (RX_Buffer[1]<<8) + RX_Buffer[2];
		if ( NewDataInt > ((1 << 23) - 1) ) {
				NewDataInt -= ( 1 << 24 );
			}
		*sample_buffer = NewDataInt * conversion_factor;
		sample_buffer++;
	}
	return ADS1256_OK;
}

ADS1256C_StatusTypeDef ads1256_send_command(ADS1256_COMMANDS command){

	ADS1256C_StatusTypeDef status;

	//setup buffers
	ads1256_HAL_Setup_Buffers(TX_Buffer, RX_Buffer);

	TX_Buffer[0] = command;

	//wait for DRDY to go low
	__disable_irq();
	__HAL_TIM_SET_COUNTER(&htim3,0);
	while (ads1256_HAL_is_DRDY_high()){
		if ( (__HAL_TIM_GET_COUNTER(&htim3)) > DRDY_SHORT_TIMEOUT_US ) {
			__enable_irq();
			return ADS1256_DRDY_TIMEOUT;
		}
	}
	status = ads1256_HAL_SPI_Transfer(1);
	if ( status != ADS1256_OK ) {
		__enable_irq();
		return status;
	}
	__enable_irq();

	return ADS1256_OK;
}

