#include "main.h"
#include "loadcell.h"
#include "arm_math.h"
//#include <stdio.h>
//#include "ads1256.h"
#include "utilities.h"
#include "usbd_cdc_if.h"

static const char ThisFileName[] = "loadcell.c";

const char __attribute__ ((aligned(4))) greeting_message[] =
		"###########################################\n"
		"Hello!\n"
		"Load Cell Bed Probe FW Version 0.1\n"
		"\xA9 Nightmechanic, May 2023\n"
		"###########################################\n";

char __attribute__ ((aligned(4))) status_message[] =
		"Current Status:\n"
		"Supply Voltage:         V\n"
		"Temperature:         C\n"
		"Load:            grams\n"
		"Time from reset: dddDays, HHh:MMm:SSs\n";

char __attribute__ ((aligned(4))) line_error_message[] =
		"***********************************\n"
		"Error !!!\n"
		"Line xxxx in file:                 \n"
		"***********************************\n";


float32_t ProbeDataBuffer[128];
uint8_t ProbeDataPointer = 0;
uint8_t ProbeStarted = 0;
uint8_t LastCollectedPointer = 64;
const float32_t GramScaleFactor = 0.000894069671631f; //5V/23bit/pga_gain/mV/ext_gain/(sensitivity*5/max_load/num_cells)
float32_t ProbeScaleFactor = GramScaleFactor * 1.0;
float32_t ProbeThresholdUp;
float32_t ProbeThresholdDn;
uint32_t IdleCounter = 0;
uint8_t dayFlag = 0;
uint8_t dayDigit[3] = {0};

char __attribute__ ((aligned(4))) TxBuffer[512];


// Utility

void delay_us (uint16_t us)
{
	LL_TIM_SetCounter(TIM3, 0);  // set the counter value a 0
	while ( LL_TIM_GetCounter(TIM3) < us);  // wait for the counter to reach the us input in the parameter
}

//LoadCell functions




float32_t lc_ntc_r2tempC(float32_t ntc_resistance) {
	const float32_t A = 0.5926474403e-3;
	const float32_t B = 2.316204755e-4;
	const float32_t C = 0.6208333430e-7;
	float32_t Temperature;

	Temperature = 1 / (A + B * logf(ntc_resistance) + C * powf(logf(ntc_resistance), 3)) - 273.15;

	return Temperature;
}

LC_StatusTypeDef lc_measure_ntc_R(ADS1256_CHANNELS ntc_ch, float32_t *NTC_Resistance){
	ADS1256C_StatusTypeDef ads_status;
	float32_t res_buf[NTC_SAMPLES];
	const float32_t vref_uv = 1e6f * VREF;
	const float32_t uV_scale_factor = 5.96046e-1f; //5V/23bit/gain/uV
	uint16_t index;
	*NTC_Resistance = 0;

	ads_status =  ads1256_configure(NTC_RATE, NTC_GAIN,
			ntc_ch, ADS1256_AINCOM, 0);
	if (ads_status != ADS1256_OK){
		return LC_ADC_ERROR;
	}

	ads_status = ads1256_get_samples(res_buf, NTC_SAMPLES, uV_scale_factor);
		if (ads_status != ADS1256_OK){
			return LC_ADC_ERROR;
		}

	for (index = 0; index<NTC_SAMPLES; index++){
			*NTC_Resistance += (res_buf[index] * NTC_RESDIV) / (vref_uv - res_buf[index]);
		}

	*NTC_Resistance = *NTC_Resistance / NTC_SAMPLES;

	return LC_OK;
}

LC_StatusTypeDef lc_measure_5V(float32_t *V5Voltage){

	ADS1256C_StatusTypeDef ads_status;
	float32_t voltage_buf[V5V_SAMPLES];
	const float32_t vref_uv = 1e6f * VREF;
	const float32_t uV_scale_factor = 5.96046e-1f; //5V/23bit/gain/uV
	uint16_t index;
	*V5Voltage = 0.0;

	ads_status =  ads1256_configure(V5V_RATE, V5V_GAIN,
						V5V_CH, REF_CH, 0);
	if (ads_status != ADS1256_OK){
		return LC_ADC_ERROR;
	}

	ads_status = ads1256_get_samples(voltage_buf, V5V_SAMPLES, uV_scale_factor);
	if (ads_status != ADS1256_OK){
		return LC_ADC_ERROR;
	}

	//lets average
	for (index = 0; index<V5V_SAMPLES; index++){
		*V5Voltage += voltage_buf[index];
	}

	*V5Voltage = (*V5Voltage / V5V_SAMPLES) + vref_uv;

	return LC_OK;
}

LC_StatusTypeDef lc_measure_load(float32_t *Load){

	ADS1256C_StatusTypeDef ads_status;
	float32_t load_buf[LOAD_SAMPLES];
	uint16_t index;
	*Load = 0.0;

	ads_status =  ads1256_configure(LOAD_RATE, LOAD_GAIN,
						LC_CH, REF_CH, 0);
	if (ads_status != ADS1256_OK){
		return LC_ADC_ERROR;
	}

	ads_status = ads1256_get_samples(load_buf, LOAD_SAMPLES, ProbeScaleFactor);
	if (ads_status != ADS1256_OK){
		return LC_ADC_ERROR;
	}

	//lets average
	for (index = 0; index<LOAD_SAMPLES; index++){
		*Load += load_buf[index];
	}

	*Load = (*Load / LOAD_SAMPLES);

	return LC_OK;

}

LC_StatusTypeDef lc_send_status(float32_t *values){
	return LC_OK;
}


void lc_do_lc_idle(void){

	LC_StatusStruct measurements;
	float32_t resistance;
	LC_StatusTypeDef status;
	char valueBuf[10];
	uint8_t index;
	uint8_t numBytes;
	uint32_t utime;

	const char digits[] = "0123456789";

	IdleCounter++;
	// Measure voltage
	status = lc_measure_5V(&measurements.V5Voltage);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}


	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
		return;
	}

	//Measure temp
	status = lc_measure_ntc_R(NTC1_CH, &resistance);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}
	measurements.Temperature = lc_ntc_r2tempC(resistance);

	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
			return;
	}

	status = lc_measure_ntc_R(NTC2_CH, &resistance);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}
	measurements.Temperature += lc_ntc_r2tempC(resistance);

	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
		return;
	}

	status = lc_measure_ntc_R(NTC3_CH, &resistance);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}
	measurements.Temperature += lc_ntc_r2tempC(resistance);

	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
		return;
	}

	status = lc_measure_ntc_R(NTC4_CH, &resistance);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}
	measurements.Temperature += lc_ntc_r2tempC(resistance);
	//Average all NTCs
	measurements.Temperature = measurements.Temperature / 4;

	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
		return;
	}

	ProbeScaleFactor = (measurements.V5Voltage * 1e6)/5.0 * GramScaleFactor;

	status = lc_measure_load(&measurements.Load);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}

	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
		return;
	}


	// lets update the status message:

	//numBytes = snprintf(valueBuf, 8, "%1.5f", measurements.V5Voltage);
	numBytes = float_to_char_opt(measurements.V5Voltage, valueBuf, 8, 10, 5);

	for (index = 0; index<numBytes; index++){
		status_message[32+index] = valueBuf[index];
	}

	//numBytes = snprintf(valueBuf, 8, "%3.1f", measurements.Temperature);
	numBytes = float_to_char_opt(measurements.Temperature, valueBuf, 8, 10, 1);


	for (index = 0; index<numBytes; index++){
		status_message[55+index] = valueBuf[index];
	}

	//numBytes = snprintf(valueBuf, 8, "%4.3f", measurements.Load);
	numBytes = float_to_char_opt(measurements.Load, valueBuf, 8, 10, 3);

	for (index = 0; index<numBytes; index++){
		status_message[73+index] = valueBuf[index];
	}

	if ( LL_GPIO_IsInputPinSet(Control_GPIO_Port, Control_Pin) ){
		return;
	}


	utime = LL_RTC_TIME_Get(RTC);

	//Hours
	status_message[STATUS_TIME_POS] = digits[(utime>>20) & 0x0f];
	status_message[STATUS_TIME_POS+1] = digits[(utime>>16) & 0x0f];
	//Minutes
	status_message[STATUS_TIME_POS+4] = digits[(utime>>12) & 0x0f];
	status_message[STATUS_TIME_POS+5] = digits[(utime>>8) & 0x0f];
	//Seconds
	status_message[STATUS_TIME_POS+8] = digits[(utime>>4) & 0x0f];
	status_message[STATUS_TIME_POS+9] = digits[utime & 0x0f];

	//Has a day passed?
	utime = (utime>>20) & 0x0f;

	if (utime > 0){		// are we passed 10:00?
		dayFlag = 1;
	} else if (dayFlag == 1) { // are we passed 23:59:59 ?
		dayDigit[0] += 1;
		dayFlag = 0;
	}

	//Day counter wrap-around
	if (dayDigit[0] >= 10){
		dayDigit[0] = 0;
		dayDigit[1] += 1;
		if (dayDigit[1] >= 10){
			dayDigit[1] = 0;
			dayDigit[2] += 1;
			if (dayDigit[2] >= 10){
				dayDigit[2] = 0;
			}
		}
	}
	status_message[STATUS_DAY_POS] = digits[dayDigit[2]];
	status_message[STATUS_DAY_POS + 1] = digits[dayDigit[1]];
	status_message[STATUS_DAY_POS + 2] = digits[dayDigit[0]];

	CDC_Transmit_FS((uint8_t *)status_message, sizeof(status_message));

}

void lc_do_lc_prepare(void){
	ADS1256C_StatusTypeDef ads_status;
	LC_StatusTypeDef status;
	float32_t V5Vmeasurement = 5.0;
	float32_t Load = 0.0;


	status = lc_measure_5V(&V5Vmeasurement);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}

	ProbeScaleFactor = (V5Vmeasurement * 1e6)/5.0 * GramScaleFactor;

	status = lc_measure_load(&Load);
	if (status != LC_OK) {
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}

	ProbeThresholdUp = Load + PROBE_THRESHOLD;
	ProbeThresholdDn = ProbeThresholdUp - PROBE_HYSTERESIS;

	ads_status =  ads1256_configure(PROBE_RATE, PROBE_GAIN,
							LC_CH, REF_CH, 0);
	if (ads_status != ADS1256_OK){
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}

	ads_status = ads1256_send_command(CMD_RDATAC);
	if (ads_status != ADS1256_OK){
		lc_Error_Handler( ThisFileName, sizeof(ThisFileName), __LINE__);
	}

	ProbeDataPointer = 0;
	ProbeStarted = 0;
	LastCollectedPointer = 64;

	__disable_irq();

	NVIC_EnableIRQ(DMA1_Stream3_IRQn);
	NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);

	NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
	NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);

	__enable_irq();

}

void lc_do_lc_running(void){

	if (LastCollectedPointer == 64){
		if (ProbeDataPointer > 64) {
			ProbeStarted = 1;
			LastCollectedPointer = 0;
			lc_convert_and_send_data(&ProbeDataBuffer[LastCollectedPointer], 64);
		}
	} else if (ProbeStarted==1) {
		if (ProbeDataPointer < 64){
			LastCollectedPointer = 64;
			lc_convert_and_send_data(&ProbeDataBuffer[LastCollectedPointer], 64);

		}
	}
}

void lc_do_lc_stopping(void){

	__disable_irq();

	NVIC_DisableIRQ(DMA1_Stream3_IRQn);
	NVIC_DisableIRQ(DMA1_Stream4_IRQn);
	NVIC_DisableIRQ(EXTI15_10_IRQn);

	__enable_irq();


	//de-assert CS and clear the DRDY IRQ
	LL_GPIO_SetOutputPin(ADC_CS_GPIO_Port, ADC_CS_Pin);
	LL_SPI_Disable(SPI2);

	if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_12) != RESET)
	{
	LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_12);
	}

	// stop the DMAs and clear any pending flags
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_4); //TX
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_3); //RX

	LL_DMA_ClearFlag_TC4(DMA1);
	LL_DMA_ClearFlag_TE4(DMA1);
	LL_DMA_ClearFlag_TC3(DMA1);
	LL_DMA_ClearFlag_TE3(DMA1);


	NVIC_ClearPendingIRQ(DMA1_Stream3_IRQn);
	NVIC_ClearPendingIRQ(DMA1_Stream4_IRQn);
	NVIC_ClearPendingIRQ(EXTI15_10_IRQn);



}

void lc_convert_and_send_data(float32_t * Buf, uint8_t Len){
	uint8_t index;


	for (index = 0; index < Len; index++) {
		float_to_char_opt(Buf[index], &TxBuffer[index*8], 7, 10, 3);
		TxBuffer[(index*8)+7] = ',';
	}

	CDC_Transmit_FS((uint8_t *)TxBuffer, (Len * 8));


}

void lc_send_greeting(void){
	CDC_Transmit_FS((uint8_t *)greeting_message, sizeof(greeting_message));
}

void lc_Error_Handler(const char * FileName, uint8_t NameLength, uint16_t LineNumber)
{
	uint8_t i;
	uint16_t rem, val;
	const char digits[] = "0123456789";
	// Set probe output
	LL_GPIO_ResetOutputPin(Probe_Out_GPIO_Port, Probe_Out_Pin);

	// insert line number
	for (i=0; i<4; i++) {
		rem = (uint16_t)(LineNumber / 10);
		val = LineNumber - (rem * 10);
		line_error_message[ERROR_LINE_POS + 3 - i] = digits[val];
		LineNumber = rem;
	}
	// insert filename
	if (NameLength > ERROR_MAX_FILENAME) {
		NameLength = ERROR_MAX_FILENAME;
	}
	for (i=0; i < NameLength; i++){
		line_error_message[ERROR_FILE_POS + i] = FileName[i];
	}

	while (CDC_Transmit_FS((uint8_t *)line_error_message, sizeof(line_error_message)) == USBD_FAIL){

	}

	while ( CDC_Transmit_FS((uint8_t *)line_error_message,0) == USBD_BUSY ){

	}

	__disable_irq();
	while (1)
	{
	}

}

