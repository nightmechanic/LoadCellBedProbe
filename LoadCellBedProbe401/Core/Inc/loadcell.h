#ifndef __LOADCELL_H
#define __LOADCELL_H

//#include "arm_math.h"
#include "ads1256.h"

#define IIR_NUMSTAGES 	2
#define MA_BUFFER_LENGTH 128
#define SPI_TIMEOUT_US 	16
#define V5V_SAMPLES 	64
#define V5V_RATE		ADS1256_3750_SPS
#define V5V_GAIN		ADS1256_GAIN_1
#define NTC_SAMPLES 	16
#define NTC_RATE		ADS1256_3750_SPS
#define NTC_GAIN		ADS1256_GAIN_1
#define NTC_RESDIV 		10000
#define VREF			2.5
#define PROBE_RATE 		ADS1256_15000_SPS
#define PROBE_GAIN 		ADS1256_GAIN_8
#define LOAD_SAMPLES	64
#define LOAD_RATE		ADS1256_3750_SPS
#define LOAD_GAIN		ADS1256_GAIN_8

#define LC_RATING		1500 // grams
#define LC_COUNT		4 // number of parallel load cells
#define LC_SENSITIVITY	1 //mV/V

#define PROBE_THRESHOLD 	100.0 // grams above baseline
#define PROBE_HYSTERESIS	50.0 // grams

#define LC_IDLE_PERIOD	1000 //milliseconds (TIM2)

#define STATUS_TIME_POS 116
#define STATUS_DAY_POS 107

#define ERROR_LINE_POS 53
#define ERROR_FILE_POS 67
#define ERROR_MAX_FILENAME 16

//Board related
#define LC_CH 	ADS1256_AIN0
#define REF_CH	ADS1256_AIN1
#define V5V_CH 	ADS1256_AIN2
#define NTC1_CH ADS1256_AIN3
#define NTC2_CH ADS1256_AIN4
#define NTC3_CH	ADS1256_AIN5
#define NTC4_CH	ADS1256_AIN6

#define IIR_COEFFS { 0.00482434,  0.00964869,  0.00482434, \
					1.04859958, -0.29614036,  1.0, \
					2.0,          1.0,          1.32091343, \
					-0.63273879 }


typedef struct{
	float32_t V5Voltage;
	float32_t Temperature;
	float32_t Load;
}LC_StatusStruct;

typedef enum
{
  LC_IDLE = 0U,
  LC_PREPARE,
  LC_RUNNING,
  LC_STOPPING,
} LC_ModeTypeDef;


typedef enum
{
  LC_OK = 0U,
  LC_ADC_BUSY,
  LC_ADC_ERROR,
  LC_SPI_TIMEOUT,
  LC_ERROR,
} LC_StatusTypeDef;



void delay_us (uint16_t us);
LC_StatusTypeDef lc_measure_5V(float32_t *V5Voltage);
LC_StatusTypeDef lc_measure_ntc_R(ADS1256_CHANNELS ntc_ch, float32_t *NTC_Resistance);
float32_t lc_ntc_r2tempC(float32_t ntc_resistance);
void lc_do_lc_idle(void);
void lc_do_lc_prepare(void);
void lc_do_lc_running(void);
void lc_do_lc_stopping(void);
void lc_convert_and_send_data(float32_t * Buf, uint8_t Len);
void lc_send_greeting(void);
void lc_Error_Handler(const char * FileName, uint8_t NameLength, uint16_t LineNumber);


#endif //LOADCELL_H
