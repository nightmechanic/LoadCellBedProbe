// ADS1256 typedefs from : https://github.com/Zatrac/ADS1256-driver/blob/master/ADS1256_Driver.h


#ifndef __ADS1256_H
#define __ADS1256_H

#include "arm_math.h"

#define ADS_RESET_LENGTH_US	20
#define DRDY_SHORT_TIMEOUT_US 100
#define DRDY_CAL_TIMEOUT_US	10000//assuming nothing slower than 500SPS
#define CMD_TO_DATA_US	8

typedef enum {
	REG_STATUS			= 0x00,		// b(ID3, ID2, ID1, ID0, ORDER, ACAL, BUFEN, DRDY)
	REG_MUX				= 0x01,		// b(PSEL3, PSEL2, PSEL1, PSEL0, NSEL3, NSEL2, NSEL1, NSEL0)
	REG_ADCON			= 0x02,		// b(0, CLK1, CLK0, SDCS1, SDCS0, PGA2, PGA1, PGA0)
	REG_DRATE			= 0x03,		// b(DR7, DR6, DR5, DR4, DR3, DR2, DR1, DR0)
	REG_IO				= 0x04,		// b(DIR3, DIR2, DIR1, DIR0, DIO3, DIO2, DIO1, DIO0)
	REG_OFC0			= 0x05,		// b(OFC07, OFC06, OFC05, OFC04, OFC03, OFC02, OFC01, OFC00)
	REG_OFC1			= 0x06,		// b(OFC15, OFC14, OFC13, OFC12, OFC11, OFC10, OFC09, OFC08)
	REG_OFC2			= 0x07,		// b(OFC23, OFC22, OFC21, OFC20, OFC19, OFC18, OFC17, OFC16)
	REG_FSC0			= 0x08,		// b(FSC07, FSC06, FSC05, FSC04, FSC03, FSC02, FSC01, FSC00)
	REG_FSC1			= 0x09,		// b(FSC15, FSC14, FSC13, FSC12, FSC11, FSC10, FSC09, FSC08)
	REG_FSC2			= 0x0A,		// b(FSC23, FSC22, FSC21, FSC20, FSC19, FSC18, FSC17, FSC16)
} ADS1256_REGISTERS;


/**
* Command definitions as defined in the datasheet
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 34)
*/
typedef enum {
	CMD_WAKEUP			= 0x00,		// Completes SYNC and exits standby mode
	CMD_RDATA			= 0x01,		// Read Data
	CMD_RDATAC			= 0x03,		// Read Data Continuously
	CMD_SDATAC			= 0x0F,		// Stop Read Data Continuously
	CMD_RREG			= 0x10,		// Read from REG rrrr
	CMD_WREG			= 0x50,		// Write from REG rrrr
	CMD_SELFCAL			= 0xF0,		// Offset and Gain Self-Calibration
	CMD_SELFOCAL		= 0xF1,		// Offset Self-Calibration
	CMD_SELFGCAL		= 0xF2,		// Gain Self-Calibration
	CMD_SYSOCAL			= 0xF3,		// System Offset Calibration
	CMD_SYSGCAL			= 0xF4,		// System Gain Calibration
	CMD_SYNC			= 0xFC,		// Synchronize the A/D Conversion
	CMD_STANDBY			= 0xFD,		// Begin Standby Mode
	CMD_RESET			= 0xFE,		// Reset to Power-Up Values
	CMD_WAKEUP_1		= 0xFF,		// Completes SYNC and exits standby mode
} ADS1256_COMMANDS;


/**
* Programmable gain amplifier levels with respective register values
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 31)
*/
typedef enum {
	ADS1256_GAIN_1		= 0x00,		// Default
	ADS1256_GAIN_2		= 0x01,
	ADS1256_GAIN_4		= 0x02,
	ADS1256_GAIN_8		= 0x03,
	ADS1256_GAIN_16		= 0x04,
	ADS1256_GAIN_32		= 0x05,
	ADS1256_GAIN_64		= 0x06,
	ADS1256_GAIN_MAX	= 0x07,		// Equals 64 Gain
} ADS1256_GAIN;


/**
* Available data rates with their respective register values
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 18)
*/
typedef enum {
	ADS1256_30000_SPS	= 0xF0,		// 1 Average (Bypassed) Default
	ADS1256_15000_SPS	= 0xE0,		// 2 Averages
	ADS1256_7500_SPS	= 0xD0,		// 4 Averages
	ADS1256_3750_SPS	= 0xC0,		// 8 Averages
	ADS1256_2000_SPS	= 0xB0,		// 15 Averages
	ADS1256_1000_SPS	= 0xA1,		// 30 Averages
	ADS1256_500_SPS		= 0x92,		// 60 Averages
	ADS1256_100_SPS		= 0x82,		// 300 Averages
	ADS1256_60_SPS		= 0x72,		// 500 Averages
	ADS1256_50_SPS		= 0x63,		// 600 Averages
	ADS1256_30_SPS		= 0x53,		// 1000 Averages
	ADS1256_25_SPS		= 0x43,		// 1200 Averages
	ADS1256_15_SPS		= 0x33,		// 2000 Averages
	ADS1256_10_SPS		= 0x23,		// 3000 Averages
	ADS1256_5_SPS		= 0x13,		// 6000 Averages
	ADS1256_2d5_SPS		= 0x03,		// 12000 Averages
} ADS1256_DRATE;


/**
* Available channels and their MUX values
* https://www.ti.com/lit/ds/symlink/ads1256.pdf (Page 31)
*/
typedef enum {
	ADS1256_AIN0		= 0x00,
	ADS1256_AIN1		= 0x01,
	ADS1256_AIN2		= 0x02,
	ADS1256_AIN3		= 0x03,
	ADS1256_AIN4		= 0x04,
	ADS1256_AIN5		= 0x05,
	ADS1256_AIN6		= 0x06,
	ADS1256_AIN7		= 0x07,
	ADS1256_AINCOM		= 0x08,
} ADS1256_CHANNELS;


typedef enum
{
	ADS1256_OK = 0U,
	ADS1256_DRDY_TIMEOUT,
	ADS1256_SPI_TIMEOUT,
	ADS1256_MISMATCH,
	ADS1256_ERROR,
} ADS1256C_StatusTypeDef;


ADS1256C_StatusTypeDef ads1256_init(void);
ADS1256C_StatusTypeDef ads1256_configure(ADS1256_DRATE ads_rate, ADS1256_GAIN ads_gain,
										ADS1256_CHANNELS ads_pchannel, ADS1256_CHANNELS ads_nchannel,
										uint8_t buf_en);
ADS1256C_StatusTypeDef ads1256_get_samples(float32_t * sample_buffer, uint16_t sample_count, float32_t conversion_factor);
ADS1256C_StatusTypeDef ads1256_send_command(ADS1256_COMMANDS command);

#endif //__ADS1256_H
