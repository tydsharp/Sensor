#ifndef __RASPBERRY_UTIL_H__
#define __RASPBERRY_UTIL_H__

#include <stdint.h>

/*
 *----------------------------------------------------------------------
 * Define for SPI
 *----------------------------------------------------------------------
*/
#define RASPBERRY_SPI_MODE_CPHAH_CPOLH 	(1)
#define RASPBERRY_SPI_MODE_CPHAL_CPOLH 	(2)
#define RASPBERRY_SPI_MODE_CPHAH_CPOLL 	(3)
#define RASPBERRY_SPI_MODE_CPHAL_CPOLL 	(4)

#define RASPBERRY_SPI_3W_ENABLE			(1)
#define RASPBERRY_SPI_3W_DISABLE		(0)

#define RASPBERRY_SPI_LSB_ENABLE		(1)
#define RASPBERRY_SPI_MSB_ENABLE		(0)

#define RASPBERRY_SPI_CS_ACTIVE_HIGH	(1)
#define RASPBERRY_SPI_CS_ACTIVE_LOW		(0)

#define RASPBERRY_ERROR					(-1)
#define RASPBERRY_OK					(0)

#define RASPBERRY_NULL					(0)

enum{
	RASPBERRY_SPI_CHANNEL_0 = 0,		/*spi channel 0*/
	RASPBERRY_SPI_CHANNEL_1,	        /*spi channel 1*/
};

typedef struct {
	/*
		speed_hz = Core Clock / CDIV
		If CDIV is set to 0, the divisor is 65536. The divisor must be a power of 2. Odd numbers rounded down. The maximum SPI clock rate is of the APB clock
	*/	
	uint32_t 	speed_hz; 	
	uint16_t 	delay;
	uint8_t 	bpw;			/*Bit Per Word*/
	uint8_t 	timing_mode;
	uint8_t		is_3w; 			/*1:3w,0:4w*/
	uint8_t		is_lsb; 		/*1:lsb,0:msb*/
	uint8_t     is_cs_high;		/*1:cs active high , 0:cs active low*/
}raspberry_spi_info_t;
/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
int32_t raspberry_spi_open(int32_t spi_channel,raspberry_spi_info_t * spi_info);
/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
int32_t raspberry_spi_close(int32_t spi_channel);
/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
int32_t raspberry_spi_write(int32_t spi_channel, uint8_t *wr_buf, uint16_t len);
int32_t raspberry_spi_read(int32_t spi_channel, uint8_t *rd_buf, uint16_t len);
int32_t raspberry_spi_write_read(int32_t spi_channel, uint8_t* t_buf, uint16_t wr_len, uint8_t *r_buf, uint16_t rd_len);
/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/

int32_t raspberry_i2c_open(void);
/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
int32_t raspberry_i2c_close(void);

/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
int32_t raspberry_i2c_read(uint16_t dev_addr, uint8_t* rd_buf, uint16_t len);

/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
int32_t raspberry_i2c_write(uint16_t dev_addr, uint8_t* wr_buf, uint16_t len);

/*
 *----------------------------------------------------------------------
 *	@Brief 
 *		
 *
 *	@Param
 *		[IN/OUT] param Description
 *
 *	@Return
 *
 *----------------------------------------------------------------------
*/
void raspberry_delay(uint32_t ms);

#endif