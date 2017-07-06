#include <string.h>
#include "bme280_drv.h"
#include "bme280_app.h"
#include "bme280_debug.h"

#define SPI_READ				(0x80)
#define SPI_WRITE				(0x7F)
#define I2C_READ				(0x01)
#define I2C_WRITE				(0xFE)
#define	I2C_BUFFER_LEN 			(128)
#define BUS_BUF_LEN_MAX 		(26)

/*#######################################################################
 * Local Function
 ########################################################################*/
static bme280_bus_if_t g_bme280_bus_if = {0};

/*
 * For the SPI mode only 7 bits of register addresses are used.
 *  This is a full duplex operation 
*/
static s8 bme280_spi_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	u8 spi_wr_buf[BUS_BUF_LEN_MAX] = {0};
	u8 spi_rd_buf[BUS_BUF_LEN_MAX] = {0};
	s32 offset = 0;

	spi_wr_buf[0] = reg_addr | SPI_READ;
#if defined (BME280_ENABLE_BUS_CONNECT_SPI_4W)
	g_bme280_bus_if.spi_i2c.spi.write_read( spi_wr_buf, 1, spi_rd_buf ,cnt );
#else
	g_bme280_bus_if.spi_i2c.spi.write_read( spi_wr_buf, 1, spi_rd_buf, cnt );
#endif
	for(offset=0; offset < cnt; offset++)
	{
		*(reg_data+offset) = spi_rd_buf[offset];
	}
	return (s8)0;
}

/*
 * For the SPI mode only 7 bits of register addresses are used.
 *  This is a full duplex operation 
*/
static s8 bme280_spi_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	u8 spi_wr_buf[BUS_BUF_LEN_MAX] = {0};
	u8 spi_rd_buf[BUS_BUF_LEN_MAX] = {0};
	u32 reg_data_offset = 0;
	u32 index = 0;
	for(index = 0; index < cnt; index+=2,reg_data_offset++)
	{
		/*Contorl Byte : Register Address*/
		spi_wr_buf[index] = reg_addr & SPI_WRITE;
		/*Register Value*/
		spi_wr_buf[index+1] = reg_data[reg_data_offset];
		/*To Next Register Address*/
		reg_addr += 1;
	}
#if defined (BME280_ENABLE_BUS_CONNECT_SPI_4W)
	g_bme280_bus_if.spi_i2c.spi.write_read( spi_wr_buf, cnt * 2, spi_rd_buf, 0 );
#else
	g_bme280_bus_if.spi_i2c.spi.write_read( spi_wr_buf, cnt * 2, spi_rd_buf, 0 );
#endif
	return (s8)0;
}


/*
 * For the I2C mode only 7 bits of slave addresses are used.
*/
static s8 bme280_i2c_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	u8 i2c_wr_buf[BUS_BUF_LEN_MAX] = {0};
	u8 i2c_rd_buf[BUS_BUF_LEN_MAX] = {0};
	u32 reg_data_offset = 0;
	u32 index = 0;
	
	/*set slave address with write flag to first byte*/
	for(index = 0; index < cnt; index+=2,reg_data_offset++)
	{
		/*Register Address*/
		i2c_wr_buf[index] = reg_addr;
		/*Register Value*/
		i2c_wr_buf[index+1] = reg_data[reg_data_offset];
		/*To Next Register Address*/
		reg_addr += 1;
	}
	/*send data*/
	g_bme280_bus_if.spi_i2c.i2c.write((u16)dev_addr,i2c_wr_buf,cnt*2);
	return (s8)0;
}

/*
 * For the I2c mode only 7 bits of slave addresses are used.
*/
static s8 bme280_i2c_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	u8 i2c_wr_buf[BUS_BUF_LEN_MAX] = {0};
	u8 i2c_rd_buf[BUS_BUF_LEN_MAX] = {0};
	s32 offset = 0;

	i2c_wr_buf[0] = reg_addr;
	g_bme280_bus_if.spi_i2c.i2c.write((u16)dev_addr,i2c_wr_buf,1);
	
	g_bme280_bus_if.spi_i2c.i2c.read((u16)dev_addr, i2c_rd_buf, cnt);
	
	for(offset = 0; offset < cnt; offset++)
	{
		*(reg_data+offset) = i2c_rd_buf[offset];
	}
	return (s8)0;
}

static void bme280_delay_msec(u32 ms)
{
	g_bme280_bus_if.delay_msec(ms);
}


/*#######################################################################
 * Interface
 ########################################################################*/
/*******************
 * Define
 ******************/
#define BME280_STATE_INIT_UNCOMPLETE 	(0)
#define BME280_STATE_INIT_COMPLETE		(1)

/*******************
 * Global variables
 ******************/
static u8 bme280_state = BME280_STATE_INIT_UNCOMPLETE;
static bme280_drv_t g_bme280_drv_info;



void bme280_init(bme280_bus_if_t* bme280_bus_if)
{
	//raspberry_util_spi_info_t spi_info = {0};
	
	if(bme280_state != BME280_STATE_INIT_UNCOMPLETE)
	{
		return;
	}

	memcpy(&g_bme280_bus_if,bme280_bus_if,sizeof(bme280_bus_if_t));
	
#if defined (BME280_ENABLE_BUS_CONNECT_SPI_4W) || defined (BME280_ENABLE_BUS_CONNECT_SPI_3W)
	/*----------------------------------------------------
	 * SPI bus access interface
	-----------------------------------------------------*/	
	g_bme280_drv_info.bus_write = bme280_spi_bus_write;
	g_bme280_drv_info.bus_read = bme280_spi_bus_read;
	
#elif defined (BME280_ENABLE_BUS_CONNECT_I2C_SDO_GND) || defined (BME280_ENABLE_BUS_CONNECT_I2C_SDO_VDD)
	/*----------------------------------------------------
	 * I2C bus access interface
	-----------------------------------------------------*/	
	g_bme280_drv_info.bus_write = bme280_i2c_bus_write;
	g_bme280_drv_info.bus_read = bme280_i2c_bus_read;
	
#if defined (BME280_ENABLE_BUS_CONNECT_I2C_SDO_GND)
	g_bme280_drv_info.dev_addr = BME280_DRV_I2C_ADDRESS1;
#else	
	g_bme280_drv_info.dev_addr = BME280_DRV_I2C_ADDRESS2;
#endif /*#if defined (BME280_ENABLE_BUS_CONNECT_I2C_SDO_GND)*/

#else
	; /* Do Nothing */
#endif /*#if defined (BME280_ENABLE_BUS_CONNECT_I2C_SDO_GND) || defined (BME280_ENABLE_BUS_CONNECT_I2C_SDO_VDD)*/

	g_bme280_drv_info.delay_msec = bme280_delay_msec;
	
	bme280_drv_init(&g_bme280_drv_info);	
	
#if defined (BME280_ENABLE_BUS_CONNECT_SPI_4W) || defined (BME280_ENABLE_BUS_CONNECT_SPI_3W)
#if defined (BME280_ENABLE_BUS_CONNECT_SPI_4W)
	BME280_DEBUG_INFO("SPI 4W Set Configure Register\n");
	/*connect bme280 with spi 4w*/
	bme280_drv_set_spi3(0);
#else
	BME280_DEBUG_INFO("SPI 3W Set Configure Register\n");
	/* connect bme280 with spi 3w */
	bme280_drv_set_spi3(1);
#endif
#endif
	
	bme280_state = BME280_STATE_INIT_COMPLETE;
}


void bme280_measure_start(s32 mode)
{
	u8 wait_time;
	
	if( bme280_state != BME280_STATE_INIT_COMPLETE)
	{
		return;
	}
	
	switch(mode)
	{
		case BME280_MODE_WEATHER_MONITORING:
			bme280_drv_set_oversamp_pressure(BME280_DRV_OVERSAMP_1X);
			bme280_drv_set_oversamp_humidity(BME280_DRV_OVERSAMP_1X);
			bme280_drv_set_oversamp_temperature(BME280_DRV_OVERSAMP_1X);
			bme280_drv_set_filter(BME280_DRV_FILTER_COEFF_OFF);
			break;
		case BME280_MODE_HUMIDITY_SENSING:
			bme280_drv_set_oversamp_pressure(BME280_DRV_OVERSAMP_SKIPPED);
			bme280_drv_set_oversamp_humidity(BME280_DRV_OVERSAMP_1X);
			bme280_drv_set_oversamp_temperature(BME280_DRV_OVERSAMP_1X);
			bme280_drv_set_filter(BME280_DRV_FILTER_COEFF_OFF);		
			break;
		case BME280_MODE_INDOOR_NAVIGATION:
			bme280_drv_set_oversamp_pressure(BME280_DRV_OVERSAMP_16X);
			bme280_drv_set_oversamp_humidity(BME280_DRV_OVERSAMP_1X);
			bme280_drv_set_oversamp_temperature(BME280_DRV_OVERSAMP_2X);
			bme280_drv_set_filter(BME280_DRV_FILTER_COEFF_16);
			break;
		default:
			; /*nothing*/
	}
	
	/* start measurement with bme280 force mode */
	bme280_drv_set_power_mode(BME280_DRV_FORCED_MODE);
	
	/* wait until measurement end */
	bme280_drv_compute_wait_time(&wait_time);
	BME280_DEBUG_INFO("measurement wait time = %d\n",wait_time);
	g_bme280_bus_if.delay_msec(wait_time);
}

/*----------------------------------------------------------------------------*
 *  struct bme280_drv_t parameters can be accessed by using bme280
 *	bme280_drv_t having the following parameters
 *	Bus write function pointer: BME280_DRV_WR_FUNC_PTR
 *	Bus read function pointer: BME280_DRV_RD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
 *---------------------------------------------------------------------------*/

/* This function is an example for reading sensor data
 *	\param: None
 *	\return: communication result
 */
void bme280_measure_data_get(double *tempereture,double *humidity,double *pressure)
{	
	/* The variable used to read compensated temperature*/
	s32 v_temperature_s32 = BME280_DRV_INIT_VALUE;
	/* The variable used to read compensated pressure*/
	u32 v_pressure_u32 = BME280_DRV_INIT_VALUE;
	/* The variable used to read compensated humidity*/
	u32 v_humidity_u32 = BME280_DRV_INIT_VALUE;
	
	if( bme280_state != BME280_STATE_INIT_COMPLETE)
	{
		return;
	}
	
	bme280_drv_read_pressure_temperature_humidity(&v_pressure_u32,&v_temperature_s32,&v_humidity_u32);
	
	*tempereture = (double)v_temperature_s32 / 100.0;
	*pressure = (double)v_pressure_u32 / 100.0;
	*humidity = (double)v_humidity_u32 / 1024.0;
}


void bme280_close(void)
{
	if( bme280_state != BME280_STATE_INIT_COMPLETE)
	{
		return;
	}
	
	bme280_drv_set_soft_rst();
	
	bme280_state = BME280_STATE_INIT_UNCOMPLETE;
}

