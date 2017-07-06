#include <stdio.h>
#include <unistd.h>
#include "bme280_app.h"
#include "raspberry_util.h"
#include "raspberry_debug.h"

#define MEASUREMENT_TIMES 			(1)
#define MEASUREMENT_INTERVAL 		(1)

void bme280_i2c(void)
{
	double tempereture,humidity,pressure;
	bme280_bus_if_t bme280_bus_if;

	raspberry_i2c_open();

	/*register if to bme280_bus_if_t*/
	bme280_bus_if.spi_i2c.i2c.read = raspberry_i2c_read;
	bme280_bus_if.spi_i2c.i2c.write = raspberry_i2c_write;
	bme280_bus_if.delay_msec = raspberry_delay;	
	bme280_init(&bme280_bus_if);
	
	int32_t times = 0;	

	while(times++ < MEASUREMENT_TIMES)
	{
		printf("############loop[%d]###############\n",times);
		bme280_measure_start(BME280_MODE_INDOOR_NAVIGATION);
		bme280_measure_data_get(&tempereture,&humidity,&pressure);
		printf("temp=%f,humi=%f,pressure=%f\n",tempereture,humidity,pressure);
		sleep(MEASUREMENT_INTERVAL);
	}
	
	bme280_close();
	raspberry_i2c_close();
	
}

int32_t spi_write_read_bme280(uint8_t *wr_buf, uint16_t wr_len, uint8_t *rd_buf,uint16_t rd_len)
{
	raspberry_spi_write_read(RASPBERRY_SPI_CHANNEL_0,wr_buf,wr_len, rd_buf, rd_len);
}

void bme280_spi_4w(void)
{
	double tempereture,humidity,pressure;
	bme280_bus_if_t bme280_bus_if;
	raspberry_spi_info_t raspberry_spi_info = {0};
	
	raspberry_spi_info.timing_mode = RASPBERRY_SPI_MODE_CPHAH_CPOLH;
	raspberry_spi_info.is_3w = RASPBERRY_SPI_3W_DISABLE;
	raspberry_spi_info.speed_hz = 2500000;
	raspberry_spi_info.delay = 20;
	raspberry_spi_info.bpw = 8;
	/*open spi for bme280*/
	raspberry_spi_open(RASPBERRY_SPI_CHANNEL_0,&raspberry_spi_info);
	
	/*register if to bme280_bus_if_t*/
	bme280_bus_if.spi_i2c.spi.write_read = spi_write_read_bme280;
	bme280_bus_if.delay_msec = raspberry_delay;
	
	bme280_init(&bme280_bus_if);
	
	int32_t times = 0;

	while(times++ < MEASUREMENT_TIMES)
	{
		printf("############loop[%d]###############\n",times);
		bme280_measure_start(BME280_MODE_INDOOR_NAVIGATION);
		bme280_measure_data_get(&tempereture,&humidity,&pressure);
		printf("temp=%f,humi=%f,pressure=%f\n",tempereture,humidity,pressure);
		sleep(MEASUREMENT_INTERVAL);
	}
	
	bme280_close();
	raspberry_spi_close(RASPBERRY_SPI_CHANNEL_0);
}

void bme280_spi_3w(void)
{
	double tempereture,humidity,pressure;
	bme280_bus_if_t bme280_bus_if;
	raspberry_spi_info_t raspberry_spi_info = {0};
	
	raspberry_spi_info.timing_mode = RASPBERRY_SPI_MODE_CPHAL_CPOLL;
	raspberry_spi_info.is_3w = RASPBERRY_SPI_3W_ENABLE;	
	raspberry_spi_info.speed_hz = 2500000;
	raspberry_spi_info.delay = 20;
	raspberry_spi_info.bpw = 8;
	/*open spi for bme280*/
	raspberry_spi_open(RASPBERRY_SPI_CHANNEL_0,&raspberry_spi_info);
	
	/*register if to bme280_bus_if_t*/
	bme280_bus_if.spi_i2c.spi.write_read = spi_write_read_bme280;
	bme280_bus_if.delay_msec = raspberry_delay;
	
	bme280_init(&bme280_bus_if);
	
	int32_t times = 0;

	while(times++ < MEASUREMENT_TIMES)
	{
		printf("############loop[%d]###############\n",times);
		bme280_measure_start(BME280_MODE_INDOOR_NAVIGATION);
		bme280_measure_data_get(&tempereture,&humidity,&pressure);
		printf("temp=%f,humi=%f,pressure=%f\n",tempereture,humidity,pressure);
		sleep(MEASUREMENT_INTERVAL);
	}
	
	bme280_close();
	raspberry_spi_close(RASPBERRY_SPI_CHANNEL_0);
}

int main(int argc, char * argv[])
{
	//bme280_i2c();
	bme280_spi_4w();
	//bme280_spi_3w();
	
	return 0;
}
