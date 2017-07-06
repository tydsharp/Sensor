#ifndef __BME280_H__
#define __BME280_H__

#include "bme280_types.h"

enum
{
	BME280_MODE_WEATHER_MONITORING = 0, 
	BME280_MODE_HUMIDITY_SENSING ,
	BME280_MODE_INDOOR_NAVIGATION ,
};

struct bme280_spi_bus_if_t {
	s32 (*write_read)(u8* wr_buf, u16 wr_len, u8* rd_buf, u16 rd_len);
};

struct bme280_i2c_bus_if_t {
	s32 (*read)(u16 dev_addr, u8* rd_buf, u16 len);
	s32 (*write)(u16 dev_addr, u8* wr_buf, u16 len);
};

typedef struct {
	union {
		struct bme280_i2c_bus_if_t i2c;
		struct bme280_spi_bus_if_t spi;
	} spi_i2c;
	void (*delay_msec)(uint32_t msecs);
}bme280_bus_if_t;

void bme280_init(bme280_bus_if_t* bme280_bus_if);
void bme280_measure_start(int32_t mode);
void bme280_measure_data_get_data(double *tempereture,double *humidity,double *pressure);
void bme280_close(void);

#endif
 