#ifndef __BME280_DEBUG_H__
#define __BME280_DEBUG_H__

#ifdef BME280_DEBUG_ENABLE

#ifdef BME280_DEBUG_KERNEL /*Debug for Linux*/
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#define BME280_PRINTF(...) printf(__VA_ARGS__)
#define BME280_DEBUG_MESSAGE(level,...) \
do{\
	{\
		struct tm *bme280_time_hms;\
		struct timeval bme280_time_ms;\
		gettimeofday(&bme280_time_ms,NULL);\
		bme280_time_hms = localtime(&(bme280_time_ms.tv_sec));\
		BME280_PRINTF("[BME280][%s][%d:%d:%d.%d][%s]",\
						  level,\
						  bme280_time_hms->tm_hour,\
						  bme280_time_hms->tm_min,\
						  bme280_time_hms->tm_sec,\
						  bme280_time_ms.tv_usec,\
						  __func__);\
		BME280_PRINTF(__VA_ARGS__);\
	}\
}while(0)

#define BME280_DEBUG_INFO(...) BME280_DEBUG_MESSAGE("INFO", __VA_ARGS__)
#define BME280_DEBUG_ERROR(...) BME280_DEBUG_MESSAGE("ERROR", __VA_ARGS__)
#define BME280_DEBUG_FUNC_RETURN(ret) BME280_DEBUG_INFO("Return : %d\n",ret)

#endif

#else

#define BME280_DEBUG_INFO(...) 
#define BME280_DEBUG_ERROR(...) 
#define BME280_DEBUG_FUNC_RETURN(ret) 

#endif

#endif /* #ifndef __BME280_DEBUG_H__ */
