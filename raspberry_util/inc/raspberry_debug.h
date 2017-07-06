#ifndef __RASPBERRY_DEBUG_H__
#define __RASPBERRY_DEBUG_H__


/*
 *---------------------------------------------------------------------
 * Marcos for Debug
 *---------------------------------------------------------------------
*/
#ifdef RASPBERRY_DEBUG_ENABLE

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#define _RASPBERRY_DEBUG_PRINT(...) printf(__VA_ARGS__)
/*#define _RASPBERRY_DEBUG_PRINT2(fmt,...) printf(fmt,__VA_ARGS__)*/

#define RASPBERRY_DEBUG_MESSAGE(level,...) \
do{\
	{\
		struct tm *time_hms;\
		struct timeval time_ms;\
		gettimeofday(&time_ms,NULL);\
		time_hms = localtime(&time_ms.tv_sec);\
		_RASPBERRY_DEBUG_PRINT( "[RASPBERRY_UTIL][%s][%d:%d:%d.%ld][%s]",\
							    level,\
								time_hms->tm_hour,\
								time_hms->tm_min,\
								time_hms->tm_sec,\
								time_ms.tv_usec,\
								__func__);\
		_RASPBERRY_DEBUG_PRINT(__VA_ARGS__);\
	}\
}while(0)

#define RASPBERRY_DEBUG_INFO(...) RASPBERRY_DEBUG_MESSAGE("INFO",__VA_ARGS__)
#define RASPBERRY_DEBUG_ERROR(...) RASPBERRY_DEBUG_MESSAGE("ERROR",__VA_ARGS__)

#else

#define RASPBERRY_DEBUG_INFO(...) 

#endif

#endif /* ifndef __RASPBERRY_DEBUG_H__ */
