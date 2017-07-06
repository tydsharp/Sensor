#ifndef __ADXL345_TYPES_H__
#define __ADXL345_TYPES_H__

/*!
* @brief The following definition uses for define the data types
*
* @note While porting the API please consider the following
* @note Please check the version of C standard
*/

/**********************************************************
* These definition uses for define the C
* standard version data types
***********************************************************/
#if defined(__STDC_VERSION__)

/************************************************
 * compiler is C11 C standard
************************************************/
#if (__STDC_VERSION__ == 201112L)
#include <stdint.h>

/*unsigned integer types*/
typedef	uint8_t 				u8;		/**< used for unsigned 8bit */
typedef	uint16_t 				u16;	/**< used for unsigned 16bit */
typedef	uint32_t 				u32;	/**< used for unsigned 32bit */
typedef	uint64_t 				u64;	/**< used for unsigned 64bit */

/*signed integer types*/
typedef	int8_t 					s8;		/**< used for signed 8bit */
typedef	int16_t 				s16;	/**< used for signed 16bit */
typedef	int32_t 				s32;	/**< used for signed 32bit */
typedef	int64_t 				s64;	/**< used for signed 64bit */

/************************************************
 * compiler is C99 C standard
************************************************/
#elif (__STDC_VERSION__ == 199901L)
/* stdint.h is a C99 supported c library.
which is used to fixed the integer size*/
/************************************************/
#include <stdint.h>
/************************************************/

/*unsigned integer types*/
typedef	uint8_t 				u8;		/**< used for unsigned 8bit */
typedef	uint16_t 				u16;	/**< used for unsigned 16bit */
typedef	uint32_t 				u32;	/**< used for unsigned 32bit */
typedef	uint64_t 				u64;	/**< used for unsigned 64bit */

/*signed integer types*/
typedef int8_t 					s8;		/**< used for signed 8bit */
typedef	int16_t 				s16;	/**< used for signed 16bit */
typedef	int32_t 				s32;	/**< used for signed 32bit */
typedef	int64_t 				s64;	/**< used for signed 64bit */

/************************************************
 * compiler is C89 or other C standard
************************************************/
#else

typedef	signed char  			s8;		/**< used for signed 8bit */
typedef	signed short int 		s16;	/**< used for signed 16bit */
typedef	signed int 				s32;	/**< used for signed 32bit */
typedef	signed long long int 	s64;	/**< used for signed 64bit */

/*unsigned integer types*/
typedef	unsigned char 			u8;		/**< used for unsigned 8bit */
typedef	unsigned short int 		u16;	/**< used for unsigned 16bit */
typedef	unsigned int 			u32;	/**< used for unsigned 32bit */
typedef	unsigned long long int 	u64;	/**< used for unsigned 64bit */

#endif /*#if defined(__STDC_VERSION__)*/

#endif/*#ifndef __ADXL345_TYPES_H__*/
