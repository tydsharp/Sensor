#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include "raspberry_util.h"
#include "raspberry_debug.h"

/*
 ***************************
 * Define for SPI
 ***************************
*/
#define RASPBERRY_SPI_DEV0 		"/dev/spidev0.0"
#define RASPBERRY_SPI_DEV1 		"/dev/spidev0.1"
#define RASPBERRY_SPI_TURE		((uint8_t)1)
#define RASPBERRY_SPI_FALSE		((uint8_t)0)


typedef struct {
	raspberry_spi_info_t 	spi_info;
	int32_t 					spi_fd;
}raspberry_g_spi_info_data;
static raspberry_g_spi_info_data g_spi_info_data[2] = {0};


int32_t raspberry_spi_open(int32_t spi_channel, raspberry_spi_info_t * spi_info)
{	
	uint8_t spi_mode = 0;
	if(g_spi_info_data[spi_channel].spi_fd == 0)
	{	
		memset( &(g_spi_info_data[spi_channel]), 0, sizeof(raspberry_g_spi_info_data));
		
		/*Spi Dev New Open*/
		if( (g_spi_info_data[spi_channel].spi_fd = open( (spi_channel == RASPBERRY_SPI_CHANNEL_0 ? RASPBERRY_SPI_DEV0 : RASPBERRY_SPI_DEV1),O_RDWR)) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable open Spi Device with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		
		switch(spi_info->timing_mode)
		{
			case RASPBERRY_SPI_MODE_CPHAH_CPOLH:
				spi_mode = SPI_MODE_3;
				break;
			case RASPBERRY_SPI_MODE_CPHAL_CPOLH:
				spi_mode = SPI_MODE_2;
				break;
			case RASPBERRY_SPI_MODE_CPHAH_CPOLL:
				spi_mode = SPI_MODE_1;
				break;
			case RASPBERRY_SPI_MODE_CPHAL_CPOLL:
			default:
				spi_mode = SPI_MODE_0;
		}
		
		if(RASPBERRY_SPI_3W_ENABLE == spi_info->is_3w)
		{
			spi_mode = spi_mode | SPI_3WIRE;
		}
		
		if(RASPBERRY_SPI_CS_ACTIVE_HIGH == spi_info->is_cs_high)
		{
			spi_mode = spi_mode | SPI_CS_HIGH;
		}
		
		/*Set Spi Paramter*/
		if( ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_WR_MODE, &spi_mode) < 0 
			||
		    ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_RD_MODE, &spi_mode) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable Set Spi Mode with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		
		if( ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_WR_BITS_PER_WORD, &(spi_info->bpw)) < 0 
			||
		    ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_RD_BITS_PER_WORD, &(spi_info->bpw)) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable Set Spi bits per word with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		
		if( ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &(spi_info->speed_hz)) < 0 
			||
		    ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &(spi_info->speed_hz)) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable Set Spi speed with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		
#if 0		
		uint32_t spi_3w = 0x00000000 |  SPI_3WIRE;
		if(RASPBERRY_SPI_3W_ENABLE == spi_info->is_3w)
		{
			if( ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_WR_MODE32, &spi_3w) < 0 
				||
			    ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_RD_MODE32, &spi_3w) < 0 )
			{
				RASPBERRY_DEBUG_ERROR("Unable Set Spi to 3W Mode with error : %s\n",strerror(errno));
				return RASPBERRY_ERROR;
			}
		}
#endif
		memcpy( &(g_spi_info_data[spi_channel].spi_info), spi_info, sizeof(raspberry_spi_info_t));
	}
	
	return RASPBERRY_OK;
}

int32_t raspberry_spi_write(int32_t spi_channel, uint8_t *wr_buf, uint16_t len)
{
	struct spi_ioc_transfer spi_data;
	if(g_spi_info_data[spi_channel].spi_fd > 0)
	{
		memset(&spi_data,0,sizeof(spi_data));
		spi_data.tx_buf 	= (unsigned long)wr_buf;
		spi_data.rx_buf 	= (unsigned long)RASPBERRY_NULL;
		spi_data.len 			= (uint32_t)len;
		spi_data.speed_hz 		= (uint32_t)g_spi_info_data[spi_channel].spi_info.speed_hz;
		spi_data.delay_usecs 	= (uint16_t)g_spi_info_data[spi_channel].spi_info.delay;
		spi_data.bits_per_word 	= (uint8_t)g_spi_info_data[spi_channel].spi_info.bpw;
		spi_data.cs_change 		= RASPBERRY_SPI_TURE;
		
		if(ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_MESSAGE(1), &spi_data) < 0)
		{
			RASPBERRY_DEBUG_ERROR("Unable Read and Write Spi with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		return RASPBERRY_OK;
	}
	RASPBERRY_DEBUG_ERROR("Spi has not yet opened!\n");
	return RASPBERRY_ERROR;	
}

int32_t raspberry_spi_read(int32_t spi_channel, uint8_t *rd_buf, uint16_t len)
{
	struct spi_ioc_transfer spi_data;	
	if(g_spi_info_data[spi_channel].spi_fd > 0)
	{
		memset(&spi_data,0,sizeof(spi_data));
		spi_data.tx_buf 	= (unsigned long)RASPBERRY_NULL;
		spi_data.rx_buf 	= (unsigned long)rd_buf;
		spi_data.len 			= (uint32_t)len;
		spi_data.speed_hz 		= (uint32_t)g_spi_info_data[spi_channel].spi_info.speed_hz;
		spi_data.delay_usecs 	= (uint16_t)g_spi_info_data[spi_channel].spi_info.delay;
		spi_data.bits_per_word 	= (uint8_t)g_spi_info_data[spi_channel].spi_info.bpw;
		spi_data.cs_change 		= RASPBERRY_SPI_TURE;
		
		if(ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_MESSAGE(1), &spi_data) < 0)
		{
			RASPBERRY_DEBUG_ERROR("Unable Read and Write Spi with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		return RASPBERRY_OK;
	}
	RASPBERRY_DEBUG_ERROR("Spi has not yet opened!\n");
	return RASPBERRY_ERROR;		
}

int32_t raspberry_spi_write_read(int32_t spi_channel, uint8_t *wr_buf, uint16_t wr_len, uint8_t *rd_buf, uint16_t rd_len)
{
	struct spi_ioc_transfer spi_data[2];
	if(g_spi_info_data[spi_channel].spi_fd > 0)
	{
		memset(spi_data,0,sizeof(struct spi_ioc_transfer)*2);

		spi_data[0].tx_buf 	= (unsigned long)wr_buf;
		spi_data[0].rx_buf = (unsigned long)RASPBERRY_NULL;
		spi_data[0].len 			= (uint32_t)wr_len;
		spi_data[0].speed_hz 		= (uint32_t)g_spi_info_data[spi_channel].spi_info.speed_hz;
		spi_data[0].delay_usecs 	= (uint16_t)g_spi_info_data[spi_channel].spi_info.delay;
		spi_data[0].bits_per_word 	= (uint8_t)g_spi_info_data[spi_channel].spi_info.bpw;
		
		spi_data[1].tx_buf = (unsigned long)RASPBERRY_NULL;
		spi_data[1].rx_buf 	= (unsigned long)rd_buf;
		spi_data[1].len 			= (uint32_t)rd_len;
		spi_data[1].speed_hz 		= (uint32_t)g_spi_info_data[spi_channel].spi_info.speed_hz;
		spi_data[1].delay_usecs 	= (uint16_t)g_spi_info_data[spi_channel].spi_info.delay;
		spi_data[1].bits_per_word 	= (uint8_t)g_spi_info_data[spi_channel].spi_info.bpw;
		spi_data[1].cs_change 		= RASPBERRY_SPI_TURE;

		if(ioctl(g_spi_info_data[spi_channel].spi_fd, SPI_IOC_MESSAGE(2), spi_data) < 0)
		{
			RASPBERRY_DEBUG_ERROR("Unable Read and Write Spi with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		return RASPBERRY_OK;
	}	
	RASPBERRY_DEBUG_ERROR("Spi has not yet opened!\n");
	return RASPBERRY_ERROR;
}

int32_t raspberry_spi_close(int32_t spi_channel)
{
	if(g_spi_info_data[spi_channel].spi_fd > 0)
	{
		close(g_spi_info_data[spi_channel].spi_fd);
		memset(&g_spi_info_data[spi_channel],0,sizeof(raspberry_g_spi_info_data));
		g_spi_info_data[spi_channel].spi_fd = 0;
	}
	return RASPBERRY_OK;
}


/*
 ***************************
 * Define for I2C
 ***************************
*/
#define RASPBERRY_I2C_DEV0		"/dev/i2c-1"
static int32_t g_i2c_fd = 0;

int32_t raspberry_i2c_open(void)
{	
	if(g_i2c_fd == 0)
	{
		/*Open I2C device*/
		if( (g_i2c_fd = open(RASPBERRY_I2C_DEV0,O_RDWR)) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable open I2C Device with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
	}
	return RASPBERRY_OK;
}

int32_t raspberry_i2c_read(uint16_t dev_addr,uint8_t * rd_buf,uint16_t len)
{
	struct i2c_msg i2c_msg_data = {0};
	struct i2c_rdwr_ioctl_data i2c_ioctl_data = {0};

	if(g_i2c_fd > 0)
	{	
		i2c_msg_data.addr = dev_addr;
		i2c_msg_data.flags = I2C_M_RD;
		i2c_msg_data.len = len;
		i2c_msg_data.buf = rd_buf;
		
		i2c_ioctl_data.msgs = &i2c_msg_data;
		i2c_ioctl_data.nmsgs = 1;
		
		if( ioctl(g_i2c_fd,I2C_RDWR,&i2c_ioctl_data) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable read data from I2C device with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		return RASPBERRY_OK;
	}
	RASPBERRY_DEBUG_ERROR("I2C has not yet opened!\n");
	return RASPBERRY_ERROR;
}

int32_t raspberry_i2c_write(uint16_t dev_addr,uint8_t * wr_buf,uint16_t len)
{
	struct i2c_msg i2c_msg_data = {0};
	struct i2c_rdwr_ioctl_data i2c_ioctl_data = {0};

	if(g_i2c_fd > 0)
	{		
		i2c_msg_data.addr = dev_addr;
		i2c_msg_data.len = len;
		i2c_msg_data.buf = wr_buf;
		
		i2c_ioctl_data.msgs = &i2c_msg_data;
		i2c_ioctl_data.nmsgs = 1;
		
		if( ioctl(g_i2c_fd,I2C_RDWR,&i2c_ioctl_data) < 0 )
		{
			RASPBERRY_DEBUG_ERROR("Unable write data from I2C device with error : %s\n",strerror(errno));
			return RASPBERRY_ERROR;
		}
		return RASPBERRY_OK;
	}
	RASPBERRY_DEBUG_ERROR("I2C has not ye opened!\n");
	return RASPBERRY_ERROR;
}

int32_t raspberry_i2c_close(void)
{
	if(g_i2c_fd > 0)
	{
		close(g_i2c_fd);
		g_i2c_fd = 0;
	}
	return RASPBERRY_OK;
}

void raspberry_delay(uint32_t ms)
{
	usleep(ms*1000);
}