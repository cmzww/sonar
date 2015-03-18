//----------------------------------------------------//
//本函数为测试函数，主要用于测试如何通过串口读取数据并通过
//uORB机制将收到的数据发送出去。采用中断触发机制实现。
//----------------------------------------------------//
#include <stdio.h>
#include <unistd.h>
#include <nuttx/config.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <getopt.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "board_config.h"

#include "drv_sensor.h"
#include "drv_orb_dev.h"


#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <systemlib/err.h>
#include <drivers/drv_gps.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/satellite_info.h>

#include <board_config.h>

#include "ubx.h"
#include "mtk.h"
#include "ashtech.h"


#define TIMEOUT_5HZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

class test_uart : public device::CDev
{
public:
	test_uart(const char *uart_path, bool fake_gps, bool enable_sat_info);
	virtual ~test_uart();

	virtual int			init();
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);
	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();
private:

	bool				_task_should_exit;				///< flag to make the main worker task exit
	int				_serial_fd;					///< serial interface to GPS
	unsigned			_baudrate;					///< current baudrate
	char				_port[20];					///< device / serial port path



	/**
	 * Try to configure the GPS, handle outgoing communication to the GPS
	 */
	void			 	config();

	/**
	 * Trampoline to the worker task
	 */
	static void			task_main_trampoline(void *arg);


	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void				task_main(void);

	/**
	 * Set the baudrate of the UART to the GPS
	 */
	int				set_baudrate(unsigned baud);

	/**
	 * Send a reset command to the GPS
	 */
	void				cmd_reset();

};




__EXPORT int test_uart_main(int agrc, char *argv[]);


#ifndef TEST_UART_DEFAULT_PORT
#define TEST_UART_PORT "/dev/ttys5" // 定义串口端口
#endif
#define TEST_UART_DEVICE_PATH	"/dev/test_uart" // 定义在伪文件系统中的路径

const unsigned test_baurd[] = {9600, 38400, 19200, 57600, 115200};
//串口配置

const char cmz = 'M' ;
int test_uart_main(int argc , char *argv[])
{
	int ch = getopt(argc, argv, "XR:");
	if(ch ==1)
	{
		printf(" this is a test");
	}
	const char *argv1 = argv[optind];
	if(!(strcmp(argv1,"start")))
	{
		printf(" cmz test uart is starting!! \n");
		write(TEST_UART_DEVICE_PATH,&cmz,sizeof(cmz));

		return OK;
	}
	else
	{
		if(!strcmp(argv1,"stop"))
		{
			printf("cmz test_uart should be stopped!! \n");
			return OK;
		}
		else
		{
			printf("please add command 'start' or 'stop'!! \n");
			return OK;
		}
	}
	return OK;
}
