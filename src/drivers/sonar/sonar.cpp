/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gps.cpp
 * Driver for the GPS on a serial port
 */

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
#include <drivers/drv_sonar.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_mti_position.h>
#include <uORB/topics/sonar_report.h>
//#include <uORB/topics/satellite_info.h>
#include <termios.h>
#include <time.h>
#include <board_config.h>
#include "sonar.h"

#define TIMEOUT_10HZ 100
#define TIMEOUT_5KHZ 500
#define RATE_MEASUREMENT_PERIOD 5000000

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

/* class for dynamic allocation of satellite info data */
//class MTI_Sat_Info
//{
//public:
//	struct satellite_info_s 	_data;
//};


class sonar : public device::CDev
{
public:
	sonar(const char *uart_path, bool fake_sonar);//, bool enable_sat_info);
	virtual ~sonar();

	virtual int			init();

	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void				print_info();

	void 				print_error_count();


private:
///< flag to make the main worker task exit
	bool				_task_should_exit;
	///< serial interface to mti
	int				_serial_fd;
//	int				_serial_fd1;
	///< current baudrate
	unsigned			_baudrate;
	///< device / serial port path
	char				_port[20];
	///< worker task
	volatile int			_task;
	///< flag to signal if the mti is ok/
	bool				_healthy;
	//< flag to signal that the baudrate with the mti has changed
	bool				_baudrate_changed;
	///< flag that the GPS mode has changed //可以不用
	bool				_mode_changed;
	bool 				serial_configured;

	///< current mode 在头文件drv_gps中。
	sonar_driver_mode_t		_mode;


   ///< instance of GPS parser // 操作句柄。在MTI_Helper.h 中声明类，
   //在MTI_helper.cpp中定义相关函数
	sonar_Helper			*_Helper;
	///< instance of mti's GPS sat info data object
//	sonar_Sat_Info			*_Sat_Info;
	///< uORB topic for gps position //指要发送的数据包结构体
//	struct vehicle_mti_position_s	_report_mti_pos;
	struct SonarReportS	_report_sonar_pos;
	///< uORB pub for gps position //ORB topic advertiser handle.
	orb_advert_t			_report_sonar_pos_pub;

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
//class mti end

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int sonar_main(int argc, char *argv[]);

namespace
{

sonar	*g_dev;

}


sonar::sonar(const char *uart_path, bool fake_sonar)://, bool enable_sat_info) :
	CDev("sonar", SONAR_DEVICE_PATH),
	_task_should_exit(false),
	_baudrate(115200),
	_healthy(false),
	_mode_changed(false),
	serial_configured(false),
	_mode(SONAR_DRIVER_MODE_SONAR), //初始化MTI模块。现不知有何用
	_Helper(nullptr),
	_report_sonar_pos_pub(-1)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	/* we need this potentially before it could be set in task_main */
	g_dev = this;
	memset(&_report_sonar_pos, 0, sizeof(_report_sonar_pos));

	_debug_enabled = true;
}

sonar::~sonar()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1)
		task_delete(_task);
	g_dev = nullptr;

}

int
sonar::init()
{
	int ret = ERROR;

	/* do regular cdev init */
	if (CDev::init() != OK)
		goto out;
	/* start the GPS driver worker task */// 任务优先级是否需要修改？
	_task = task_spawn_cmd("sonar", SCHED_DEFAULT,
			SCHED_PRIORITY_SLOW_DRIVER, 1500, (main_t)&sonar::task_main_trampoline, nullptr);

	if (_task < 0) {
		warnx("task start failed: %d", errno);
		return -errno;
	}else{
		//printf("task start success!!!\n");
	}

	ret = OK;
out:
	return ret;
}

int
sonar::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	lock();
	int ret = OK;

	switch (cmd) {
	case SENSORIOCRESET:
		cmd_reset();
		break;
	default:
		/* give it to parent if no one wants it */
		ret = CDev::ioctl(filp, cmd, arg);
		break;
	}
	unlock();
	return ret;
}

void
sonar::task_main_trampoline(void *arg)
{
	g_dev->task_main();
}
void
sonar::task_main()
{
	log("sonar starting");
	static uint16_t error_count = 0;
	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR|O_NONBLOCK);
	if (_serial_fd < 0) {
		log("failed to open serial port: %s err: %d", _port, errno);
		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		_exit(1);
	}else
	{
		printf("open serial port success, serial_fd: %d !!!\n",_serial_fd);
//		::write(_serial_fd, "cmz11\n\r", sizeof("cmz11\n\r"));
	}
	uint64_t last_rate_measurement = hrt_absolute_time();
//	unsigned last_rate_count = 0;
	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		last_rate_measurement = hrt_absolute_time();
		if (_Helper != nullptr) {
		delete(_Helper);
		/* set to zero to ensure parser is not used while not instantiated */
		_Helper = nullptr;
		}
		if(_mode == SONAR_DRIVER_MODE_SONAR)
		{
			////初始化MTI，并通过传递_report_mti_pos引用，发布数据，数据类型为vehicle_mti_position_s
			// MTI内部函数应用的mti_position实质就是_report_mti_pos
			_Helper = new SONAR(_serial_fd, &_report_sonar_pos);//,_p_report_sat_info);
		}
		unlock();
		if (_Helper->configure(_baudrate) == 0) {
				unlock();
				// sonar is obviously detected successfully, reset statistics
				int helper_ret;
				while (!_task_should_exit) {
					helper_ret = _Helper->receive(TIMEOUT_10HZ);
						/* opportunistic publishing - else invalid data would end up on the bus */
					if (!(_pub_blocked)) {
						//printf("$$$$$$$$$$$$$$$$$$$$$$$$$");
						switch(helper_ret)
						{
							case SONAR_RIGHT:
							{
								_report_sonar_pos.SonarReportStatus = Valid ;
								last_rate_measurement = hrt_absolute_time();
								_report_sonar_pos.SonarTimeStamp = last_rate_measurement;
								if (_report_sonar_pos_pub > 0) {
									orb_publish(ORB_ID(Sonar_Report), _report_sonar_pos_pub, &_report_sonar_pos);
								} else {
									_report_sonar_pos_pub = orb_advertise(ORB_ID(Sonar_Report), &_report_sonar_pos);
								}
								break;
							}
							case SONAR_POLL_ERR:
							{
								_report_sonar_pos.SonarReportStatus = Poll_Error ;
								last_rate_measurement = hrt_absolute_time();
								_report_sonar_pos.SonarTimeStamp = last_rate_measurement;
								error_count++;
								_report_sonar_pos.sonar_error_count1  = error_count;
								if (_report_sonar_pos_pub > 0) {
									orb_publish(ORB_ID(Sonar_Report), _report_sonar_pos_pub, &_report_sonar_pos);
								} else {
									_report_sonar_pos_pub = orb_advertise(ORB_ID(Sonar_Report), &_report_sonar_pos);
								}
								break;
							}
							case SONAR_POLL_TIMEOUT:
							{
								printf("&&&\n");
								_report_sonar_pos.SonarReportStatus = Poll_TimeOut ;
								last_rate_measurement = hrt_absolute_time();
								_report_sonar_pos.SonarTimeStamp = last_rate_measurement;
								error_count++;
								_report_sonar_pos.sonar_error_count1  = error_count;
								if (_report_sonar_pos_pub > 0) {
									orb_publish(ORB_ID(Sonar_Report), _report_sonar_pos_pub, &_report_sonar_pos);
								} else {
									_report_sonar_pos_pub = orb_advertise(ORB_ID(Sonar_Report), &_report_sonar_pos);
								}
								break;
							}
							case SONAR_DATA_TIMEOUT:
							{
								_report_sonar_pos.SonarReportStatus = Data_TimeOut;
								last_rate_measurement = hrt_absolute_time();
								_report_sonar_pos.SonarTimeStamp = last_rate_measurement;
								error_count++;
								_report_sonar_pos.sonar_error_count1  = error_count;
								if (_report_sonar_pos_pub > 0) {
									orb_publish(ORB_ID(Sonar_Report), _report_sonar_pos_pub, &_report_sonar_pos);
								} else {
									_report_sonar_pos_pub = orb_advertise(ORB_ID(Sonar_Report), &_report_sonar_pos);
								}
								break;
							}
							case SONAR_ERROR:
							{
								_report_sonar_pos.SonarReportStatus = Rx_Error;
								last_rate_measurement = hrt_absolute_time();
								_report_sonar_pos.SonarTimeStamp = last_rate_measurement;
								error_count++;
								_report_sonar_pos.sonar_error_count1  = error_count;
								if (_report_sonar_pos_pub > 0) {
									orb_publish(ORB_ID(Sonar_Report), _report_sonar_pos_pub, &_report_sonar_pos);
								} else {
									_report_sonar_pos_pub = orb_advertise(ORB_ID(Sonar_Report), &_report_sonar_pos);
								}
								break;
							}
							default:
							{
								_report_sonar_pos.SonarReportStatus = Invaild;
								last_rate_measurement = hrt_absolute_time();
								_report_sonar_pos.SonarTimeStamp = last_rate_measurement;
								error_count++;
								_report_sonar_pos.sonar_error_count1  = error_count;
								if (_report_sonar_pos_pub > 0) {
									orb_publish(ORB_ID(Sonar_Report), _report_sonar_pos_pub, &_report_sonar_pos);
								} else {
									_report_sonar_pos_pub = orb_advertise(ORB_ID(Sonar_Report), &_report_sonar_pos);
								}
								break;
							}
						}// end switch(helper_ret)
					} //   end  if (!(_pub_blocked))
				}// end while ()
				lock();
			}//end if (_Helper->configure(_baudrate) == 0)
			lock();
		}// end while (!_task_should_exit)

	warnx("exiting");
	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);
}
void
sonar::cmd_reset()
{
#ifdef GPIO_GPS_NRESET
	warnx("Toggling sonar reset pin");
	stm32_configgpio(GPIO_GPS_NRESET);
	stm32_gpiowrite(GPIO_GPS_NRESET, 0);
	usleep(100);
	stm32_gpiowrite(GPIO_GPS_NRESET, 1);
	warnx("Toggled sonar reset pin");
#endif
}

void
sonar::print_info()
{
	switch (_mode) {
	case SONAR_DRIVER_MODE_UBX:
		warnx("protocol: UBX \n");
		break;
	case SONAR_DRIVER_MODE_MTK:
		warnx("protocol: MTK \n");
		break;
	case SONAR_DRIVER_MODE_SONAR:
		warnx("protocol: SONAR MTDATA \n");
		break;
	case SONAR_DRIVER_MODE_ASHTECH:
		warnx("protocol: ASHTECH \n");
		break;
	default:
		break;
	}

	warnx("port: %s, baudrate: %d, status: %s", _port, _baudrate, (_healthy) ? "OK" : "NOT OK");
	struct SonarReportS raw;
	int sonar_report_fd = orb_subscribe(ORB_ID(Sonar_Report));
	orb_set_interval(sonar_report_fd, 1000);
	orb_copy(ORB_ID(Sonar_Report), sonar_report_fd, &raw);
	printf("the Sonar Right is :0x%x\n",raw.SonarRight);
	printf("the Sonar Left is :0x%x\n",raw.SonarLeft);
	printf("the Sonar Front is :0x%x\n",raw.SonarFront);
	printf("the Sonar Back is :0x%x\n",raw.SonarBack);
	printf("the Sonar Down is :0x%x\n",raw.SonarDown);
	printf("the Sonar error count is :%d\n",raw.sonar_error_count1);
	printf("the report status is : %d\n",raw.SonarReportStatus);
//	printf("the Sonar time stamp is :%f\n",raw.SonarTimeStamp);
	printf("the stamp time is : %d\n",(double)raw.SonarTimeStamp);

}

namespace sonar_namespace
{

sonar	*g_dev;

void	start(const char *uart_path, bool fake_sonar);//, bool enable_sat_info);
void	stop();
void	test();
void	reset();
void	info();
/**
 * Start the driver.
 */
void
start(const char *uart_path, bool fake_sonar)//, bool enable_sat_info)
// uart_path 的默认地址为：GPS_DEFAULT_UART_PORT "/dev/ttyS5"
{
	int fd;

	if (g_dev != nullptr)
		errx(1, "sonar already started!!\n");

	/* create the driver */
	g_dev = new sonar(uart_path, fake_sonar);//, enable_sat_info);

	if (g_dev == nullptr)
		goto fail;

	if (OK != g_dev->init())
		goto fail;

	/* set the poll rate to default, starts automatic data collection */
	fd = open(SONAR_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		errx(1, "Could not open device path: %s\n", SONAR_DEVICE_PATH);
		goto fail;
	}else
	{
		printf("open SONAR device path successes!!\n");
	}

	exit(0);
fail:
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}
	errx(1, "driver start failed!!");
}

/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;
	if(g_dev == nullptr)
	{
		printf("SONAR have stopped!!!\n");
	}else
	{
		printf("SONAR stopped failed!!!\n");
	}
	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */


/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SONAR_DEVICE_PATH, O_RDONLY);

	if (fd < 0)
		err(1, "SONAR open failed ");

	if (ioctl(fd, SENSORIOCRESET, 0) < 0)
		err(1, "driver reset failed");

	exit(0);
}

/**
 * Print the status of the driver.
 */
void test()
{
//	g_dev->test();
}
void info()
{
	if (g_dev == nullptr)
		errx(1, "driver not running");

	g_dev->print_info();

	exit(0);
}

} // namespace


int
sonar_main(int argc, char *argv[])
{

	/* set to default */
	const char *device_name = SONAR_DEFAULT_UART_PORT;
	bool fake_sonar = false;
//	bool enable_sat_info = false;

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		/* work around getopt unreliability */
//		if (argc > 3) {
//			if (!strcmp(argv[2], "-d")) {
//				device_name = argv[3];
//
//			} else {
//				goto out;
//			}
//		}
//
//		/* Detect fake gps option */
//		for (int i = 2; i < argc; i++) {
//			if (!strcmp(argv[i], "-f"))
//				fake_sonar = true;
//		}
//
//		/* Detect sat info option */
//		for (int i = 2; i < argc; i++) {
//			if (!strcmp(argv[i], "-s"))
//				enable_sat_info = true;
//		}

//		printf("the fake_mti = %b;\n device_name =%s;\n enable_st = %d \n",fake_sonar,device_name,enable_sat_info);

		sonar_namespace::start(device_name, fake_sonar);//, enable_sat_info);
	}

	if (!strcmp(argv[1], "stop"))
		sonar_namespace::stop();

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test"))
		sonar_namespace::test();

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset"))
		sonar_namespace::reset();

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info"))
		sonar_namespace::info();

out:
	errx(1, "unrecognized command, try 'start', 'stop', 'test', 'reset' ,'status','info','vel','accel'"
			" 'eula','gry','lon_lat_alt','errorcount' [-d /dev/ttyS0-n][-f][-s]");
}
