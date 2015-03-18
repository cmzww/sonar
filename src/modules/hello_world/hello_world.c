#include <assert.h>
#include <math.h>
//#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <nuttx/clock.h>
#include <sys/types.h>
#include <stdint.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/gnssins_report.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>
#include <errno.h>

#include <sys/time.h>
#include <nuttx/config.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/scheduling_priorities.h>
#include <termios.h>
//#include <getopt.h>
#include <poll.h>
__EXPORT int hello_world_main(int agrc, char *argv[]);

void hello_task_main()
{
	uint64_t last_time = 0;
	uint64_t test_time_start = 0;
	uint32_t delt_time = 0;
//	uint64_t count = 0;
	struct GNSSINSReportS raw;
	int gnss_report_fd = orb_subscribe(ORB_ID(gnssins_report));
//	orb_set_interval(gnss_report_fd, 1000);

	struct pollfd fds;
	fds.fd = gnss_report_fd;
	fds.events = POLLIN;
	while(true)
	{

		int ret = poll(&fds,1,150);
		orb_copy(ORB_ID(gnssins_report), gnss_report_fd, &raw);
		last_time = test_time_start;
        test_time_start =  hrt_absolute_time();
		if(ret < 0)
		{
			printf("@@@@@@@@@@@@@@@poll error!!!! \n");
		}
		else if(ret == 0){
			printf("$$$$$$$$$$$$$$$$$poll time out!!!!!\n");
		}else{
//			printf("the pitch is :%6.3f\n",(double)raw.pitch);
//			printf("the roll is :%6.3f\n",(double)raw.roll);
//			printf("the yaw is :%6.3f\n",(double)raw.yaw);
//			printf("the stamp time is : %d\n",(double)raw.TimeStamp);
//			printf("the error count is : %u\n",raw.mti_error_count1);
//			printf("the accelx is :%6.3f\n",(double)raw.LinearAccelerationX);
//			printf("the accely is :%6.3f\n",(double)raw.LinearAccelerationY);
//			printf("the accelz is :%6.3f\n",(double)raw.LinearAccelerationZ);
//			printf("the gryx is :%6.3f\n",(double)raw.AngularVelocityX);
//			printf("the gryy is :%6.3f\n",(double)raw.AngularVelocityY);
//			printf("the gryz is :%6.3f\n",(double)raw.AngularVelocityZ);
//			printf("the VELX is :%6.3f\n",(double)raw.LinearVelocityN);
//			printf("the VELY is :%6.3f\n",(double)raw.LinearVelocityE);
//			printf("the VELZ is :%6.3f\n",(double)raw.LinearVelocityD);
//			printf("the lat is :%6.3f\n",(double)raw.Latitude);
//			printf("the lon is :%6.3f\n",(double)raw.Longitude);
//			printf("the alt is :%6.3f\n",(double)raw.Altitude);
//			printf("the status is : %0x\n",raw.Status);
//			printf("the report status is : %d\n",raw.GNSSINSReportStatus);
			delt_time = test_time_start - last_time;
			if((delt_time > 2700) || (delt_time < 2300))
			{
				printf(" %%%%%%%%%%%%%%%%%%%%: %u\n",(unsigned)(delt_time));
			}

		}
	}
}
int hello_world_main(int argc , char *argv[])
{
	if (!strcmp(argv[1], "start")) {
		/* start the GPS driver worker task */// 任务优先级是否需要修改？
		volatile int _task = task_spawn_cmd("hello_world", SCHED_DEFAULT,
					180, 1500, (main_t)&hello_task_main, 0);

		if (_task < 0) {
			warnx("task start failed: %d", errno);
			return -errno;
		}else{
			printf("task start success!!!\n");
		}
	}
}
