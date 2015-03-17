/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_mti.h
 *
 * GPS driver interface.
 */

#ifndef _DRV_MTI_H
#define _DRV_MTI_H

#include <stdint.h>
#include <sys/ioctl.h>

#include "board_config.h"

#include "drv_sensor.h"
#include "drv_orb_dev.h"
/*
#ifndef GPS_DEFAULT_UART_PORT
#define GPS_DEFAULT_UART_PORT "/dev/ttyS3"
#endif
*/
#ifndef MTI_DEFAULT_UART_PORT
#define MTI_DEFAULT_UART_PORT "/dev/ttyS6" //MTI µÄÄ¬ÈÏ´®¿ÚºÅ6¡£
#endif

#define MTI_DEVICE_PATH "/dev/mti"

typedef enum {
	MTI_DRIVER_MODE_NONE = 0,
	MTI_DRIVER_MODE_UBX,
	MTI_DRIVER_MODE_MTK,
	MTI_DRIVER_MODE_ASHTECH,
	MTI_DRIVER_MODE_MTI
} mti_driver_mode_t;
/*
 * ObjDev tag for GPS data.
 */
ORB_DECLARE(mti);

/*
 * ioctl() definitions

#define _MTIIOCBASE			(0x2800)            //TODO: arbitrary choice...
#define _GPSIOC(_n)		(_IOC(_GPSIOCBASE, _n))
*/
#endif /* _DRV_GPS_H */
