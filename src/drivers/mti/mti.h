/****************************************************************************
 *
 *   Copyright (c) 2012, 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file mti.h
 *in this file, class MTI is defined. and some configure various is defined
 *by #define and enum.
 *
 */

#ifndef MTI_H_
#define MTI_H_

#include "mti_helper.h"
#include <uORB/topics/gnssins_report.h>
////////////////////////////////////////////////////
#define EXCHANG(X,Y)  X=X^Y;Y=X^Y;X=X^Y
// header value
#define MTI_PREAMBLE 0x0FA
#define MTI_BID 0x0ff
//-------------------data relate message ----------//
#define MTI_ID_REQ_DATA       0x34 // host request device to send MTdata message
#define MTI_ID_MT_DATA        0x32   // message with un-calibrate raw data, calibrate data,orientation data or GPS PVT DATA
#define MTI_ID_MT_DATA2       0x36  // message with one or more output data packets
//------------------------------------------------//
#define RX_DATA_LENGTH 0x49
////////////////////////////////////////////////

/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* Decoder state */
typedef enum {
	MTI_DECODE_SYNC1 = 0,
	MTI_DECODE_SYNC2,
	MTI_DECODE_MID,
	MTI_DECODE_DATA,
	MTI_DECODE_DATA2,
	MTI_DECODE_DATA_LENGTH,
	//MTI_DECODE_DATA_LENGTH1,
	MTI_DECODE_DATA2_LENGTH,//MTDATA2 LENGTH OF ALL PACKET
	MTI_DECODE_DATA2_LENGTH1,//MTDATA2 LENGTH OF EVERY PACKET
	// 加载数据
	MTI_DECODE_DATA_PAYLOAD,
	MTI_DECODE_DATA2_PAYLOAD,
	// MTI_DATA2 -->DATA ID -->2byte
	MTI_DECODE_DATA2_ID,
	MTI_DECODE_DATA2_ID1,
	// detect error by check sum;
	MTI_DECODE_DATA_CHKSUM,
	MTI_DECODE_DATA2_CHKSUM
} mti_decode_state_t;

/* Rx message state */
typedef enum {
	MTI_DATAID_IGNORE = 0,
	MTI_DATAID_HANDLE,
	MTI_DATAID_DISABLE,
	MTI_DATAID_ERROR_LENGTH
} mti_dataid_state_t;


/* ACK state */
typedef enum {
	MTI_ACK_IDLE = 0,
	MTI_ACK_WAITING,
	MTI_ACK_GOT_ACK,
	MTI_ACK_GOT_NAK
} mti_ack_state_t;
/////////////////////////////////////////////////////
// ---------------mti data identifier--------//
enum XsDataIdentifier
{
	XDI_None					= 0x0000,
	XDI_TypeMask				= 0xFE00,
	XDI_FullTypeMask			= 0xFFF0,
	XDI_FullMask				= 0xFFFF,
	XDI_FormatMask				= 0x01FF,
	XDI_DataFormatMask			= 0x000F,

	XDI_SubFormatMask			= 0x0003,	//determines, float, fp12.20, fp16.32, double output... (where applicable)
	XDI_SubFormatFloat			= 0x0000,
	XDI_SubFormatFp1220			= 0x0001,
	XDI_SubFormatFp1632			= 0x0002,
	XDI_SubFormatDouble			= 0x0003,

	XDI_TemperatureGroup		= 0x0800,
	XDI_Temperature				= 0x0810,

	XDI_TimestampGroup			= 0x1000,
	XDI_UtcTime					= 0x1010,
	XDI_PacketCounter			= 0x1020,
	XDI_Itow					= 0x1030,
	XDI_GpsAge					= 0x1040,
	XDI_PressureAge				= 0x1050,
	XDI_SampleTimeFine			= 0x1060,
	XDI_SampleTimeCoarse		= 0x1070,
	XDI_FrameRange				= 0x1080,	// add for MTw (if needed)
	XDI_PacketCounter8			= 0x1090,
	XDI_SampleTime64			= 0x10A0,

	XDI_OrientationGroup		= 0x2000,
	XDI_CoordSysMask			= 0x000C,
	XDI_CoordSysEnu				= 0x0000,
	XDI_CoordSysNed				= 0x0004,
	XDI_CoordSysNwu				= 0x0008,
	XDI_Quaternion				= 0x2010,
	XDI_RotationMatrix			= 0x2020,
	XDI_EulerAngles				= 0x2030,

	XDI_PressureGroup			= 0x3000,
	XDI_BaroPressure			= 0x3010,

	XDI_AccelerationGroup		= 0x4000,
	XDI_DeltaV					= 0x4010,
	XDI_Acceleration			= 0x4020,
	XDI_FreeAcceleration		= 0x4030,
#ifdef NOT_FOR_PUBLIC_RELEASE
	//XDI_TransposedAcceleration	= 0x4040,
#endif //NOT_FOR_PUBLIC_RELEASE

	XDI_PositionGroup			= 0x5000,
	XDI_AltitudeMsl				= 0x5010,
	XDI_AltitudeEllipsoid		= 0x5020,
	XDI_PositionEcef			= 0x5030,
	XDI_LatLon					= 0x5040,

	XDI_AngularVelocityGroup	= 0x8000,
	XDI_RateOfTurn				= 0x8020,
	XDI_DeltaQ					= 0x8030,

	XDI_GpsGroup				= 0x8800,
	XDI_GpsDop					= 0x8830,
	XDI_GpsSol					= 0x8840,
	XDI_GpsTimeUtc				= 0x8880,
	XDI_GpsSvInfo				= 0x88A0,

	XDI_RawSensorGroup			= 0xA000,
	XDI_RawAccGyrMagTemp		= 0xA010,
	XDI_RawGyroTemp				= 0xA020,
	XDI_RawAcc					= 0xA030,
	XDI_RawGyr					= 0xA040,
	XDI_RawMag					= 0xA050,

	XDI_AnalogInGroup			= 0xB000,
	XDI_AnalogIn1				= 0xB010,
	XDI_AnalogIn2				= 0xB020,

	XDI_MagneticGroup			= 0xC000,
	XDI_MagneticField			= 0xC020,

	XDI_VelocityGroup			= 0xD000,
	XDI_VelocityXYZ				= 0xD010,

	XDI_StatusGroup				= 0xE000,
	XDI_StatusByte				= 0xE010,
	XDI_StatusWord				= 0xE020,
	XDI_Rssi					= 0xE040,

	XDI_IndicationGroup	        = 0x4800, // 0100.1000 -> bit reverse = 0001.0010 -> type 18
	XDI_TriggerIn1				= 0x4810,
	XDI_TriggerIn2				= 0x4820,

#ifdef NOT_FOR_PUBLIC_RELEASE
	/*
	XDI_Accuracy				= 0xF000,
	XDI_GpsHAcc					= 0xF010,
	XDI_GpsVAcc					= 0xF020,
	XDI_GpsSAcc					= 0xF030,
	*/
#endif //NOT_FOR_PUBLIC_RELEASE
};

// MTDATA DECODE DEFINE
//NOTE: if we change the output mode, those should be modified.
#define MTDATA_ACCX 0
#define MTDATA_INC 6 // APPLY FP16.32.


/*! @} */
typedef enum XsDataIdentifier XsDataIdentifier;
// MTDATA/MTDATA2 decoded state
#define MTDATA_ERROR -1
#define MTDATA_CONTINUE 0
#define MTDATA_DATA_COMP 3
#define MTDATA_COMP 2
#define MTDATA_ALL_COMP 1
//orientation / euler angles
#define ROLL_POS 4
#define PITCH_POS 8
#define YAW_POS 12
typedef struct{
float roll;
float pitch;
float yaw;
}mti_euler_angles;
typedef union{
	mti_euler_angles ela_s;
	char raw[12];
}u_ela;
// free acceleration
#define FACCELX_POS 4
#define FACCELY_POS 8
#define FACCELZ_POS 12
typedef struct{
float faccelX;
float faccelY;
float faccelZ;
}mti_faccel;
typedef union{
	mti_faccel faccel_s;
	char raw[12];
}u_faccel;
// rate of turn
#define GRYX_POS 4
#define GRYY_POS 8
#define GRYZ_POS 12

typedef struct{
float gryX;
float gryY;
float gryZ;
}mti_rate_turn;
typedef union{
	mti_rate_turn  rate_turn_s;
	char raw[12];
}u_rate_turn;
// status
typedef struct{
uint32_t status_word;
}mti_status;
typedef union{
	mti_status  status;
	char raw[4];
}u_status;

// latlon
#define LAT_POS 8
#define LON_POS 16
typedef struct{
double  lat;
double	lon;
}mti_latlon;
typedef union{
	mti_latlon  latlon_s;
	long long raw[12];
}u_latlon;
// altmsl
typedef struct{
float altmsl;
}mti_altmsl;
typedef union{
	mti_altmsl altmsl_s;
	char raw[4];
}u_altmsl;
// velocity x/y/z
#define VELX_POS 4
#define VELY_POS 8
#define VELZ_POS 12
typedef struct{
float vel_X;
float vel_Y;
float vel_Z;
}mti_velxyz;
typedef union{
	mti_velxyz velxyz_s;
	char raw[12];
}u_velxyz;

// temperature
typedef struct{
	float temperature; //温度，单位：摄氏度
}mti_temp;

// pressure
typedef struct{
	float pressure; //压力，单位：摄氏度
}mti_pressure;

// timstamp

#define NS_POS 4
#define YEAR_POS 6
#define MONTH_POS 7
#define DAY_POS 8
#define HOUR_POS 9
#define MINUUTE_POS 10
#define SECOND_POS 11
#define FLAGE_POS 12
typedef enum{
	MTDATA_AX = 0,
	MTDATA_AY ,
	MTDATA_AZ ,
	MTDATA_GX,
	MTDATA_GY,
	MTDATA_GZ,
	MTDATA_MX,
	MTDATA_MY,
	MTDATA_MZ,
	MTDATA_ROLL,
	MTDATA_PITCH,
	MTDATA_YAW,
	MTDATA_LAT,
	MTDATA_LON,
	MTDATA_ALT,
	MTDATA_VX,
	MTDATA_VY,
	MTDATA_VZ,
	MTDATA_STATUS,
}mtdata_decode_status;
typedef struct{
uint32_t utc_ns;
uint16_t utc_year;
uint8_t  utc_month;
uint8_t	 utc_day;
uint8_t  utc_hour;
uint8_t  utc_minute;
uint8_t  utc_second;
uint8_t  utc_flags;
}mti_timestamp;
typedef union{
	mti_timestamp mti_timestamp_s;
	uint8_t *raw;
}u_utc_time;
#pragma pack(pop)
typedef union{
	GNSSINSReportS temp_gnss;
	uint8_t *raw;
	uint64_t temp;
}temp_buf;
typedef union{

	double data64;
	uint8_t *temp;
}decode_data;
typedef union{
	GNSSINSReportS vehicle_mti_position_s;
	long long raw[RX_DATA_LENGTH];
}mti_data_buf;

typedef enum{
	RX_INIT = 0,
	RX_POLL_ERR,
	RX_POLL_TIMEOUT,
	RX_DATA_TIMEOUT,
	RX_ERROR,
	RX_RIGHT,
}rx_status;
#define DEVIDE_PARAM 0X100000000
//////////////////////////////////////////////////////

class MTI : public mti_Helper
{
public:
	MTI(const int &fd, struct GNSSINSReportS *mti_position, struct satellite_info_s *satellite_info);
	~MTI();
	int			receive(const unsigned timeout);
	int			configure(unsigned &baudrate);
private:
	uint8_t         buf[256];
	/**
	 * Parse the binary MTI packet
	 */
	int			parse_char(const uint8_t b);

//	/**
//	 * Start payload rx
//	 */
//	int			payload_rx_init(void);
///////////////////////////////////////////////////////////
	//init MTDATA PAYLOAD
		int payload_rx_data_init(void);
	//init MTDATA2 PAYLOAD
		int payload_rx_data2_init(void);
	//
		int payload_rx_data_add(const uint8_t b);
	//
		int payload_rx_data2_add(const uint8_t b);
	//
		int payload_rx_add_temperature(const uint8_t b);
	//
		int payload_rx_add_timestamp(const uint8_t b);
	//
		int payload_rx_add_orientation(const uint8_t b);
	//
		int payload_rx_add_pressure(const uint8_t b);
	//
		int payload_rx_add_acceleration(const uint8_t b);
	//
		int payload_rx_add_position( const uint8_t b);
	//
		int payload_rx_add_angular_vel(const uint8_t b);
	//
		int payload_rx_add_gps(const uint8_t b);
	//
		int payload_rx_add_analog(const uint8_t b);
	//
		int payload_rx_add_magnetic(const uint8_t b);
	//
		int payload_rx_add_velocity(const uint8_t b);
	//
		int payload_rx_add_status(const uint8_t b);
	//
		/**
		 * Finish payload rx
		 */
		int			payload_rx_data_done(void);
		int 		payload_rx_data_decode(void);
		int			payload_rx_data2_done(void);
////////////////////////////////////////////////////////////////
//	/**
//	 * Add payload rx byte
//	 */
//	int			payload_rx_add(const uint8_t b);
//	int			payload_rx_add_nav_svinfo(const uint8_t b);
//	int			payload_rx_add_mon_ver(const uint8_t b);

//	/**
//	 * Finish payload rx
//	 */
//	int			payload_rx_done(void);

	/**
	 * Reset the parse state machine for a fresh start
	 */
	void			decode_init(void);

	/**
	 * While parsing add every byte (except the sync bytes) to the checksum
	 */
	void			add_byte_to_checksum(const uint8_t);

	/**
	 * Send a message
	 */
	void			send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length);

	/**
	 * Configure message rate
	 */
	void			configure_message_rate(const uint16_t msg, const uint8_t rate);

	/**
	 * Calculate & add checksum for given buffer
	 */
//	void			calc_checksum(const uint8_t *buffer, const uint16_t length, mti_checksum_t *checksum);

	/**
	 * Wait for message acknowledge
	 */
//	int			wait_for_ack(const uint16_t msg, const unsigned timeout, const bool report);

	/**
	 * Calculate FNV1 hash
	 */
//	uint32_t		fnv1_32_str(uint8_t *str, uint32_t hval);

	int			_fd;
	struct GNSSINSReportS *_mti_position;
//	struct satellite_info_s *_satellite_info;
//	bool			_enable_sat_info;
	bool			_configured;
	mti_ack_state_t		_ack_state;
//	bool			_got_posllh;
//	bool			_got_velned;
	mti_decode_state_t	_decode_state;
	uint16_t		_rx_msg;
	mti_dataid_state_t	_rx_state;
	mtdata_decode_status e_mti_data_decode_status;
////////////////////////////////////////////////////////////////////////
	uint16_t		_rx_payload_data_length; // MTI DATA LENGTH
	uint16_t		_rx_payload_data2_length; // ALL PACKET LEN
	uint16_t		_rx_payload_data2_length1;// DATA PACKET LEN
	uint16_t		_rx_payload_data2_id;
//	uint16_t 		mti_error_count;
//	uint64_t				right_count;
	uint16_t		_rx_payload_data_index;
	uint16_t		_rx_payload_data2_index; // ALL PACKET
	uint16_t		_rx_payload_data2_index1; // PACKET DATA
	//--------用于格式转换----------//
	u_ela		test_ela;
	u_faccel	test_faccel;
	u_latlon	test_lonlat;
	u_altmsl    test_altmsl;
	u_rate_turn test_rateturn;
	u_velxyz	test_velxyz;
	u_status	test_status;
	u_utc_time	test_utc_time;
	//------------------------------//
	mti_temp   *mt_data2_t;
	mti_euler_angles mt_data2_ela;// euler angles
	mti_pressure *mt_data2_pressure;
	mti_altmsl	*mt_data2_alt;   // ALTITUDE
	mti_timestamp mt_data2_utc;  // TIMESTAMP
	mti_faccel  *mt_data2_faccel; // free accelaration
	mti_rate_turn *mt_data2_rateofturn; //rate of turn
	mti_velxyz	*mt_data2_velxyz;   // velocity
	mti_status	*mt_data2_status;  // status word

	////////////////////////////////////////////


//	uint16_t		_rx_payload_length;
//	uint16_t		_rx_payload_index;
	uint8_t			_rx_ck_a;
	decode_data 	u_decode_data;
	mti_data_buf 	_buf;
//	uint8_t			_rx_ck_b;
//	hrt_abstime		_disable_cmd_last;
//	uint16_t		_ack_waiting_msg;
//	//uint8_t         buf[128];
//	mti_buf_t		_buf;
//	uint32_t		_mti_version;
//	bool			_use_nav_pvt;
};

#endif /* MTI_H_ */
