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
 * @file SONAR.h
 *in this file, class SONAR is defined. and some configure various is defined
 *by #define and enum.
 *
 */

#ifndef SONAR_H_
#define SONAR_H_

#include "sonar_helper.h"
#include <uORB/topics/sonar_report.h>
////////////////////////////////////////////////////
#define EXCHANG(X,Y)  X=X^Y;Y=X^Y;X=X^Y
// header value
#define SONAR_PREAMBLE 0x0FA
#define SONAR_BID 0x0ff
//-------------------data relate message ----------//
//#define SONAR_ID_REQ_DATA       0x34 // host request device to send MTdata message
#define SONAR_ID_MT_DATA        0x32   // message with un-calibrate raw data, calibrate data,orientation data or GPS PVT DATA
#define SONAR_ID_MT_DATA2       0x34  // message with one or more output data packets
//------------------------------------------------//
#define SONAR_RX_DATA_LENGTH    0x0A  // BASED input type. according our requirement, we should modify its value
////////////////////////////////////////////////
//#define SONARDATA_ERROR -1
//#define SONARDATA_CONTINUE 0
//#define SONARDATA_DATA_COMP 3
//#define SONARDATA_COMP 2
//#define SONARDATA_ALL_COMP 1
/*** u-blox protocol binary message and payload definitions ***/
#pragma pack(push, 1)

/* Decoder state */
typedef enum {
	SONAR_DECODE_SYNC1 = 0,
	SONAR_DECODE_SYNC2,
	SONAR_DECODE_MID,
	SONAR_DECODE_DATA,
	SONAR_DECODE_DATA2,
	SONAR_DECODE_DATA_LENGTH,
	//SONAR_DECODE_DATA_LENGTH1,
	SONAR_DECODE_DATA2_LENGTH,//MTDATA2 LENGTH OF ALL PACKET
	SONAR_DECODE_DATA2_LENGTH1,//MTDATA2 LENGTH OF EVERY PACKET
	// 加载数据
	SONAR_DECODE_DATA_PAYLOAD,
	SONAR_DECODE_DATA2_PAYLOAD,
	// SONAR_DATA2 -->DATA ID -->2byte
	SONAR_DECODE_DATA2_ID,
	SONAR_DECODE_DATA2_ID1,
	// detect error by check sum;
	SONAR_DECODE_DATA_CHKSUM,
	SONAR_DECODE_DATA2_CHKSUM
} sonar_decode_state_t;

/* Rx message state */
typedef enum {
	SONAR_DATAID_IGNORE = 0,
	SONAR_DATAID_HANDLE,
	SONAR_DATAID_DISABLE,
	SONAR_DATAID_ERROR_LENGTH
} sonar_dataid_state_t;

// MTDATA DECODE DEFINE
//NOTE: if we change the output mode, those should be modified.
#define MTDATA_ACCX 0
#define MTDATA_INC 6 // APPLY FP16.32.


/*! @} */
//typedef enum XsDataIdentifier XsDataIdentifier;
// MTDATA/MTDATA2 decoded state
#define SONARDATA_ERROR -1
#define SONARDATA_CONTINUE 0
#define SONARDATA_DATA_COMP 3
#define SONARDATA_COMP 2
#define SONARDATA_ALL_COMP 1
typedef struct{
	uint16_t SonarDownTemp;
	uint16_t SonarFrontTemp;
	uint16_t SonarBackTemp;
	uint16_t SonarLeftTemp;
	uint16_t SonarRightTemp;
}sonar_data_temp;
typedef union{
	sonar_data_temp sonar_data_temp_s;
	uint16_t raw[SONAR_RX_DATA_LENGTH];
}temp_buf;

typedef enum{
	SONAR_INIT = 0,
	SONAR_POLL_ERR,
	SONAR_POLL_TIMEOUT,
	SONAR_DATA_TIMEOUT,
	SONAR_ERROR,
	SONAR_RIGHT
}sonar_rx_status;
#define SONAR_FRONT 0x1000
#define SONAR_BACK 0x2000
#define SONAR_LEFT 0x3000
#define SONAR_RIGHT_T 0x4000
#define SONAR_DOWN 0x5000
#define SONAR_FRONT2 0x0046
#define SONAR_BACK2 0x0042
#define SONAR_LEFT2 0x004C
#define SONAR_RIGHT_T2 0x0052
#define SONAR_DOWN2 0x0044
#define SONAR_DATA_LEGHTH 2
//#define DEVIDE_PARAM 0X100000000 // f32.16 --->double
//////////////////////////////////////////////////////

class SONAR : public sonar_Helper
{
public:
	SONAR(const int &fd, struct SonarReportS *sonar_position);//, struct satellite_info_s *satellite_info);
	~SONAR();
	int			receive(const unsigned timeout);
	int			configure(unsigned &baudrate);
private:
	uint8_t         buf[256];
	/**
	 * Parse the binary SONAR packet
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
		int payload_rx_add_front(const uint8_t b);
		int payload_rx_add_back(const uint8_t b);
		int payload_rx_add_left(const uint8_t b);
		int payload_rx_add_right(const uint8_t b);
		int payload_rx_add_down(const uint8_t b);
		/**
		 * Finish payload rx
		 */
		int			payload_rx_data_done(void);
		int         payload_rx_data2_done();
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

	int			_fd;
	struct SonarReportS *_sonar_position;
	bool			_configured;
	sonar_decode_state_t	_decode_state;
	uint16_t		_rx_msg;
	sonar_dataid_state_t	_rx_state;
////////////////////////////////////////////////////////////////////////
	uint16_t		_rx_payload_data_length; // SONAR DATA LENGTH
	uint16_t		_rx_payload_data2_length; // ALL PACKET LEN
	uint16_t		_rx_payload_data2_length1;// DATA PACKET LEN
	uint16_t		_rx_payload_data2_id;
	uint16_t		_rx_payload_data_index;
	uint16_t		_rx_payload_data2_index; // ALL PACKET
	uint16_t		_rx_payload_data2_index1; // PACKET DATA
	temp_buf		SonarBuf;
	uint16_t        Sonar_Front;
	uint16_t        Sonar_Back;
	uint16_t        Sonar_Left;
	uint16_t        Sonar_Right;
	uint16_t        Sonar_Down;
	uint16_t        Temp_Data[2];
	uint16_t count_right_data;
	//--------用于格式转换----------//
	uint8_t			_rx_ck_a;
};

#endif /* SONAR_H_ */
