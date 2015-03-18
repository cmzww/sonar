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
 * @file MTI.cpp
 *
 * U-Blox protocol implementation. Following u-blox 6/7/8 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @author Hannes Delago
 *   (rework, add MTI7+ compatibility)
 *
 * @see http://www.u-blox.com/images/downloads/Product_Docs/u-blox6_ReceiverDescriptionProtocolSpec_%28GPS.G6-SW-10018%29.pdf
 * @see http://www.u-blox.com/images/downloads/Product_Docs/u-bloxM8-V15_ReceiverDescriptionProtocolSpec_Public_%28MTI-13003221%29.pdf
 */

#include <assert.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <systemlib/err.h>
#include <uORB/uORB.h>
#include <uORB/topics/gnssins_report.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>
#include <errno.h>
#include "mti.h"

#include <sys/time.h>

#define MTI_CONFIG_TIMEOUT	300		// ms, timeout for waiting ACK
#define MTI_PACKET_TIMEOUT	1		// ms, if now data during this delay assume that full update received
#define MTI_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define DISABLE_MSG_INTERVAL	1000000		// us, try to disable message with this interval

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

#define FNV1_32_INIT	((uint32_t)0x811c9dc5)	// init value for FNV1 hash algorithm
#define FNV1_32_PRIME	((uint32_t)0x01000193)	// magic prime for FNV1 hash algorithm


/**** Trace macros, disable for production builds */
#define MTI_TRACE_PARSER(s, ...)	{/*printf(s, ## __VA_ARGS__);*/}	/* decoding progress in parse_char() */
#define MTI_TRACE_RXMSG(s, ...)		{/*printf(s, ## __VA_ARGS__);*/}	/* Rx msgs in payload_rx_done() */
#define MTI_TRACE_SVINFO(s, ...)	{/*printf(s, ## __VA_ARGS__);*/}	/* NAV-SVINFO processing (debug use only, will cause rx buffer overflows) */

/**** Warning macros, disable to save memory */
#define MTI_WARN(s, ...)		{warnx(s, ## __VA_ARGS__);}


MTI::MTI(const int &fd, struct GNSSINSReportS *mti_position, struct satellite_info_s *satellite_info) :
	_fd(fd),
	_mti_position(mti_position),
	_configured(false),
	_ack_state(MTI_ACK_IDLE)// 需要宏定义后续参数。
//	mti_error_count(0)
//	error_count(0),
{
	_rx_payload_data2_id = 0x0000;
	decode_init();
}

MTI::~MTI()
{
}

int
MTI::configure(unsigned &baudrate)
{
	_configured = false;
	/* try different baudrates */
	const unsigned baudrates[] = {9600, 38400, 19200, 57600, 115200,921600}; //拟采用921600

	//uint8_t cmz[] = "cmz_test";
//	unsigned baud_i ;
	unsigned baud_i = 5;
	set_baudrate(_fd, baudrates[baud_i]);
//	printf("set baudrate %d \n",baudrates[baud_i]);
	_configured = true;
	return 0;
}


int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
MTI::receive(const unsigned timeout)
{
//	printf("receive!\n");
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	ssize_t count = 0; // the count of read data
	uint64_t time_started =  hrt_absolute_time();
	rx_status ret_status;
	int handled = MTDATA_CONTINUE;
//	static uint64_t last_count1 = 0;
	while (true) {
//		int ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;
		/* poll for new data, wait for only MTI_PACKET_TIMEOUT (2ms) if something already received */
		int ret = poll(fds, 1, timeout); // 3ms polldata
//		int ret = 1;
		if (ret < 0){
			/* something went wrong when polling */
			MTI_WARN("MTI poll() err %d\n",fds[0].fd);
//			switch(errno)
//			{
//			case EBADF : //- An invalid file descriptor was given in one of the sets.
//				printf("1");
//				break;
//			case EFAULT ://- The fds address is invalid
//				printf("2");
//				break;
//			case EINTR ://- A signal occurred before any requested event.
//				printf("3");
//				break;
//			case EINVAL:// - The nfds value exceeds a system limit.
//				printf("4");
//				break;
//			case ENOMEM ://- There was no space to allocate internal data structures.
//				printf("5");
//				break;
//			case ENOSYS :
//				printf("6");
//				break;
//			default:
//				printf("7");
//				break;
//			}
			ret_status = RX_POLL_ERR;
//			printf("???POLL_ERR???\n");
			break;
		}else if (ret == 0) {
//			mti_error_count++;
//			printf("@@@POLL_TIMEOUT@@@\n");
			ret_status = RX_POLL_TIMEOUT;
			break;
		}else if (ret > 0) {
			/* if we have new data from mti, go handle it */
			if (fds[0].revents & POLLIN)
			{
				/*
				 * We are here because poll says there is some data, so this
				 * won't block even on a blocking device. But don't read immediately
				 * by 1-2 bytes, wait for some more data to save expensive read() calls.
				 * If more bytes are available, we'll go back to poll() again.
				 */
				count = read(_fd, buf, sizeof(buf));
				/* pass received bytes to the packet decoder */
				for (int i = 0; i < count; i++) {
					handled |= parse_char(buf[i]);
				}
				if(MTDATA_ALL_COMP == handled){
//					uint64_t test_time_start =  hrt_absolute_time();
//					if(((unsigned)(test_time_start-last_count1)>2700 )||((unsigned)(test_time_start-last_count1)<2000))
//					printf("*: %u\n", (unsigned)(test_time_start-last_count1));
//					last_count1 = test_time_start;
					ret_status = RX_RIGHT;
					break;
				}
			}else{
					ret_status = RX_ERROR;
//					printf("??RX_POLL_TIMEOUT??\n");
					break;
			}
		}// end ret > 0
		/* abort after timeout if no useful packets received */
		if (time_started + 4 * 1000 < hrt_absolute_time()) {
//			printf("!!!poll timeout!!!!\n");
			ret_status = RX_DATA_TIMEOUT;
			break;
		}
	} // end while(true)
	return ret_status;
}

//////////////////////////////////////////////////////////////////////////
//function: parse_char(const uint8_t b),used to decode the mti data
//input : raw data -->uint8: 0 = decoding, 3 = message handled,
//output: ret --> the function state
//author:
//////////////////////////////////////////////////////////////////////////
int	//
MTI::parse_char(const uint8_t b)
{
//	int ret2 = MTDATA_CONTINUE;
	int ret = MTDATA_CONTINUE;
	switch (_decode_state) {
	/* Expecting Sync1 -->HEADER PREAMBLE = 0XFF*/
	case MTI_DECODE_SYNC1:
		if (b == MTI_PREAMBLE) {	// Sync1 found --> header
			MTI_TRACE_PARSER("\nA");
			_decode_state = MTI_DECODE_SYNC2; // 下一步获取BID
		}
		break;
	/* Expecting Sync2  MTI_BID = 0XFA*/
	case MTI_DECODE_SYNC2:
		if (b == MTI_BID) // Sync2 found --> expecting Class
		{
			MTI_TRACE_PARSER("B");
			//计算SUMcheck
			add_byte_to_checksum(b);
			// 下一步获取MID
			_decode_state = MTI_DECODE_MID;
		} else {
			// Sync1 not followed by Sync2: reset parser
			decode_init(); //init _decode_state/init _rx_ck/init rx_payload/init rx_payload_index
	//		mti_error_count++;
		}
		break;
	/* Expecting Class */
	case MTI_DECODE_MID:
		if(MTI_ID_MT_DATA2 == b)
			{
			MTI_TRACE_PARSER("C2");
			add_byte_to_checksum(b);
			_decode_state = MTI_DECODE_DATA2_LENGTH;// 下一步获取长度
			break;
		}else
		if(MTI_ID_MT_DATA == b){
			MTI_TRACE_PARSER("C1");
			add_byte_to_checksum(b);
			_decode_state = MTI_DECODE_DATA_LENGTH;// 下一步获取长度
			break;
		}else{
			decode_init();
	//		mti_error_count++;
			break;
		}
		break;
//------------ MTDATA---------------//
	/* Expecting MTDATA length byte */
	case MTI_DECODE_DATA_LENGTH:
		MTI_TRACE_PARSER("E1");
		add_byte_to_checksum(b); //SUMCHECK
		_rx_payload_data_length = b;
				//开始准备接收数据
		_decode_state = MTI_DECODE_DATA_PAYLOAD;
		if (payload_rx_data_init() != 0)// start payload reception
		{
			// payload will not be handled, discard message
			decode_init();
		} else {
			_decode_state = (_rx_payload_data_length > 0) ? MTI_DECODE_DATA_PAYLOAD : MTI_DECODE_DATA_CHKSUM;
		}
		break;
	/* Expecting MTDATA payload */
	case MTI_DECODE_DATA_PAYLOAD:
		add_byte_to_checksum(b);
		ret = payload_rx_data_add(b);
		if (MTDATA_ERROR == ret){
	   // payload not handled, discard message
			decode_init();
			ret = MTDATA_CONTINUE;
		} else if ( MTDATA_COMP == ret){
	   // payload complete, expecting checksum
			_decode_state = MTI_DECODE_DATA_CHKSUM;
			ret = MTDATA_CONTINUE;
		} else{
	  // expecting more payload, stay in state MTI_DECODE_PAYLOAD
			ret = MTDATA_CONTINUE;
		}
		break;
//----------- MTDATA2-------------//
	/* Expecting ALL PACKET LEGNTH */
	case MTI_DECODE_DATA2_LENGTH:
		MTI_TRACE_PARSER("F");
		add_byte_to_checksum(b);
		_rx_payload_data2_length = b ;	// calculate payload size
		_decode_state = MTI_DECODE_DATA2_ID;
		break;
	/* Expecting DATA2 dataID HIGH BYTE */
	case MTI_DECODE_DATA2_ID:
		MTI_TRACE_PARSER("F");
		_rx_payload_data2_index++;
		add_byte_to_checksum(b);
		//big endian
		_rx_payload_data2_id |=((b << 8)&0xff00) ;
		_decode_state = MTI_DECODE_DATA2_ID1;
		break;
	/* Expecting DATA2 dataID LOW BYTE */
	case MTI_DECODE_DATA2_ID1:
		_rx_payload_data2_index++;
		MTI_TRACE_PARSER("F");
		add_byte_to_checksum(b);
		//big endian
		_rx_payload_data2_id |= (b & 0x00ff);
		_decode_state = MTI_DECODE_DATA2_LENGTH1;
		break;
	case MTI_DECODE_DATA2_LENGTH1:
		MTI_TRACE_PARSER("F");
		_rx_payload_data2_index++;
		add_byte_to_checksum(b);
		_rx_payload_data2_length1 = b;
		_decode_state = MTI_DECODE_DATA2_PAYLOAD;
		//init payload
		if (payload_rx_data2_init() != MTDATA_CONTINUE) {	// start payload reception
		// payload will not be handled, discard message
			decode_init();
	//		mti_error_count++;
		} else {
			_decode_state = (_rx_payload_data2_length1 > 0) ? MTI_DECODE_DATA2_PAYLOAD : MTI_DECODE_DATA2_CHKSUM;
		}
		break;
	case MTI_DECODE_DATA2_PAYLOAD:
		MTI_TRACE_PARSER(".");
		add_byte_to_checksum(b);
		switch (_rx_payload_data2_id & 0xff00) {
		case XDI_TimestampGroup:// add timestamp
			ret = payload_rx_add_timestamp(b);
			break;
		case XDI_OrientationGroup:// add orientation
			ret = payload_rx_add_orientation(b);	// add a MON-VER payload byte
			break;
		case XDI_AccelerationGroup:// add acceleration
			ret = payload_rx_add_acceleration(b);
			break;
		case XDI_PositionGroup:// add position
			ret = payload_rx_add_position(b);
			break;
		case XDI_AngularVelocityGroup:// add Angular Velocity
			ret = payload_rx_add_angular_vel(b);
			break;
		case XDI_VelocityGroup: // add velocity (x,y,z)
			ret = payload_rx_add_velocity(b);
			break;
		case XDI_StatusGroup: // add status information
			ret = payload_rx_add_status(b);
			break;
		default:
//			ret = payload_rx_data2_add(b);		// add a payload byte
			break;
		}
		if (MTDATA_ERROR == ret) {
//			mti_error_count++;
			ret = MTDATA_CONTINUE;
			// payload not handled, discard message
			decode_init();
		} else if (MTDATA_DATA_COMP ==  ret) {
			// payload complete, expecting checksum
			_rx_payload_data2_id = 0x0000;
			ret = MTDATA_CONTINUE;
			_decode_state = MTI_DECODE_DATA2_ID;// next packet ID
		} else if(MTDATA_COMP ==  ret){
		//when ret ==0 ; all packet is recieved.
			ret = MTDATA_CONTINUE;
			_decode_state = MTI_DECODE_DATA2_CHKSUM;
			// expecting more payload, stay in state MTI_DECODE_PAYLOAD
		}
		break;
		/* Expecting first checksum byte */
	case MTI_DECODE_DATA2_CHKSUM:
		if ((_rx_ck_a += b)!= 0X00) {
			printf("MTI checksum err");

			ret = MTDATA_CONTINUE ;
//			mti_error_count++;
			decode_init();
		} else
		{
			ret = payload_rx_data2_done();	// finish payload processing
			decode_init();
		}
		break;
	/* Expecting first checksum byte */
	case MTI_DECODE_DATA_CHKSUM:
		if ((_rx_ck_a += b)!= 0x00) {
			printf("MTI checksum err");
			decode_init();
		} else
		{
//			printf("MTI checksum sucess!!!");
			ret = payload_rx_data_done();	// finish payload processing
			ret = MTDATA_ALL_COMP;
			decode_init();
		}
		break;
	default:
		printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
		ret = MTDATA_CONTINUE ;
//		mti_error_count++;
		decode_init();
		break;
	}
	return ret;
}
//-----------add mti function for init payload of MTdata/MTdata2
/**
 * Start payload rx of mtdata
 */
int MTI::payload_rx_data_init(void)
{
	int ret = MTDATA_CONTINUE; // in default, did not check the length;
	if(RX_DATA_LENGTH != _rx_payload_data_length)
	{
		ret = MTDATA_ERROR;
	}
	return ret ;
}
/**
 * Start payload rx of mtdata2; LENGTH CHECK
 */
int MTI::payload_rx_data2_init()
{
	int ret = MTDATA_CONTINUE;
	_rx_state = MTI_DATAID_HANDLE;
	switch(_rx_payload_data2_id & 0xff00){
	case XDI_TemperatureGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_temp))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_TimestampGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_timestamp))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_OrientationGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_euler_angles))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_PressureGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_pressure))
		{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_AccelerationGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_faccel))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_PositionGroup:
		switch(_rx_payload_data2_id & 0xfff0)
		{
			case XDI_AltitudeEllipsoid:
			if(_rx_payload_data2_length1 != sizeof(mti_altmsl))
				{
				_rx_state = MTI_DATAID_ERROR_LENGTH;
			}
			break;
			case XDI_LatLon:
			if(_rx_payload_data2_length1 != 12)//sizeof(mti_latlon))
				{
				_rx_state = MTI_DATAID_ERROR_LENGTH;
			}
			break;
			default:
				_rx_state = MTI_DATAID_DISABLE;
				break;
		}
		break;
	case XDI_AngularVelocityGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_rate_turn))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_VelocityGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_velxyz))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	case XDI_StatusGroup:
		if(_rx_payload_data2_length1 != sizeof(mti_status))
			{
			_rx_state = MTI_DATAID_ERROR_LENGTH;
		}
		break;
	default:
		_rx_state = MTI_DATAID_DISABLE;
		break;
	}
	switch(_rx_state){
	case MTI_DATAID_HANDLE:
	case MTI_DATAID_IGNORE:
		ret = MTDATA_CONTINUE;
		break;
	case MTI_DATAID_DISABLE:
		printf("MTI msg MTI_DATAID_DISABLe,********************%\n");
		ret = MTDATA_ERROR;
		break;
	case MTI_DATAID_ERROR_LENGTH:
		printf("MTI msg 0x%04x invalid len %u", ((unsigned)_rx_payload_data2_id), (unsigned)_rx_payload_data2_length1);
		ret = MTDATA_ERROR;
		break;
	default:
		printf("MTI internal err1");
		ret = MTDATA_ERROR;
		break;
	}
	return ret;
}

/**
 * Add payload rx byte of MTDATA
 */
int	// -1 = error, 0 = ok, 1 = payload completed
MTI:: payload_rx_data_add(const uint8_t b)
{
	int ret = MTDATA_CONTINUE;
	_buf.raw[_rx_payload_data_index++] = b;
	if (_rx_payload_data_index == _rx_payload_data_length) {
		ret = MTDATA_COMP;	// payload received completely
	}
	return ret;
}
/**
 * Add payload rx byte of MTDATA2
 */
//int	// -1 = error, 0 = ok, 1 = payload completed
//MTI::payload_rx_data2_add(const uint8_t b)
//{
//	int ret = MTDATA_CONTINUE;
//	_buf.raw[_rx_payload_data2_index] = b;
//	if (++_rx_payload_data2_index >= _rx_payload_data2_length) {
//		ret = MTDATA_COMP;	// payload received completely
//	}
//	return ret;
//}
//--------------------------------------------------------//
//function:payload_rx_add_timestamp: reciever data and decode ,4byte-->ns,
// 2byte -->year, month/day /hour/minute/second/flage:1byte
//input : uint8 data
//output: packet status:ret = -1:error;ret = 0:continue;ret =1:THIS PACKET COMPLETE
//                        ret = 2: all packet complete
//author:cmz
//-------------------------------------------------------//
int MTI::payload_rx_add_timestamp(const uint8_t b)
{

	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++; // all packet index;
	if(XDI_UtcTime == (_rx_payload_data2_id&0xfff0)) {
		if((_rx_payload_data2_index1) < _rx_payload_data2_length1)
		{
			test_utc_time.raw[_rx_payload_data2_index1++] = b;
			//decode
//			switch(_rx_payload_data2_index1)
//			{
//			case NS_POS:
//				EXCHANG(test_utc_time.raw[0],test_utc_time.raw[3]);
//				EXCHANG(test_utc_time.raw[1],test_utc_time.raw[2]);
//				break;
//			case YEAR_POS:
//				EXCHANG(test_utc_time.raw[YEAR_POS-2],test_utc_time.raw[YEAR_POS-1]);
//				break;
//			default:
//				break;
//			}
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = MTDATA_DATA_COMP;
			_rx_payload_data2_id = 0;
			_rx_payload_data2_index1 = 0; // packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
		{
			_rx_payload_data2_index = 0;
			ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = MTDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_UtcTime". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = MTDATA_ERROR;
//		error_count++;
		return ret;
	}

}
//------------------------------------//
//function : add data to orientation
//input: uint8 data
// output : packet status
//author:cmz
//------------------------------------//
int MTI::payload_rx_add_orientation(const uint8_t b)
{
	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(XDI_EulerAngles == (_rx_payload_data2_id & 0xfff0))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_ela.raw[_rx_payload_data2_index1++] = b;
			// decode
//			switch(_rx_payload_data2_index1){
//				case ROLL_POS:
//					EXCHANG(test_ela.raw[0],test_ela.raw[3]);
//					EXCHANG(test_ela.raw[1],test_ela.raw[2]);
//					 break;
//				case PITCH_POS:
//					EXCHANG(test_ela.raw[7],test_ela.raw[4]);
//					EXCHANG(test_ela.raw[5],test_ela.raw[6]);
//					 break;
//				case YAW_POS:
//					EXCHANG(test_ela.raw[11],test_ela.raw[8]);
//					EXCHANG(test_ela.raw[10],test_ela.raw[9]);
//					 break;
//				default:
//					break;
//			}// end decode
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = MTDATA_DATA_COMP;
//			_rx_payload_data2_id = 0;  // init message id
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = MTDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_EulerAngles". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = MTDATA_ERROR;
//		error_count++;
		return ret;
	}
} // end add orientation
//
int MTI::payload_rx_add_acceleration(const uint8_t b)
{

	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(XDI_FreeAcceleration== (_rx_payload_data2_id & 0xfff0))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_faccel.raw[_rx_payload_data2_index1++] = b;
			// decode
//			switch(_rx_payload_data2_index1){
//			case FACCELX_POS:
//					EXCHANG(test_faccel.raw[0],test_faccel.raw[3]);
//					EXCHANG(test_faccel.raw[1],test_faccel.raw[2]);
//					break;
//				case FACCELY_POS:
//					EXCHANG(test_faccel.raw[4],test_faccel.raw[7]);
//					EXCHANG(test_faccel.raw[5],test_faccel.raw[6]);
//					break;
//				case FACCELZ_POS:
//					EXCHANG(test_faccel.raw[8],test_faccel.raw[11]);
//					EXCHANG(test_faccel.raw[9],test_faccel.raw[10]);
//					break;
//				default:
//					break;
//			}// end decode
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = MTDATA_DATA_COMP;
//			_rx_payload_data2_id = 0;
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = MTDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_FreeAcceleration". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = MTDATA_ERROR;
//		error_count++;
		return ret;
	}
}// end add acceleration
//
int MTI::payload_rx_add_position( const uint8_t b)
{
	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++;
	if(XDI_AltitudeMsl == (_rx_payload_data2_id&0xfff0)) // decode mean sea level altitude
	{
		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_altmsl.raw[_rx_payload_data2_index1++] = b;
			if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
//				EXCHANG(test_altmsl.raw[0],test_altmsl.raw[3]);
//				EXCHANG(test_altmsl.raw[1],test_altmsl.raw[2]);
//				_rx_payload_data2_id = 0;
				_rx_payload_data2_index1 = 0;
				ret = MTDATA_DATA_COMP;
			}
			if(_rx_payload_data2_index == _rx_payload_data2_length){
				_rx_payload_data2_index = 0;
				ret = MTDATA_COMP;
			}
			if(_rx_payload_data2_index < _rx_payload_data2_index1){
				ret = MTDATA_ERROR;
//				error_count++;
			}
		}
		return ret;
	}else if(XDI_LatLon == (_rx_payload_data2_id&0xfff0)) // decode mean sea longtitude latitude
	{
		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_lonlat.raw[_rx_payload_data2_index1++] = b;
//			switch(_rx_payload_data2_index1){
//			case LAT_POS:
//				EXCHANG( test_lonlat.raw[0],test_lonlat.raw[3]);
//				EXCHANG( test_lonlat.raw[1],test_lonlat.raw[2]);
//				break;
//			case LON_POS:
//				EXCHANG( test_lonlat.raw[4],test_lonlat.raw[7]);
//				EXCHANG( test_lonlat.raw[5],test_lonlat.raw[6]);
//				break;
//			default:
//				break;
//			}
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
//			_rx_payload_data2_id = 0;
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
			ret = MTDATA_DATA_COMP;
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length){
			_rx_payload_data2_index = 0;
			ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = MTDATA_ERROR;
//			error_count++;
		}
		return ret;
	}
	else if(XDI_AltitudeEllipsoid ==(_rx_payload_data2_id&0xfff0)){
		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_altmsl.raw[_rx_payload_data2_index1++] = b;
			if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
//				EXCHANG(test_altmsl.raw[0],test_altmsl.raw[3]);
//				EXCHANG(test_altmsl.raw[1],test_altmsl.raw[2]);
//				_rx_payload_data2_id = 0;
				_rx_payload_data2_index1 = 0;
				ret = MTDATA_DATA_COMP;
			}
			if(_rx_payload_data2_index == _rx_payload_data2_length){
				_rx_payload_data2_index = 0;
				ret = MTDATA_COMP;
			}
			if(_rx_payload_data2_index < _rx_payload_data2_index1){
				ret = MTDATA_ERROR;
//				error_count++;
			}
		}
		return ret;
	}
	else{
		// to do somthing when the data type is not "status word". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = MTDATA_ERROR;
		return ret;
	}
}// end add position
//
int MTI::payload_rx_add_angular_vel(const uint8_t b)
{
	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(XDI_RateOfTurn == (_rx_payload_data2_id & 0xfff0))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_rateturn.raw[_rx_payload_data2_index1++] = b;
			// decode
//			switch(_rx_payload_data2_index1){
//				case GRYX_POS:
//					EXCHANG(test_rateturn.raw[0],test_rateturn.raw[3]);
//					EXCHANG(test_rateturn.raw[1],test_rateturn.raw[2]);
//					break;
//				case GRYY_POS:
//					EXCHANG(test_rateturn.raw[4],test_rateturn.raw[7]);
//					EXCHANG(test_rateturn.raw[5],test_rateturn.raw[6]);
//					break;
//				case GRYZ_POS:
//					EXCHANG(test_rateturn.raw[8],test_rateturn.raw[11]);
//					EXCHANG(test_rateturn.raw[9],test_rateturn.raw[10]);
//					break;
//				default:
//					break;
//			}// end decode
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
			ret = MTDATA_DATA_COMP;
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = MTDATA_ERROR;
	//		error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "status word". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = MTDATA_ERROR;
//		error_count++;
		return ret;
	}

}// end  add anguler vel


// payload velocity data : write data to the velocity stucture
int MTI::payload_rx_add_velocity(const uint8_t b)
{
	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(XDI_VelocityXYZ == (_rx_payload_data2_id & 0xfff0))
	{
		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_velxyz.raw[_rx_payload_data2_index1++] = b;
			// decode
//			switch(_rx_payload_data2_index1){
//				case VELX_POS:
//					EXCHANG(test_velxyz.raw[0],test_velxyz.raw[3]);
//					EXCHANG(test_velxyz.raw[1],test_velxyz.raw[2]);
//					break;
//				case VELY_POS:
//					EXCHANG(test_velxyz.raw[4],test_velxyz.raw[7]);
//					EXCHANG(test_velxyz.raw[5],test_velxyz.raw[6]);
//					break;
//				case VELZ_POS:
//					EXCHANG(test_velxyz.raw[8],test_velxyz.raw[11]);
//					EXCHANG(test_velxyz.raw[9],test_velxyz.raw[10]);
//					break;
//				default:
//					break;
//			}// end decode
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
			ret = MTDATA_DATA_COMP;
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = MTDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "status word". we consider as error in this process.
			printf("the ID of velocity messsage is error!! \n");
			ret = MTDATA_ERROR;
//			error_count++;
			return ret;
	}
}// end add velocity xyz
//payload status data : write data to the status stucture
int
MTI::payload_rx_add_status(const uint8_t b)
{
	int ret = MTDATA_CONTINUE;
	_rx_payload_data2_index++;
	if(XDI_StatusWord == (_rx_payload_data2_id&0xfff0) ){
		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			test_status.raw[_rx_payload_data2_index1++] = b;
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
//			mt_data2_status->status_word= (float)(((_buf.raw[0]<<24)&0xff000000)|((_buf.raw[1]<<16)&0x00ff0000)|((_buf.raw[2]<<8)&0x0000ff00)|((_buf.raw[3]<<0)&0x000000ff));
//			EXCHANG(test_status.raw[0],test_status.raw[3]);
//			EXCHANG(test_status.raw[1],test_status.raw[2]);
			_rx_payload_data2_index1 = 0;
			ret = MTDATA_DATA_COMP;
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
				_rx_payload_data2_index = 0;
				ret = MTDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index){
			ret = MTDATA_ERROR;
		}
	}else{
		// to do somthing when the data type is not "status word". we consider as error in this process.
			printf("the ID of velocity messsage is error!! \n");
			ret = MTDATA_ERROR;
			return ret;
	}
	return ret;
}
//-----------------------------------------//
void
MTI::decode_init(void)
{
	_decode_state = MTI_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_payload_data2_id = 0;
	_rx_payload_data_length = 0;
	_rx_payload_data2_length = 0; // ALL PACKET LENGTH
	_rx_payload_data2_length1 = 0; // THE LENGTH OF ONE PACKET
	_rx_payload_data_index = 0;
	_rx_payload_data2_index = 0; //EXTERN INDEX OF data2
	_rx_payload_data2_index1 = 0; // index of dataid IN MTDATA2
}

void
MTI::add_byte_to_checksum(const uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b; //只用计算除preamble以外的所有数据之和
//	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}
int MTI::payload_rx_data2_done(void)
{
	int ret = MTDATA_COMP;
	long long temp2 = 0x0000000000000000;
	uint8_t temp1 = 0x00;
	// return if no message handled
//	if (_rx_state != MTI_DATAID_IGNORE) {
//		return ret;
//	}

//------------------faccel------------------//
//	decode faccelx
	EXCHANG(test_faccel.raw[0],test_faccel.raw[3]);
	EXCHANG(test_faccel.raw[1],test_faccel.raw[2]);
//	decode faccely
	EXCHANG(test_faccel.raw[4],test_faccel.raw[7]);
	EXCHANG(test_faccel.raw[5],test_faccel.raw[6]);
//	decode faccelz
	EXCHANG(test_faccel.raw[8],test_faccel.raw[11]);
	EXCHANG(test_faccel.raw[9],test_faccel.raw[10]);
// report
	_mti_position->LinearAccelerationX = test_faccel.faccel_s.faccelX;
	_mti_position->LinearAccelerationY = test_faccel.faccel_s.faccelY;
	_mti_position->LinearAccelerationZ = test_faccel.faccel_s.faccelZ;
//-----------------eueler angle-------------//
//decode pitch
	EXCHANG(test_ela.raw[0],test_ela.raw[3]);
	EXCHANG(test_ela.raw[1],test_ela.raw[2]);
//decode roll
	EXCHANG(test_ela.raw[7],test_ela.raw[4]);
	EXCHANG(test_ela.raw[5],test_ela.raw[6]);
//decode yaw
	EXCHANG(test_ela.raw[11],test_ela.raw[8]);
	EXCHANG(test_ela.raw[10],test_ela.raw[9]);
// report euler angle
	_mti_position->pitch = test_ela.ela_s.pitch;
	_mti_position->yaw = test_ela.ela_s.yaw;
	_mti_position->roll = test_ela.ela_s.roll;

//----------------------angle_vel--------------------//
//decode gryx
	EXCHANG(test_rateturn.raw[0],test_rateturn.raw[3]);
	EXCHANG(test_rateturn.raw[1],test_rateturn.raw[2]);
//decode gryY
	EXCHANG(test_rateturn.raw[4],test_rateturn.raw[7]);
	EXCHANG(test_rateturn.raw[5],test_rateturn.raw[6]);
//decode gryZ
	EXCHANG(test_rateturn.raw[8],test_rateturn.raw[11]);
	EXCHANG(test_rateturn.raw[9],test_rateturn.raw[10]);
//report angle_vel
	_mti_position->AngularVelocityX = test_rateturn.rate_turn_s.gryX;
	_mti_position->AngularVelocityY = test_rateturn.rate_turn_s.gryY;
	_mti_position->AngularVelocityZ = test_rateturn.rate_turn_s.gryZ;
//---------------------_Altitude----------------------//
	EXCHANG(test_altmsl.raw[0],test_altmsl.raw[3]);
	EXCHANG(test_altmsl.raw[1],test_altmsl.raw[2]);
	_mti_position->Altitude = test_altmsl.altmsl_s.altmsl ;
//-----------------------latlon-----------------------//
////decode lat
//	EXCHANG( test_lonlat.raw[0],test_lonlat.raw[3]);
//	EXCHANG( test_lonlat.raw[1],test_lonlat.raw[2]);
////decode lon
//	EXCHANG( test_lonlat.raw[4],test_lonlat.raw[7]);
//	EXCHANG( test_lonlat.raw[5],test_lonlat.raw[6]);
//// report lat&&lon
//	_mti_position->Latitude = test_lonlat.latlon_s.lat;
//	_mti_position->Longitude = test_lonlat.latlon_s.lon;
//	double result[2] = {0.0,0.0};
//	for(int i = 0;i<2;i++)
//	{
	temp1 = (test_lonlat.raw[4])&0x80;
	switch(temp1){
	case 0x80:
		temp2 = (0xffff000000000000)|(((test_lonlat.raw[4])<<40)&0x0000ff0000000000)
				|(((test_lonlat.raw[5])<<32)&0x000000ff00000000)|(((test_lonlat.raw[0])<<24)&0x00000000ff000000)
				|(((test_lonlat.raw[1])<<16)&0x0000000000ff0000)|(((test_lonlat.raw[2])<<8)&0x000000000000ff00)
				|(((test_lonlat.raw[3])<<00)&0x00000000000000ff);
		break;
	case 0x00:
		temp2 = (0x0000000000000000)|(((test_lonlat.raw[4])<<40)&0x0000ff0000000000)
				|(((test_lonlat.raw[5])<<32)&0x000000ff00000000)|(((test_lonlat.raw[0])<<24)&0x00000000ff000000)
				|(((test_lonlat.raw[1])<<16)&0x0000000000ff0000)|(((test_lonlat.raw[2])<<8)&0x000000000000ff00)
				|(((test_lonlat.raw[3])<<00)&0x00000000000000ff);
		break;
	default:
		break;
	}
	_mti_position->Latitude  = (double)temp2/DEVIDE_PARAM;
	temp1 = 0x00;
	temp2 = 0x0000000000000000;
	temp1 = (test_lonlat.raw[4])&0x80;
	switch(temp1){
	case 0x80:
		temp2 = (0xffff000000000000)|(((test_lonlat.raw[10])<<40)&0x0000ff0000000000)
				|(((test_lonlat.raw[11])<<32)&0x000000ff00000000)|(((test_lonlat.raw[6])<<24)&0x00000000ff000000)
				|(((test_lonlat.raw[7])<<16)&0x0000000000ff0000)|(((test_lonlat.raw[8])<<8)&0x000000000000ff00)
				|(((test_lonlat.raw[9])<<00)&0x00000000000000ff);
		break;
	case 0x00:
		temp2 = (0x0000000000000000)|(((test_lonlat.raw[10])<<40)&0x0000ff0000000000)
				|(((test_lonlat.raw[11])<<32)&0x000000ff00000000)|(((test_lonlat.raw[6])<<24)&0x00000000ff000000)
				|(((test_lonlat.raw[7])<<16)&0x0000000000ff0000)|(((test_lonlat.raw[8])<<8)&0x000000000000ff00)
				|(((test_lonlat.raw[9])<<00)&0x00000000000000ff);
		break;
	default:
		break;
	}
	_mti_position->Longitude  = (double)temp2/DEVIDE_PARAM;
//	}
//	_mti_position->Latitude = result[0];
//	_mti_position->Longitude = result[1];
//--------------------LinearVelocity-----------------//
// decode LinearVelocityN
	EXCHANG(test_velxyz.raw[0],test_velxyz.raw[3]);
	EXCHANG(test_velxyz.raw[1],test_velxyz.raw[2]);
//	decode LinearVelocityE
	EXCHANG(test_velxyz.raw[4],test_velxyz.raw[7]);
	EXCHANG(test_velxyz.raw[5],test_velxyz.raw[6]);
//	decode LinearVelocityD
	EXCHANG(test_velxyz.raw[8],test_velxyz.raw[11]);
	EXCHANG(test_velxyz.raw[9],test_velxyz.raw[10]);
//report LinearVelocity
	_mti_position->LinearVelocityD =test_velxyz.velxyz_s.vel_Z;
	_mti_position->LinearVelocityN =test_velxyz.velxyz_s.vel_X;
	_mti_position->LinearVelocityE =test_velxyz.velxyz_s.vel_Y;
//--------------------------STATUS--------------------//
	EXCHANG(test_status.raw[0],test_status.raw[3]);
	EXCHANG(test_status.raw[1],test_status.raw[2]);
	_mti_position->Status = (test_status.status.status_word);
//	_mti_position->mti_error_count1 = mti_error_count;
	// report status
	_mti_position->GNSSINSReportStatus = IsValid; //数据包。
	// utc time
//	_mti_position->utc_time.utc_ns = test_utc_time.mti_timestamp_s.utc_ns;
//	_mti_position->utc_time.utc_year = test_utc_time.mti_timestamp_s.utc_year;
//	_mti_position->utc_time.utc_month = test_utc_time.mti_timestamp_s.utc_month;
//	_mti_position->utc_time.utc_day = test_utc_time.mti_timestamp_s.utc_day;
//	_mti_position->utc_time.utc_hour = test_utc_time.mti_timestamp_s.utc_hour;
//	_mti_position->utc_time.utc_minute = test_utc_time.mti_timestamp_s.utc_minute;
//	_mti_position->utc_time.utc_second = test_utc_time.mti_timestamp_s.utc_second;
//	_mti_position->utc_time.utc_flags = test_utc_time.mti_timestamp_s.utc_flags;
//



	///////////////////  add   in the future/////////////////////////////////
	//timestamp
	_mti_position->TimeStamp = hrt_absolute_time();
//	printf("pitch = %6.5f\n",(double)_mti_position->pitch);
	ret = MTDATA_ALL_COMP;
	return ret;
}// end add mtdata2 done


int MTI::payload_rx_data_done(void)
{
	int ret = MTDATA_ERROR;
	if(_rx_payload_data_index == _rx_payload_data_length)
	{
		_mti_position->Status = (uint8_t)(_buf.raw[(_rx_payload_data_length-1)]) ;
		ret = MTDATA_CONTINUE;
		for(int i = 0 ; i<_rx_payload_data_length/MTDATA_INC; i++){
		// decode
//			uint8_t *temp = &(_mti_position->raw[i*6]);
		// fp16.32--->double
			uint8_t temp1 = (_buf.raw[i*6+4])&0x80;
//			uint64_t *temp2 = new uint64_t(0);
			long long temp2 = 0x0000000000000000;
			switch(temp1){
			case 0x80:
				temp2 = (0xffff000000000000)|(((_buf.raw[i*6+4])<<40)&0x0000ff0000000000)
						|(((_buf.raw[i*6+5])<<32)&0x000000ff00000000)|(((_buf.raw[i*6+0])<<24)&0x00000000ff000000)
						|(((_buf.raw[i*6+1])<<16)&0x0000000000ff0000)|(((_buf.raw[i*6+2])<<8)&0x000000000000ff00)
						|(((_buf.raw[i*6+3])<<00)&0x00000000000000ff);
//				u_decode_data.temp[0] = 0xff;
//				u_decode_data.temp[1] = 0xff;
//				u_decode_data.temp[2] = (_mti_position->raw[i*6+4]);
//				u_decode_data.temp[3] = (_mti_position->raw[i*6+5]);
//				u_decode_data.temp[4] = (_mti_position->raw[i*6+0]);
//				u_decode_data.temp[5] = (_mti_position->raw[i*6+1]);
//				u_decode_data.temp[7] = (_mti_position->raw[i*6+2]);
//				u_decode_data.temp[8] = (_mti_position->raw[i*6+3]);
				break;
			case 0x00:
				temp2 = (0x0000000000000000)|(((_buf.raw[i*6+4])<<40)&0x0000ff0000000000)
						|(((_buf.raw[i*6+5])<<32)&0x000000ff00000000)|(((_buf.raw[i*6+0])<<24)&0x00000000ff000000)
						|(((_buf.raw[i*6+1])<<16)&0x0000000000ff0000)|(((_buf.raw[i*6+2])<<8)&0x000000000000ff00)
						|(((_buf.raw[i*6+3])<<00)&0x00000000000000ff);
//				u_decode_data.temp[0] = 0x00;
//				u_decode_data.temp[1] = 0x00;
//				u_decode_data.temp[2] = (_mti_position->raw[i*6+4]);
//				u_decode_data.temp[3] = (_mti_position->raw[i*6+5]);
//				u_decode_data.temp[4] = (_mti_position->raw[i*6+0]);
//				u_decode_data.temp[5] = (_mti_position->raw[i*6+1]);
//				u_decode_data.temp[7] = (_mti_position->raw[i*6+2]);
//				u_decode_data.temp[8] = (_mti_position->raw[i*6+3]);
				break;
			default:
				break;
			}
			double result = (double)temp2/DEVIDE_PARAM;
			switch(i){
			case MTDATA_AX:
				_mti_position->LinearAccelerationX =(float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_AY:
				_mti_position->LinearAccelerationY =(float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_AZ:
				_mti_position->LinearAccelerationZ =(float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_GX:
				_mti_position->AngularVelocityX = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_GY:
				_mti_position->AngularVelocityY = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_GZ:
				_mti_position->AngularVelocityZ = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_MX:
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_MY:
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_MZ:
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_ROLL:
				_mti_position->roll = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_PITCH:
				_mti_position->pitch = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_YAW:
				_mti_position->yaw = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_LAT:
				_mti_position->Latitude = result*1e7;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_LON:
				_mti_position->Longitude = result*1e7;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_ALT:
				_mti_position->Altitude = result*1e2; //CM-->M
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_VX:
				_mti_position->LinearVelocityN = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_VY:
				_mti_position->LinearVelocityE = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			case MTDATA_VZ:
				_mti_position->LinearVelocityD = (float)result;
				ret = MTDATA_CONTINUE;
				break;
			default:
				ret = MTDATA_ERROR;
				break;
			}
//			delete temp2;
		}
		_mti_position->TimeStamp = hrt_absolute_time();
		ret = MTDATA_ALL_COMP;
	}

	return ret;
}

void
MTI::send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	if (payload != nullptr)
	{
		write(_fd, (const void *)payload, length);
		printf("send message !\n");
	}
}

//uint32_t
//MTI::fnv1_32_str(uint8_t *str, uint32_t hval)
//{
//    uint8_t *s = str;
//
//    /*
//     * FNV-1 hash each octet in the buffer
//     */
//    while (*s) {
//
//	/* multiply by the 32 bit FNV magic prime mod 2^32 */
//#if defined(NO_FNV_GCC_OPTIMIZATION)
//	hval *= FNV1_32_PRIME;
//#else
//	hval += (hval<<1) + (hval<<4) + (hval<<7) + (hval<<8) + (hval<<24);
//#endif
//
//	/* xor the bottom with the current octet */
//	hval ^= (uint32_t)*s++;
//    }
//
//    /* return our new hash value */
//    return hval;
//}

