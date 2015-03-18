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
 * @file SONAR.cpp
 *
 * U-Blox protocol implementation. Following u-blox 6/7/8 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <joes@student.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 *
 * @author Hannes Delago
 *   (rework, add SONAR7+ compatibility)
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
#include <uORB/topics/sonar_report.h>
#include <uORB/topics/satellite_info.h>
#include <drivers/drv_hrt.h>
#include <errno.h>
#include "sonar.h"

#include <sys/time.h>

#define SONAR_CONFIG_TIMEOUT	300		// ms, timeout for waiting ACK
#define SONAR_PACKET_TIMEOUT	1		// ms, if now data during this delay assume that full update received
#define SONAR_WAIT_BEFORE_READ	20		// ms, wait before reading to save read() calls
#define DISABLE_MSG_INTERVAL	1000000		// us, try to disable message with this interval

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

#define FNV1_32_INIT	((uint32_t)0x811c9dc5)	// init value for FNV1 hash algorithm
#define FNV1_32_PRIME	((uint32_t)0x01000193)	// magic prime for FNV1 hash algorithm


/**** Trace macros, disable for production builds */
#define SONAR_TRACE_PARSER(s, ...)	{/*printf(s, ## __VA_ARGS__);*/}	/* decoding progress in parse_char() */
#define SONAR_TRACE_RXMSG(s, ...)		{/*printf(s, ## __VA_ARGS__);*/}	/* Rx msgs in payload_rx_done() */
#define SONAR_TRACE_SVINFO(s, ...)	{/*printf(s, ## __VA_ARGS__);*/}	/* NAV-SVINFO processing (debug use only, will cause rx buffer overflows) */

/**** Warning macros, disable to save memory */
#define SONAR_WARN(s, ...)		{warnx(s, ## __VA_ARGS__);}


SONAR::SONAR(const int &fd, struct SonarReportS *sonar_position)://, struct satellite_info_s *satellite_info) :
	_fd(fd),
	_sonar_position(sonar_position),
	_configured(false),
	count_right_data(0)
{
	_rx_payload_data2_id = 0x0000;
	decode_init();
}

SONAR::~SONAR()
{
}

int
SONAR::configure(unsigned &baudrate)
{
	_configured = false;
	/* try different baudrates */
	const unsigned baudrates[] = {9600, 38400, 19200, 57600, 115200,921600}; //拟采用921600

//uint8_t cmz[] = "cmz_test";
//	unsigned baud_i ;
	unsigned baud_i = 4;
	set_baudrate(_fd, baudrates[baud_i]);
//	printf("set baudrate %d \n",baudrates[baud_i]);
	_configured = true;
	return 0;
}


int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
SONAR::receive(const unsigned timeout)
{
	/* poll descriptor */
	pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;
	ssize_t count = 0; // the count of read data
//	uint64_t sonar_time_started =  hrt_absolute_time();
	sonar_rx_status ret_status;
	int handled = SONARDATA_CONTINUE;
//	static uint64_t last_count1 = 0;
	while (true) {
//		int ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;
		/* poll for new data, wait for only MTI_PACKET_TIMEOUT (2ms) if something already received */
		int ret = poll(fds, 1, (-1)); // 3ms polldata
		usleep(80*1000);
		if (ret < 0){
			/* something went wrong when polling */
			SONAR_WARN("SONAR poll() err %d\n",fds[0].fd);
			ret_status = SONAR_POLL_ERR;
			printf("???POLL_ERR???\n");
			break;
		}else if (ret == 0) {
			printf("@@@POLL_TIMEOUT@@@\n");
			ret_status = SONAR_POLL_TIMEOUT;
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
				printf(" %%%%%%%% count = %d \n",count);
				/* pass received bytes to the packet decoder */
				for (int i = 0; i < count; i++) {
					handled |= parse_char(buf[i]);
				}
				if(SONARDATA_ALL_COMP == handled){
					ret_status = SONAR_RIGHT;
//					count_right_data++;
//					printf("count = %d \n",count_right_data);
					break;
				}
			}else{
					ret_status = SONAR_ERROR;
//					printf("??RX_POLL_revents != POLLIN??\n");
					break;
			}
		}// end ret > 0
		/* abort after timeout if no useful packets received */
//		if ((sonar_time_started +200*1000) < hrt_absolute_time()) {
//			printf("!!!timeout!!!!\n");
//			sonar_time_started = 0;
//			ret_status = SONAR_DATA_TIMEOUT;
//			break;
//		}
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
SONAR::parse_char(const uint8_t b)
{
	int ret = SONARDATA_CONTINUE;
	switch (_decode_state) {
	/* Expecting Sync1 -->HEADER PREAMBLE = 0XFF*/
	case SONAR_DECODE_SYNC1:
		if (b == SONAR_PREAMBLE) {	// Sync1 found --> header
			SONAR_TRACE_PARSER("\nA");
			_decode_state = SONAR_DECODE_SYNC2; // 下一步获取BID
		}
		break;
	/* Expecting Sync2  SONAR_BID = 0XFA*/
	case SONAR_DECODE_SYNC2:
		if (b == SONAR_BID) // Sync2 found --> expecting Class
		{
			SONAR_TRACE_PARSER("B");
			//计算SUMcheck
			add_byte_to_checksum(b);
//			printf("_rx_ck_a = %x\n",_rx_ck_a);
			// 下一步获取MID
			_decode_state = SONAR_DECODE_MID;
		} else {
			// Sync1 not followed by Sync2: reset parser
			decode_init(); //init _decode_state/init _rx_ck/init rx_payload/init rx_payload_index
		}
		break;
	/* Expecting Class */
	case SONAR_DECODE_MID:
		if(SONAR_ID_MT_DATA == b){
//			SONAR_TRACE_PARSER("C");
			add_byte_to_checksum(b);
//			printf("_rx_ck_a = %x\n",_rx_ck_a);
			_decode_state = SONAR_DECODE_DATA_LENGTH;// 下一步获取长度
			break;
		}else if(SONAR_ID_MT_DATA2 == b){
//			SONAR_TRACE_PARSER("C");
			add_byte_to_checksum(b);
//			printf("_rx_ck_a = %x\n",_rx_ck_a);
			_decode_state = SONAR_DECODE_DATA2_LENGTH;// 下一步获取长度
			break;
		}else{
			decode_init();
			break;
		}
		break;
// ----------------mtdata2------------//
	case SONAR_DECODE_DATA2_LENGTH:
//		printf("11\n");
//		SONAR_TRACE_PARSER("E");
		add_byte_to_checksum(b); //SUMCHECK
//		printf("_rx_ck_a = %x\n",_rx_ck_a);
		_rx_payload_data2_length = b;
				//开始准备接收数据
		_decode_state = SONAR_DECODE_DATA2_ID;
		break;
	case SONAR_DECODE_DATA2_ID:
//		printf("22\n");
//		SONAR_TRACE_PARSER("E");
		add_byte_to_checksum(b); //SUMCHECK
//		printf("_rx_ck_a = %x\n",_rx_ck_a);
		_rx_payload_data2_index++;
		_rx_payload_data2_id |=((b << 8)&0xff00) ;
		_decode_state = SONAR_DECODE_DATA2_ID1;
		break;
	case SONAR_DECODE_DATA2_ID1:
//		printf("33\n");
		add_byte_to_checksum(b); //SUMCHECK
//		printf("_rx_ck_a = %x\n",_rx_ck_a);
		_rx_payload_data2_index++;
//		printf("_rx_payload_data2_index = %d\n",_rx_payload_data2_index);
		_rx_payload_data2_id |= (b & 0x00ff);
		_decode_state = SONAR_DECODE_DATA2_LENGTH1;
		break;
	case SONAR_DECODE_DATA2_LENGTH1:
//		printf("44\n");
		_rx_payload_data2_index++;
		add_byte_to_checksum(b); //SUMCHECK
//		printf("_rx_ck_a = %x\n",_rx_ck_a);
		_rx_payload_data2_length1 = b;
		_decode_state = SONAR_DECODE_DATA2_PAYLOAD;
//		printf("_rx_payload_data2_index = %d\n",_rx_payload_data2_index);
		if (payload_rx_data2_init() != SONARDATA_CONTINUE)// start payload reception
		{
			// payload will not be handled, discard message
			printf(" data2 payload init error!!!");
			decode_init();
		} else {
			_decode_state = (_rx_payload_data2_length1 > 0) ? SONAR_DECODE_DATA2_PAYLOAD : SONAR_DECODE_DATA2_CHKSUM;
		}
		break;
	case SONAR_DECODE_DATA2_PAYLOAD:

		add_byte_to_checksum(b); //SUMCHECK
//		SOANR_TRACE_PARSER(".");
		switch (_rx_payload_data2_id & 0xff00) {
		case SONAR_FRONT:// add timestamp
//			printf("55\n");
			ret = payload_rx_add_front(b);
			break;
		case SONAR_BACK:// add orientation
//		/	printf("66\n");
			ret = payload_rx_add_back(b);	// add a MON-VER payload byte
			break;
		case SONAR_LEFT:// add acceleration
//			printf("77\n");
			ret = payload_rx_add_left(b);
			break;
		case SONAR_RIGHT_T:// add position
//			printf("88\n");
			ret = payload_rx_add_right(b);
			break;
		case SONAR_DOWN:// add Angular Velocity
//			printf("99\n");
			ret = payload_rx_add_down(b);

			break;
		default:
//			ret = payload_rx_add_velocity(b);
			break;
		}
//		printf("_rx_ck_a = %x\n",_rx_ck_a);
		if (SONARDATA_ERROR == ret) {
//			mti_error_count++;
//			printf("SONARDATA_ERROR\n");
			ret = SONARDATA_CONTINUE;
			// payload not handled, discard message
			decode_init();
		}else if (SONARDATA_DATA_COMP ==  ret) {
			// payload complete, expecting checksum
//		/	printf("SONARDATA_DATA_COMP\n");
			_rx_payload_data2_id = 0x0000;
			ret = SONARDATA_CONTINUE;
			_decode_state = SONAR_DECODE_DATA2_ID;// next packet ID
		}else if(SONARDATA_COMP ==  ret){
//			printf("SONARDATA_COMP\n");
		//when ret ==0 ; all packet is recieved.
			ret = SONARDATA_CONTINUE;
			_decode_state = SONAR_DECODE_DATA2_CHKSUM;
			// expecting more payload, stay in state MTI_DECODE_PAYLOAD
		}
		break;
		case SONAR_DECODE_DATA2_CHKSUM:
			if ((_rx_ck_a += b)!= 0X00) {
//				printf("SONAR checksum err");
//				printf(" checksum = %x ",_rx_ck_a += b);
				ret = SONARDATA_CONTINUE ;
				decode_init();
			} else
			{
				printf("SONAR checksum sucess!!!");
				ret = payload_rx_data2_done();	// finish payload processing
				ret = SONARDATA_ALL_COMP;
				decode_init();
			}
			break;
//------------ MTDATA---------------//
	/* Expecting MTDATA length byte */
	case SONAR_DECODE_DATA_LENGTH:
		SONAR_TRACE_PARSER("E");
		add_byte_to_checksum(b); //SUMCHECK
		_rx_payload_data_length = b;
				//开始准备接收数据
		_decode_state = SONAR_DECODE_DATA_PAYLOAD;
		if (payload_rx_data_init() != 0)// start payload reception
		{
			// payload will not be handled, discard message
			decode_init();
		} else {
			_decode_state = (_rx_payload_data_length > 0) ? SONAR_DECODE_DATA_PAYLOAD : SONAR_DECODE_DATA_CHKSUM;
		}
		break;
	/* Expecting MTDATA payload */
	case SONAR_DECODE_DATA_PAYLOAD:
		add_byte_to_checksum(b);
		ret = payload_rx_data_add(b);
		if (SONARDATA_ERROR == ret){
	   // payload not handled, discard message
			decode_init();
			ret = SONARDATA_CONTINUE;
		} else if (SONARDATA_COMP == ret){
	   // payload complete, expecting checksum
			_decode_state = SONAR_DECODE_DATA_CHKSUM;
			ret = SONARDATA_CONTINUE;
		} else{
	  // expecting more payload, stay in state SONAR_DECODE_PAYLOAD
			ret = SONARDATA_CONTINUE;
		}
		break;
	/* Expecting first checksum byte */
	case SONAR_DECODE_DATA_CHKSUM:
		if ((_rx_ck_a += b)!= 0x00) {
			printf("SONAR checksum err");
			decode_init();
		} else
		{
			printf("SONAR checksum sucess!!!");
			ret = payload_rx_data_done();	// finish payload processing
			ret = SONARDATA_ALL_COMP;
			decode_init();
		}
		break;
	default:
		printf("^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\n");
		ret = SONARDATA_CONTINUE ;
		decode_init();
		break;
	}
	return ret;
}
//-----------add SONAR function for init payload of MTdata/MTdata2
/**
 * Start payload rx of mtdata
 */
int SONAR::payload_rx_data_init(void)
{
	int ret = SONARDATA_CONTINUE; // in default, did not check the length;
	if(SONAR_RX_DATA_LENGTH != _rx_payload_data_length)
	{
		ret = SONARDATA_ERROR;
	}
	return ret ;
}
int SONAR::payload_rx_data2_init(void)
{
	int ret = SONARDATA_CONTINUE;
	_rx_state = SONAR_DATAID_HANDLE;
	switch(_rx_payload_data2_id & 0xff00){
	case SONAR_FRONT:
		if(_rx_payload_data2_length1 != SONAR_DATA_LEGHTH)
			{
			_rx_state = SONAR_DATAID_ERROR_LENGTH;
		}
		break;
	case SONAR_BACK:
		if(_rx_payload_data2_length1 != SONAR_DATA_LEGHTH)
			{
			_rx_state = SONAR_DATAID_ERROR_LENGTH;
		}
		break;
	case SONAR_LEFT:
		if(_rx_payload_data2_length1 != SONAR_DATA_LEGHTH)
			{
			_rx_state = SONAR_DATAID_ERROR_LENGTH;
		}
		break;
	case SONAR_RIGHT_T:
		if(_rx_payload_data2_length1 != SONAR_DATA_LEGHTH)
		{
			_rx_state = SONAR_DATAID_ERROR_LENGTH;
		}
		break;
	case SONAR_DOWN:
		if(_rx_payload_data2_length1 != SONAR_DATA_LEGHTH)
			{
			_rx_state = SONAR_DATAID_ERROR_LENGTH;
		}
		break;
	default:
		_rx_state = SONAR_DATAID_DISABLE;
		break;
		}

	switch(_rx_state){
	case SONAR_DATAID_HANDLE:
	case SONAR_DATAID_IGNORE:
		ret = SONARDATA_CONTINUE;
		break;
	case SONAR_DATAID_DISABLE:
		printf("SONAR msg SOANR_DATAID_DISABLe,********************%\n");
		ret = SONARDATA_ERROR;
		break;
	case SONAR_DATAID_ERROR_LENGTH:
		printf("SONAR msg 0x%04x invalid len %u", ((unsigned)_rx_payload_data2_id), (unsigned)_rx_payload_data2_length1);
		ret = SONARDATA_ERROR;
		break;
	default:
		printf("SONAR internal err1");
		ret = SONARDATA_ERROR;
		break;
	}
	return ret;
}
/**
 * Add payload rx byte of MTDATA
 */
int	// -1 = error, 0 = ok, 1 = payload completed
SONAR:: payload_rx_data_add(const uint8_t b)
{
	int ret = SONARDATA_CONTINUE;
	SonarBuf.raw[_rx_payload_data_index++] = b;
	if (_rx_payload_data_index == _rx_payload_data_length) {
		ret = SONARDATA_COMP;	// payload received completely
	}
	return ret;
}

//-----------------------------------------//
void
SONAR::decode_init(void)
{
	_decode_state = SONAR_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_payload_data2_id = 0;
	_rx_payload_data_length = 0;
	_rx_payload_data2_length = 0; // ALL PACKET LENGTH
	_rx_payload_data2_length1 = 0; // THE LENGTH OF ONE PACKET
	_rx_payload_data_index = 0;
	_rx_payload_data2_index = 0; //EXTERN INDEX OF data2
	_rx_payload_data2_index1 = 0; // index of dataid IN MTDATA2
}

// data decode
int SONAR::payload_rx_add_front(const uint8_t b)
{

	int ret = SONARDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(SONAR_FRONT2== (_rx_payload_data2_id & 0X00ff))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			Temp_Data[_rx_payload_data2_index1++] = b;
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = SONARDATA_DATA_COMP;
			Sonar_Front = ((Temp_Data[0]&0x00ff)<<8)|(Temp_Data[1]&0x00ff);
			printf("front = 0x%x \n",Sonar_Front);
//			printf("Temp_Data[0] = 0x%x \n",Temp_Data[0]);
//			printf("Temp_Data[1] = 0x%x \n",Temp_Data[1]);
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = SONARDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = SONARDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_FreeAcceleration". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = SONARDATA_ERROR;
//		error_count++;
		return ret;
	}
}// end add FRONT
//BACK
int SONAR::payload_rx_add_back(const uint8_t b)
{

	int ret = SONARDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(SONAR_BACK2== (_rx_payload_data2_id & 0X00ff))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			Temp_Data[_rx_payload_data2_index1++] = b;
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = SONARDATA_DATA_COMP;
//			_rx_payload_data2_id = 0;
			Sonar_Back= ((Temp_Data[0]&0x00ff)<<8)|(Temp_Data[1]&0x00ff);
			printf("back = 0x%x \n",Sonar_Back);
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = SONARDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = SONARDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_FreeAcceleration". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = SONARDATA_ERROR;
//		error_count++;
		return ret;
	}
}// end add acceleration
int SONAR::payload_rx_add_left(const uint8_t b)
{

	int ret = SONARDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(SONAR_LEFT2== (_rx_payload_data2_id & 0X00ff))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			Temp_Data[_rx_payload_data2_index1++] = b;
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = SONARDATA_DATA_COMP;
//			_rx_payload_data2_id = 0;
			Sonar_Left = ((Temp_Data[0]&0x00ff)<<8)|(Temp_Data[1]&0x00ff);
			printf("left = 0x%x \n",Sonar_Left);
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = SONARDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = SONARDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_FreeAcceleration". we consider as error in this process.
//		printf("the ID of velocity messsage is error!! \n");
		ret = SONARDATA_ERROR;
//		error_count++;
		return ret;
	}
}// end add acceleration
int SONAR::payload_rx_add_right(const uint8_t b)
{

	int ret = SONARDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(SONAR_RIGHT_T2== (_rx_payload_data2_id & 0X00ff))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			Temp_Data[_rx_payload_data2_index1++] = b;
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = SONARDATA_DATA_COMP;
//			_rx_payload_data2_id = 0;
			Sonar_Right = ((Temp_Data[0]&0x00ff)<<8)|(Temp_Data[1]&0x00ff);
			printf("right = 0x%x \n",Sonar_Right);
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = SONARDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = SONARDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_FreeAcceleration". we consider as error in this process.
//		printf("the ID of velocity messsage is error!! \n");
		ret = SONARDATA_ERROR;
//		error_count++;
		return ret;
	}
}// end add acceleration
int SONAR::payload_rx_add_down(const uint8_t b)
{

	int ret = SONARDATA_CONTINUE;
	_rx_payload_data2_index++; // all index add
	// select euler angles descrble orientation
	if(SONAR_DOWN2== (_rx_payload_data2_id & 0X00ff))
	{

		if(_rx_payload_data2_index1 < _rx_payload_data2_length1)
		{
			Temp_Data[_rx_payload_data2_index1++] = b;
		}
		if(_rx_payload_data2_index1 == _rx_payload_data2_length1){
			ret = SONARDATA_DATA_COMP;
//			_rx_payload_data2_id = 0;
			Sonar_Down = ((Temp_Data[0]&0x00ff)<<8)|(Temp_Data[1]&0x00ff);
			printf("down = 0x%x \n",Sonar_Down);
			_rx_payload_data2_index1 = 0;// packet reciever end, init  packet length index.
		}
		if(_rx_payload_data2_index == _rx_payload_data2_length)
			{
			_rx_payload_data2_index = 0;
			ret = SONARDATA_COMP;
		}
		if(_rx_payload_data2_index < _rx_payload_data2_index1){
			ret = SONARDATA_ERROR;
//			error_count++;
		}
		return ret;
	}else{
		// to do somthing when the data type is not "XDI_FreeAcceleration". we consider as error in this process.
		printf("the ID of velocity messsage is error!! \n");
		ret = SONARDATA_ERROR;
//		error_count++;
		return ret;
	}
}// end add acceleration

//---------------------------------------------//

void
SONAR::add_byte_to_checksum(const uint8_t b)
{
	_rx_ck_a += b; //只用计算除preamble以外的所有数据之和
}
int SONAR::payload_rx_data2_done()
{
	int ret = SONARDATA_ERROR;
	if(_rx_payload_data_index == _rx_payload_data_length)
	{

		_sonar_position->SonarDown = Sonar_Down;
		_sonar_position->SonarFront = Sonar_Front;
		_sonar_position->SonarBack= Sonar_Back;
		_sonar_position->SonarLeft = Sonar_Left;
		_sonar_position->SonarRight = Sonar_Right;
		ret = SONARDATA_ALL_COMP;
	}
	Sonar_Down = 0;
	Sonar_Front = 0;
	Sonar_Back = 0;
	Sonar_Left = 0;
	Sonar_Right = 0;
	return ret;
}

int SONAR::payload_rx_data_done(void)
{
	int ret = SONARDATA_ERROR;
	if(_rx_payload_data_index == _rx_payload_data_length)
	{
//		_sonar_position->SonarDown = SonarBuf.temp_sonar.SonarDown;
		_sonar_position->SonarDown = (0xff00&(SonarBuf.raw[0]<<8))|(0x00ff&SonarBuf.raw[1]);
//		printf("%d\n",SonarBuf.raw[0]);
//		printf("%d\n",SonarBuf.raw[1]);
//		printf("%x\n",_sonar_position->SonarDown);
		_sonar_position->SonarFront = (0xff00&(SonarBuf.raw[2]<<8))|(0x00ff&SonarBuf.raw[3]);//SonarBuf.temp_sonar.SonarFront;
		_sonar_position->SonarBack= (0xff00&(SonarBuf.raw[4]<<8))|(0x00ff&SonarBuf.raw[5]);//SonarBuf.temp_sonar.SonarLeft;
		_sonar_position->SonarLeft = (0xff00&(SonarBuf.raw[6]<<8))|(0x00ff&SonarBuf.raw[7]);//SonarBuf.temp_sonar.SonarRight;
		_sonar_position->SonarRight = (0xff00&(SonarBuf.raw[8]<<8))|(0x00ff&SonarBuf.raw[9]);//SonarBuf.temp_sonar.SonarBack;
//		printf("%x\n",_sonar_position->SonarDown);
//		printf("%x\n",_sonar_position->SonarBack);
//		printf("%x\n",_sonar_position->SonarFront);
//		printf("%x\n",_sonar_position->SonarLeft);
//		printf("%x\n",_sonar_position->SonarRight);
//		_sonar_position->SonarReportStatus = SonarBuf.temp_sonar.SonarReportStatus;
//		_sonar_position->TimeStamp = hrt_absolute_time();
//		_sonar_position->SonarReportStatus = Valid;
//		_sonar_position->SonarFront = SonarBuf.sonar_data_temp_s.SonarFrontTemp;
//		_sonar_position->SonarBack =SonarBuf.sonar_data_temp_s.SonarBackTemp;
//		_sonar_position->SonarRight = SonarBuf.sonar_data_temp_s.SonarRightTemp;
//		_sonar_position->SonarLeft =SonarBuf.sonar_data_temp_s.SonarLeftTemp;
//		_sonar_position->SonarDown = SonarBuf.sonar_data_temp_s.SonarDownTemp;
//		_sonar_position->SonarBack =SonarBuf.SonarBackTemp;
//		_sonar_position->SonarFront = SonarBuf.SonarFrontTemp;
		ret = SONARDATA_ALL_COMP;
	}
	return ret;
}

void
SONAR::send_message(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	if (payload != nullptr)
	{
		write(_fd, (const void *)payload, length);
		printf("send message !\n");
	}
}


