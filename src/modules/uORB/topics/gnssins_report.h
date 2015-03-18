/*
 * gnssins_report.h
 *
 *  Created on: 2014-12-16
 *      Author: uav-robotics
 */

/**
 * @file gnssins_report.h
 * Definition of the GPS and INS Report.
 */
#ifndef GNSSINS_REPORT_H_
#define GNSSINS_REPORT_H_

#include <stdint.h>
#include "../uORB.h"


/**
 * @addtogroup topics
 * @{
 */
enum GNSSINSReportStatusE
{
    IsInvaild = 0,
    CRCIsWrong,
    IsTimeOut,
    IsBreak,
    IsValid
};

struct GNSSINSReportS
{
    uint64_t TimeStamp;
    enum GNSSINSReportStatusE GNSSINSReportStatus;

    //WGS84
    double Latitude;
    double Longitude;
    //MSL
    float Altitude;

    //NED
    float LinearVelocityN;
    float LinearVelocityE;
    float LinearVelocityD;

    //Body-XYZ
    float LinearAccelerationX;
    float LinearAccelerationY;
    float LinearAccelerationZ;

    //Euler Angle
    float roll;
    float pitch;
    float yaw;

    //Body-XYZ
    float AngularVelocityX;
    float AngularVelocityY;
    float AngularVelocityZ;

    uint16_t mti_error_count1;
    uint32_t Status;
//	uint64_t temp;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(gnssins_report);



#endif /* GNSSINS_REPORT_H_ */
