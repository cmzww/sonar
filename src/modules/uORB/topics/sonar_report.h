/*
 * sonar_report.h
 *
 *  Created on: 2014-12-16
 *      Author: uav-robotics
 */

/**
 * @file gnssins_report.h
 * Definition of the GPS and INS Report.
 */
#ifndef SONAR_REPORT_H_
#define SONAR_REPORT_H_

#include <stdint.h>
#include "../uORB.h"


/**
 * @addtogroup topics
 * @{
 */
enum SonarReportStatusE
{
    Invaild = 0,
    Poll_TimeOut,
    Data_TimeOut,
    Poll_Error,
    Rx_Error,
    CRCWrong,
    TimeOut,
    Break,
    Valid
};

struct SonarReportS
{
    uint64_t SonarTimeStamp;
    enum SonarReportStatusE SonarReportStatus;
    uint16_t SonarDown;
    uint16_t SonarFront;
    uint16_t SonarBack;
    uint16_t SonarRight;
    uint16_t SonarLeft;
    uint16_t sonar_error_count1;
//    uint32_t Status;
//	uint64_t temp;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(Sonar_Report);



#endif /* GNSSINS_REPORT_H_ */
