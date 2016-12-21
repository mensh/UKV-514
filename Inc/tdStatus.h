#ifndef __TD_STATUS_H_
#define __TD_STATUS_H_

typedef enum { Busy = 0, Ok = 1, Fail = 2, TimeOut = 3, ProcessBreak = 4 } tdStatus;
typedef enum { ARINC429 = 0x20 } tdEnumDataType;

#endif
