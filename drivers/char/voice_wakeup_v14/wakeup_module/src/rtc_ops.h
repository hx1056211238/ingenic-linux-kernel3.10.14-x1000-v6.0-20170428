#ifndef __RTC_OPS_H__
#define __RTC_OPS_H__

#include "rtc-jz.h"



/*#define __RTC_ALRM_IS_ENABLED()	()*/
/*#define __RTC_ALRM_ENABLE()	()*/
#define ALARM_VALUE		(30)/*10s*/

int rtc_save(void);
int rtc_restore(void);

int rtc_init(void);
int rtc_set_alarm(unsigned long alarm_seconds, int period);
int rtc_int_handler(void);
int rtc_exit(void);

int is_os_rtc_alarm_occur(void);
int rtc_set_alarm_and_polling_rtc_alarm_flag(unsigned long alarm_seconds);

#define SYS_TIMER	0x2
#define DMIC_TIMER	0x3

#endif
