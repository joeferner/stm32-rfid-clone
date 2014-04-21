#ifndef TIME_H
#define	TIME_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

struct tm {
  int tm_sec;
  int tm_min;
  int tm_hour;
  int tm_mday;
  int tm_mon;
  int tm_year;
  int tm_wday;
  int tm_yday;
  int tm_isdst;
};
typedef long time_t;

void time_setup();
volatile uint32_t time_ms();
void time_SysTick_Handler();

struct tm * gmtime(register const time_t *timer);

#ifdef	__cplusplus
}
#endif

#endif	/* TIME_H */

