#ifndef DELAY_H
#define	DELAY_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

#ifdef	__cplusplus
}
#endif

#endif	/* DELAY_H */

