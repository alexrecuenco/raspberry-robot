#ifndef PI_HELPER_H
#define PI_HELPER_H

#include "wiringPins.h"

extern int get_default_var(const char* envvar, int default_value);

extern void wait_delay(unsigned int target_wait_ms, unsigned int last_millis);

extern void shutdown(void);
extern int startup(void);

#endif
