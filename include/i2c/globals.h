#ifndef _I2C_GLOBALS_H
#define _I2C_GLOBALS_H

#include <3ds/synchronization.h>

extern RecursiveLock g_I2C_BusLocks[3];
extern Handle g_I2C_BusInterrupts[3];

#endif
