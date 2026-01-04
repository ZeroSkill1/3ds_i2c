#ifndef _I2C_IPC_H
#define _I2C_IPC_H

#include <3ds/types.h>
#include <i2c/i2c.h>

#ifdef N3DS
#define I2C_INPUT_STATICBUF_SIZE 0x200
#define I2C_OUTPUT_STATICBUF_SIZE 0x200
#else
#define I2C_INPUT_STATICBUF_SIZE 0x10
#define I2C_OUTPUT_STATICBUF_SIZE 0x20
#endif

typedef struct I2C_SessionData
{
	Handle thread;
	Handle session; // needs to be freed in thread itself!
	I2C_SessionType session_type;
	u8 input_staticbuf[I2C_INPUT_STATICBUF_SIZE];
	u8 output_staticbuf[I2C_OUTPUT_STATICBUF_SIZE];
} I2C_SessionData;

void I2C_HandleIPC(I2C_SessionData *session);

#endif
