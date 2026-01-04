#ifndef _I2C_I2C_H
#define _I2C_I2C_H

#include <3ds/types.h>

typedef enum I2C_SessionType {
	I2C_SESSION_TYPE_MCU = 0x0,
	I2C_SESSION_TYPE_CAM = 0x1,
	I2C_SESSION_TYPE_LCD = 0x2,
	I2C_SESSION_TYPE_DEB = 0x3,
	I2C_SESSION_TYPE_HID = 0x4,
	I2C_SESSION_TYPE_IR  = 0x5,
	I2C_SESSION_TYPE_EEP = 0x6,
#ifdef N3DS
	I2C_SESSION_TYPE_NFC = 0x7,
	I2C_SESSION_TYPE_QTM = 0x8,
	I2C_SERVICE_MAX      = I2C_SESSION_TYPE_QTM + 1,
#else
	I2C_SERVICE_MAX      = I2C_SESSION_TYPE_EEP + 1,
#endif
} I2C_SessionType;

typedef struct __attribute__((packed)) I2C_BusRegset {
	vu8 DATA;
	vu8 CNT;
	vu16 CNTEX;
	vu16 SCL;
} I2C_BusRegset;

typedef struct I2C_DeviceConfig {
	u8 port;
	u8 write_addr;
} I2C_DeviceConfig;

enum {
	I2C_CNT_TXN_FINISH     = BIT(0), // stop / finish transaction
	I2C_CNT_TXN_START      = BIT(1), // start / begin transaction
	I2C_CNT_TXN_CANCEL     = BIT(2), // cancel transaction
	I2C_CNT_TXN_ACK        = BIT(4), // transaction acknowledge(d)
	I2C_CNT_DIRECTION_READ = BIT(5), // data direction
	I2C_CNT_IRQ_ENABLE     = BIT(6), // enable IRQ
	I2C_CNT_ENABLE         = BIT(7), // general enable
};

enum {
	I2C_CNTEX_WAIT_SCL_IDLE = BIT(1),
};

#define I2C_SCL_LOW_DURATION(val) (val & 0x3F)
#define I2C_SCL_HIGH_DURATION(val) ((val & 0x1F) << 8)

void I2C_Initialize();
bool I2C_CheckDeviceAccess(I2C_SessionType session_type, u8 devid);

bool I2C_ReplaceRegisterBits8(u8 devid, u8 regid, u8 value, u8 mask);
bool I2C_ReplaceRegisterBits16(u8 devid, u16 regid, u16 value, u16 mask);

bool I2C_WriteRegister8(u8 devid, u8 regid, u8 value);
bool I2C_WriteDevice8(u8 devid, u8 value);
bool I2C_WriteRegister16(u8 devid, u16 regid, u16 value);

bool I2C_ReadRegister8(u8 devid, u8 regid, u8 *out_value);
bool I2C_ReadRegister16(u8 devid, u16 regid, u16 *out_value);

bool I2C_WriteRegisters8(u8 devid, u8 regid, const u8 *buf, u32 size);
bool I2C_WriteRegisters16(u8 devid, u16 regid, const u16 *buf, u32 count);

bool I2C_ReadRegisters8(u8 devid, u8 regid, u8 *buf, u32 size);
bool I2C_ReadRegisters16(u8 devid, u16 regid, u16 *buf, u32 count);
bool I2C_ReadRegisters8Legacy(u8 devid, u8 regid, u8 *buf, u32 size);

#ifdef N3DS
bool I2C_ReadDeviceRaw(u8 devid, u8 *out_value);
bool I2C_WriteDeviceRawMulti(u8 devid, const u8 *buf, u32 size);
bool I2C_ReadDeviceRawMulti(u8 devid, u8 *buf, u32 size);
#endif


#endif
