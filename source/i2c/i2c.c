#include <3ds/synchronization.h>
#include <3ds/err.h>

#include <i2c/ipc.h>
#include <i2c/i2c.h>

Handle g_I2C_BusInterrupts[3] = { 0 };
RecursiveLock g_I2C_BusLocks[3] = { 0 };

#ifdef N3DS
#define I2C_DEVID_MAX 17
#else
#define I2C_DEVID_MAX 15
#endif

#define I2C_MAX_N_TRIES 8

static const I2C_DeviceConfig devConf[I2C_DEVID_MAX + 1] = {
	{ .port = 0, .write_addr = 0x4A }, 
	{ .port = 0, .write_addr = 0x7A }, 
	{ .port = 0, .write_addr = 0x78 }, 
	{ .port = 1, .write_addr = 0x4A }, 
	{ .port = 1, .write_addr = 0x78 }, 
	{ .port = 1, .write_addr = 0x2C }, 
	{ .port = 1, .write_addr = 0x2E }, 
	{ .port = 1, .write_addr = 0x40 }, 
	{ .port = 1, .write_addr = 0x44 }, 
	{ .port = 2, .write_addr = 0xD6 }, 
	{ .port = 2, .write_addr = 0xD0 }, 
	{ .port = 2, .write_addr = 0xD2 }, 
	{ .port = 2, .write_addr = 0xA4 }, 
	{ .port = 2, .write_addr = 0x9A }, 
	{ .port = 2, .write_addr = 0xA0 }, 
	{ .port = 1, .write_addr = 0xEE }, 
#ifdef N3DS
	{ .port = 0, .write_addr = 0x40 }, 
	{ .port = 2, .write_addr = 0x54 }, 
#endif
};

bool I2C_CheckDeviceAccess(I2C_SessionType session_type, u8 devid) {
	switch (session_type)
	{
	case I2C_SESSION_TYPE_MCU:
		return devid == 0 || devid == 3;
	case I2C_SESSION_TYPE_CAM:
		return devid == 1 || devid == 2 || devid == 4;
	case I2C_SESSION_TYPE_LCD:
		return devid == 5 || devid == 6;
	case I2C_SESSION_TYPE_DEB:
		return devid == 7 || devid == 8;
	case I2C_SESSION_TYPE_HID:
		return devid == 9 || devid == 10 || devid == 11 || devid == 12;
	case I2C_SESSION_TYPE_IR:
#ifdef N3DS
		return devid == 13 || devid == 17;
#else
		return devid == 13;
#endif
	case I2C_SESSION_TYPE_EEP:
		return devid == 14;
#ifdef N3DS
	case I2C_SESSION_TYPE_NFC:
		return devid == 15;
	case I2C_SESSION_TYPE_QTM:
		return devid == 16;
#endif
	default:
		return false;
	}
}

volatile I2C_BusRegset *const I2C_BUS[3] = {
	(I2C_BusRegset *)0x1EC61000,
	(I2C_BusRegset *)0x1EC44000,
	(I2C_BusRegset *)0x1EC48000,
};

static inline void spinwait(u32 n) {
	for (u32 i = n; i > 2; i -= 2) { }
}

void I2C_Initialize() {
	for (int i = 0; i < 3; i++) {
		I2C_BUS[i]->CNTEX = I2C_CNTEX_WAIT_SCL_IDLE;
		I2C_BUS[i]->SCL = I2C_SCL_HIGH_DURATION(5);
		T(svcClearEvent(g_I2C_BusInterrupts[i]));
	}
}

#define BUS(dc) (I2C_BUS[dc->port])
#define CHECK_ACK(dc) ((BUS(dc)->CNT & I2C_CNT_TXN_ACK) == I2C_CNT_TXN_ACK)

// low level

static bool I2C_SelectDevice(u8 devid) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	BUS(dc)->DATA = dc->write_addr;
	
	spinwait(1125);
	
	BUS(dc)->CNT = I2C_CNT_TXN_START | I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
	
	return CHECK_ACK(dc);
}

static bool I2C_SelectRegister(u8 devid, u8 regid) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	BUS(dc)->DATA = regid;
	BUS(dc)->CNT = I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
	
	return CHECK_ACK(dc);
}

static bool I2C_WriteIntermediate(u8 devid, u8 value) {
	return I2C_SelectRegister(devid, value); /* identical, apparently */
}

static void I2C_CancelTransaction(u8 devid) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	BUS(dc)->CNT = I2C_CNT_TXN_FINISH | I2C_CNT_TXN_CANCEL | I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
}

static bool I2C_BeginRead(u8 devid) {
	const I2C_DeviceConfig *dc = &devConf[devid];

	BUS(dc)->DATA = dc->write_addr | 1; // read address
	
	spinwait(1125);
	
	BUS(dc)->CNT = I2C_CNT_TXN_START | I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
	
	return CHECK_ACK(dc);
}

static u8 I2C_ReadIntermediate(u8 devid) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	BUS(dc)->CNT = I2C_CNT_TXN_ACK | I2C_CNT_DIRECTION_READ | I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
	
	return BUS(dc)->DATA;
}

static u8 I2C_FinishRead(u8 devid) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	BUS(dc)->CNT = I2C_CNT_TXN_FINISH | I2C_CNT_DIRECTION_READ | I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
	
	return BUS(dc)->DATA;
}

static bool I2C_FinishWrite(u8 devid, u8 value) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	BUS(dc)->DATA = value;
	BUS(dc)->CNT = I2C_CNT_TXN_FINISH | I2C_CNT_IRQ_ENABLE | I2C_CNT_ENABLE;
	
	TIS(svcWaitSynchronization(g_I2C_BusInterrupts[dc->port], -1));
	
	return CHECK_ACK(dc);
}

// low-ish level

static bool _I2C_ReadRegister8(u8 devid, u8 regid, u8 *out_val) {
	bool res = false;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid)) ||
			!(res = I2C_BeginRead(devid))) {
				I2C_CancelTransaction(devid);
				continue;
		} else {
			break;
		}
	}
	
	if (!res) return res;
	
	*out_val = I2C_FinishRead(devid);
	
	return true;
}

static bool _I2C_WriteRegister8(u8 devid, u8 regid, u8 value) {
	bool res = false;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid)) ||
			!(res = I2C_FinishWrite(devid, value))) {
				I2C_CancelTransaction(devid);
				continue;
		} else {
			break;
		}
	}
	
	return res;
}

static bool _I2C_WriteDevice8(u8 devid, u8 value) {
	bool res = false;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_FinishWrite(devid, value))) {
				I2C_CancelTransaction(devid);
				continue;
		} else {
			break;
		}
	}
	
	return res;
}

static bool _I2C_ReadRegister16(u8 devid, u16 regid, u16 *out_val) {
	bool res = false;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid >> 8)) ||
			!(res = I2C_SelectRegister(devid, regid & 0xFF)) ||
			!(res = I2C_BeginRead(devid))) {
				I2C_CancelTransaction(devid);
				continue;
		} else {
			break;
		}
	}
	
	if (!res) return res;
	
	*out_val = I2C_ReadIntermediate(devid) << 8;
	*out_val |= I2C_FinishRead(devid);
	
	return true;
}

static bool _I2C_WriteRegister16(u8 devid, u16 regid, u16 value) {
	bool res = false;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid >> 8)) ||
			!(res = I2C_SelectRegister(devid, regid & 0xFF)) ||
			!(res = I2C_WriteIntermediate(devid, value >> 8)) ||
			!(res = I2C_FinishWrite(devid, value & 0xFF))) {
				I2C_CancelTransaction(devid);
				continue;
		} else {
			break;
		}
	}
	
	return res;
}

bool I2C_ReplaceRegisterBits8(u8 devid, u8 regid, u8 value, u8 mask) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u8 curval = 0;
	
	if (!_I2C_ReadRegister8(devid, regid, &curval))
		goto exit;
	
	curval = (curval &~ mask) | (value & mask);
	
	res = _I2C_WriteRegister8(devid, regid, curval);
	
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

bool I2C_ReplaceRegisterBits16(u8 devid, u16 regid, u16 value, u16 mask) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u16 curval = 0;
	
	if (!_I2C_ReadRegister16(devid, regid, &curval))
		goto exit;
	
	curval = (curval &~ mask) | (value & mask);
	
	res = _I2C_WriteRegister16(devid, regid, curval);
	
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

bool I2C_WriteRegister8(u8 devid, u8 regid, u8 value) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	bool res = _I2C_WriteRegister8(devid, regid, value);
	
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	
	return res;
}

bool I2C_WriteDevice8(u8 devid, u8 value) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	bool res = _I2C_WriteDevice8(devid, value);
	
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	
	return res;
}

bool I2C_WriteRegister16(u8 devid, u16 regid, u16 value) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	bool res = _I2C_WriteRegister16(devid, regid, value);
	
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	
	return res;
}

bool I2C_ReadRegister8(u8 devid, u8 regid, u8 *out_value) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	bool res = _I2C_ReadRegister8(devid, regid, out_value);
	
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

bool I2C_ReadRegister16(u8 devid, u16 regid, u16 *out_value) {
	const I2C_DeviceConfig *dc = &devConf[devid];
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	bool res = _I2C_ReadRegister16(devid, regid, out_value);
	
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

bool I2C_WriteRegisters8(u8 devid, u8 regid, const u8 *buf, u32 size) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		if (size == 1)
			break;
		
		for (; index < size - 1; index++) {
			if (!(res = I2C_WriteIntermediate(devid, buf[index]))) {
				I2C_CancelTransaction(devid);
				goto retry;
			}
		}
		
		break;
retry:
	}
	
	if (!res) goto exit;
	
	res = I2C_FinishWrite(devid, buf[size - 1]);
	
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}
	
bool I2C_WriteRegisters16(u8 devid, u16 regid, const u16 *buf, u32 count) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid >> 8)) ||
			!(res = I2C_SelectRegister(devid, regid & 0xFF))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		if (count == 1)
			break;
		
		for (; index < count - 1; index++) {
			if (!(res = I2C_WriteIntermediate(devid, buf[index] >> 8)) ||
				!(res = I2C_WriteIntermediate(devid, buf[index] & 0xFF))) {
				I2C_CancelTransaction(devid);
				goto retry;
			}
		}
		
		break;
retry:
	}
	
	if (!res || !(res = I2C_WriteIntermediate(devid, buf[count - 1] >> 8))) goto exit;
	
	res = I2C_FinishWrite(devid, buf[count - 1] & 0xFF);
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}
	
bool I2C_ReadRegisters8(u8 devid, u8 regid, u8 *buf, u32 size) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid)) ||
			!(res = I2C_BeginRead(devid))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		if (size == 1)
			break;
		
		for (; index < size - 1; index++) {
			buf[index] = I2C_ReadIntermediate(devid);
		}
		
		break;
	}
	
	if (!res) goto exit;
	
	buf[size - 1] = I2C_FinishRead(devid);
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

bool I2C_ReadRegisters16(u8 devid, u16 regid, u16 *buf, u32 count) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		if (!(res = I2C_SelectDevice(devid)) ||
			!(res = I2C_SelectRegister(devid, regid >> 8)) ||
			!(res = I2C_SelectRegister(devid, regid & 0xFF)) ||
			!(res = I2C_BeginRead(devid))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		if (count == 1)
			break;
		
		for (; index < count - 1; index++) {
			buf[index] = I2C_ReadIntermediate(devid) << 8;
			buf[index] |= I2C_ReadIntermediate(devid);
		}
		
		break;
	}
	
	if (!res) goto exit;
	
	buf[count - 1] = I2C_ReadIntermediate(devid) << 8;
	buf[count - 1] |= I2C_FinishRead(devid);
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

/* no clue what this is for, maybe used in previous versions? not used in anything i've looked at */
bool I2C_ReadRegisters8Legacy(u8 devid, u8 regid, u8 *buf, u32 size) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		svcSleepThread(50000);
		
		if (!(res = I2C_SelectDevice(devid)))
			goto retry;
		
		if (!(res = I2C_FinishWrite(devid, regid)))
			goto retry;
		
		svcSleepThread(150000);
		
		if (!(res = I2C_BeginRead(devid)))
			goto retry;
		
		if (size == 1)
			break;
		
		for (; index < size - 1; index++) {
			buf[index] = I2C_ReadIntermediate(devid);
		}
		
		break;
retry:
		I2C_CancelTransaction(devid);
	}
	
	if (!res) goto exit;
	
	buf[size - 1] = I2C_FinishRead(devid);
	svcSleepThread(150000);
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

#ifdef N3DS
bool I2C_ReadDeviceRaw(u8 devid, u8 *out_value) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		if (!(res = I2C_BeginRead(devid))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		*out_value = I2C_FinishRead(devid);
		break;
	}

	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}

bool I2C_WriteDeviceRawMulti(u8 devid, const u8 *buf, u32 size) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		if (!(res = I2C_SelectDevice(devid))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		if (size == 1)
			break;
		
		for (; index < size - 1; index++) {
			if (!(res = I2C_WriteIntermediate(devid, buf[index]))) {
				I2C_CancelTransaction(devid);
				goto retry;
			}
		}
		
		break;
retry:
	}
	
	if (!res) goto exit;
	
	res = I2C_FinishWrite(devid, buf[size - 1]);
	
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}
	
bool I2C_ReadDeviceRawMulti(u8 devid, u8 *buf, u32 size) {
	if (devid > I2C_DEVID_MAX)
		return false;
	
	const I2C_DeviceConfig *dc = &devConf[devid];
	bool res = false;
	
	RecursiveLock_Lock(&g_I2C_BusLocks[dc->port]);
	
	u32 index = 0;
	
	for (int i = 0; i < I2C_MAX_N_TRIES; i++) {
		index = 0;
		
		if (!(res = I2C_BeginRead(devid))) {
			I2C_CancelTransaction(devid);
			continue;
		}
		
		if (size == 1)
			break;
		
		for (; index < size - 1; index++) {
			buf[index] = I2C_ReadIntermediate(devid);
		}
		
		break;
	}
	
	if (!res) goto exit;
	
	buf[size - 1] = I2C_FinishRead(devid);
exit:
	RecursiveLock_Unlock(&g_I2C_BusLocks[dc->port]);
	return res;
}
#endif
