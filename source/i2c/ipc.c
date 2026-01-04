#include <3ds/synchronization.h>
#include <3ds/err.h>


#include <i2c/globals.h>
#include <i2c/i2c.h>
#include <i2c/ipc.h>

#define CMD_ID_RANGE(id, lower, upper) \
	(id >= lower && id <= upper)

#define I2C_CHKPERM(x) (I2C_CheckDeviceAccess(session->session_type, devid) ? (x) : I2C_UNAUTHORIZED)
#define I2C_CHKPERM_EXPLICIT (I2C_CheckDeviceAccess(session->session_type, devid) ? 0 : I2C_UNAUTHORIZED)
#define I2C_TRY(x) ((x) ? 0 : I2C_FATAL_FAIL)
#define I2CT(x) I2C_CHKPERM(I2C_TRY(x))

void I2C_HandleIPC(I2C_SessionData *session)
{
	u32 *cmdbuf = getThreadCommandBuffer();
	u32 cmd_header = cmdbuf[0];
	u16 cmd_id = (cmd_header >> 16) & 0xFFFF;
	
	switch (cmd_id)
	{
	case 0x0001: // replace register bits (8 bit variant)
		{
			CHECK_HEADER(0x0001, 4, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u8 value = (u8)(cmdbuf[3] & 0xFF);
			u8 mask  = (u8)(cmdbuf[4] & 0xFF);

			Result res = I2CT(I2C_ReplaceRegisterBits8(devid, regid, value, mask));

			cmdbuf[0] = IPC_MakeHeader(0x0001, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0002: // set register bits (8 bit variant)
		{
			CHECK_HEADER(0x0002, 3, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u8 mask  = (u8)(cmdbuf[3] & 0xFF);

			Result res = I2CT(I2C_ReplaceRegisterBits8(devid, regid, mask, mask));

			cmdbuf[0] = IPC_MakeHeader(0x0002, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0003: // clear register bits (8 bit variant)
		{
			CHECK_HEADER(0x0003, 3, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u8 mask  = (u8)(cmdbuf[3] & 0xFF);

			Result res = I2CT(I2C_ReplaceRegisterBits8(devid, regid, 0, mask));

			cmdbuf[0] = IPC_MakeHeader(0x0003, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0004: // replace register bits (16 bit variant) across multiple devices
		{
			CHECK_HEADER(0x0004, 4, 2);

			u16 regid = (u16)(cmdbuf[1] & 0xFFFF);
			u16 value = (u16)(cmdbuf[2] & 0xFFFF);
			u16 mask = (u16)(cmdbuf[3] & 0xFFFF);
			u32 n_devids = cmdbuf[4];
			const u8 *devids = (const u8 *)cmdbuf[6];

			CHECK_WRONGARG(
				!IPC_VerifyStaticBuffer(cmdbuf[5], 0) ||
				IPC_GetStaticBufferSize(cmdbuf[5]) != n_devids
			);

			Result res = 0;

			for (u32 i = 0; i < n_devids; i++) {
				u8 devid = devids[i];
				if (R_FAILED(res = I2C_CHKPERM_EXPLICIT)) {
					break;
				}
			}

			if (R_SUCCEEDED(res)) {
				for (u32 i = 0; i < n_devids; i++) {
					if (R_FAILED(res = I2C_TRY(I2C_ReplaceRegisterBits16(devids[i], regid, value, mask)))) {
						break;
					}
				}
			}

			cmdbuf[0] = IPC_MakeHeader(0x0004, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0005: // write register (8 bit variant)
		{
			CHECK_HEADER(0x0005, 3, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u8 value = (u8)(cmdbuf[3] & 0xFF);

			Result res = I2CT(I2C_WriteRegister8(devid, regid, value));

			cmdbuf[0] = IPC_MakeHeader(0x0005, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0006: // write (8 bit variant) (select device without selecting register afterwards)
		{
			CHECK_HEADER(0x0006, 2, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 value = (u8)(cmdbuf[2] & 0xFF);

			Result res = I2CT(I2C_WriteDevice8(devid, value));

			cmdbuf[0] = IPC_MakeHeader(0x0006, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0007: // write register (16 bit variant)
		{
			CHECK_HEADER(0x0007, 3, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u16 regid = (u16)(cmdbuf[2] & 0xFFFF);
			u16 value = (u16)(cmdbuf[3] & 0xFFFF);

			Result res = I2CT(I2C_WriteRegister16(devid, regid, value));

			cmdbuf[0] = IPC_MakeHeader(0x0007, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0008: // write register (16 bit variant) across multiple devices
		{
			CHECK_HEADER(0x0008, 3, 2);

			u16 regid = (u16)(cmdbuf[1] & 0xFFFF);
			u16 value = (u16)(cmdbuf[2] & 0xFFFF);
			u32 n_devids = cmdbuf[3];
			const u8 *devids = (const u8 *)cmdbuf[5];

			CHECK_WRONGARG(
				!IPC_VerifyStaticBuffer(cmdbuf[4], 0) ||
				IPC_GetStaticBufferSize(cmdbuf[4]) != n_devids
			);

			Result res = 0;

			for (u32 i = 0; i < n_devids; i++) {
				u8 devid = devids[i];
				if (R_FAILED(res = I2C_CHKPERM_EXPLICIT)) {
					break;
				}
			}

			if (R_SUCCEEDED(res)) {
				for (u32 i = 0; i < n_devids; i++) {
					if (R_FAILED(res = I2C_TRY(I2C_WriteRegister16(devids[i], regid, value)))) {
						break;
					}
				}
			}

			cmdbuf[0] = IPC_MakeHeader(0x0008, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0009: // read register (8 bit variant)
		{
			CHECK_HEADER(0x0009, 2, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u8 value = 0;

			Result res = I2CT(I2C_ReadRegister8(devid, regid, &value));

			cmdbuf[0] = IPC_MakeHeader(0x0009, 2, 0);
			cmdbuf[1] = res;
			cmdbuf[2] = value;
		}
		break;
	case 0x000A: // read register (16 bit variant)
		{
			CHECK_HEADER(0x000A, 2, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u16 regid = (u16)(cmdbuf[2] & 0xFFFF);
			u16 value = 0;

			Result res = I2CT(I2C_ReadRegister16(devid, regid, &value));

			cmdbuf[0] = IPC_MakeHeader(0x000A, 2, 0);
			cmdbuf[1] = res;
			cmdbuf[2] = value;
		}
		break;
	case 0x000B: // write registers (8 bit variant)
	case 0x000E: // write registers (8 bit variant) same for some reason?
		{
			CHECK_HEADER(cmd_id, 3, 2);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u32 size = cmdbuf[3];
			const u8 *buf = (const u8 *)cmdbuf[5];

			CHECK_WRONGARG(
				!IPC_VerifyStaticBuffer(cmdbuf[4], 1) ||
				IPC_GetStaticBufferSize(cmdbuf[4]) != size
			);

			Result res = I2CT(I2C_WriteRegisters8(devid, regid, buf, size));

			cmdbuf[0] = IPC_MakeHeader(cmd_id, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x000C: // write registers (16 bit variant)
		{
			CHECK_HEADER(0x000C, 3, 2);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u16 regid = (u16)(cmdbuf[2] & 0xFFFF);
			u32 count = cmdbuf[3];
			const u16 *buf = (const u16 *)cmdbuf[5];

			CHECK_WRONGARG(
				!IPC_VerifyStaticBuffer(cmdbuf[4], 1) ||
				(IPC_GetStaticBufferSize(cmdbuf[4]) / 2) != count
			);

			Result res = I2CT(I2C_WriteRegisters16(devid, regid, buf, count));

			cmdbuf[0] = IPC_MakeHeader(0x000C, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x000D: // read registers (8 bit variant)
		{
			CHECK_HEADER(0x000D, 3, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u32 size = cmdbuf[3];

			if (size > sizeof(session->output_staticbuf))
				size = sizeof(session->output_staticbuf);

			Result res = I2CT(I2C_ReadRegisters8(devid, regid, session->output_staticbuf, size));

			cmdbuf[0] = IPC_MakeHeader(0x000D, 1, 2);
			cmdbuf[1] = res;
			cmdbuf[2] = IPC_Desc_StaticBuffer(size, 0);
			cmdbuf[3] = (u32)session->output_staticbuf;
		}
		break;
	// note: 0x000E is handled with 0x000B since they're 1:1 identical
	case 0x000F: // read registers (8 bit variant) with delay, possibly legacy?
		{
			CHECK_HEADER(0x000F, 3, 0);

			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u32 size = cmdbuf[3];

			if (size > sizeof(session->output_staticbuf))
				size = sizeof(session->output_staticbuf);

			Result res = I2CT(I2C_ReadRegisters8Legacy(devid, regid, session->output_staticbuf, size));

			cmdbuf[0] = IPC_MakeHeader(0x000F, 1, 2);
			cmdbuf[1] = res;
			cmdbuf[2] = IPC_Desc_StaticBuffer(size, 0);
			cmdbuf[3] = (u32)session->output_staticbuf;
		}
		break;
	case 0x0010: // read registers (16 bit variant)
		{
			CHECK_HEADER(0x0010, 3, 0);
		
			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u16 regid = (u16)(cmdbuf[2] & 0xFFFF);
			u32 count = cmdbuf[3];
		
			if (count > sizeof(session->output_staticbuf) / 2)
				count = sizeof(session->output_staticbuf) / 2;
		
			Result res = I2CT(I2C_ReadRegisters16(devid, regid, (u16 *)session->output_staticbuf, count));
		
			cmdbuf[0] = IPC_MakeHeader(0x0010, 1, 2);
			cmdbuf[1] = res;
			cmdbuf[2] = IPC_Desc_StaticBuffer(count * sizeof(u16), 0);
			cmdbuf[3] = (u32)session->output_staticbuf;
		}
		break;
	case 0x0011: // write registers (8 bit variant) using mapped buffer
		{
			CHECK_HEADER(0x0011, 3, 2);
			
			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u32 size = cmdbuf[3];
			const u8 *buf = (const u8 *)cmdbuf[5];
			
			CHECK_WRONGARG(
				!IPC_VerifyBuffer(cmdbuf[4], IPC_BUFFER_R) ||
				IPC_GetBufferSize(cmdbuf[4]) != size
			);
			
			Result res = I2CT(I2C_WriteRegisters8(devid, regid, buf, size));
			
			cmdbuf[0] = IPC_MakeHeader(0x0011, 1, 2);
			cmdbuf[1] = res;
			cmdbuf[2] = IPC_Desc_Buffer(size, IPC_BUFFER_R);
			cmdbuf[3] = (u32)buf;
		}
		break;
	case 0x0012: // read registers (8 bit variant) using mapped buffer
		{
			CHECK_HEADER(0x0012, 3, 2);
			
			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u8 regid = (u8)(cmdbuf[2] & 0xFF);
			u32 size = cmdbuf[3];
			u8 *buf = (u8 *)cmdbuf[5];
			
			CHECK_WRONGARG(
				!IPC_VerifyBuffer(cmdbuf[4], IPC_BUFFER_W) ||
				IPC_GetBufferSize(cmdbuf[4]) != size
			);
			
			Result res = I2CT(I2C_ReadRegisters8(devid, regid, buf, size));
			
			cmdbuf[0] = IPC_MakeHeader(0x0012, 1, 2);
			cmdbuf[1] = res;
			cmdbuf[2] = IPC_Desc_Buffer(size, IPC_BUFFER_W);
			cmdbuf[3] = (u32)buf;
		}
		break;
	case 0x0013: // [n3ds only] read device raw
		{
			CHECK_HEADER(0x0013, 1, 0);
			
			u8 value = 0;
#ifdef N3DS
			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			
			Result res = I2CT(I2C_ReadDeviceRaw(devid, &value));
#else
			Result res = I2C_NOT_IMPLEMENTED;
#endif

			cmdbuf[0] = IPC_MakeHeader(0x0013, 2, 0);
			cmdbuf[1] = res;
			cmdbuf[2] = value;
		}
		break;
	case 0x0014: // [n3ds only] write device raw (multi)
		{
			CHECK_HEADER(0x0014, 2, 2);
			
#ifdef N3DS
			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			u32 size = cmdbuf[2];
			const u8 *buf = (const u8 *)cmdbuf[4];
			
			CHECK_WRONGARG(
				!IPC_VerifyStaticBuffer(cmdbuf[3], 1) ||
				IPC_GetStaticBufferSize(cmdbuf[3]) != cmdbuf[2]
			);
			
			Result res = I2CT(I2C_WriteDeviceRawMulti(devid, buf, size));
#else
			Result res = I2C_NOT_IMPLEMENTED;
#endif

			cmdbuf[0] = IPC_MakeHeader(0x0014, 1, 0);
			cmdbuf[1] = res;
		}
		break;
	case 0x0015: // [n3ds only] read device raw (multi)
		{
			CHECK_HEADER(0x0015, 2, 0);
			
			u32 size = cmdbuf[2];
#ifdef N3DS
			u8 devid = (u8)(cmdbuf[1] & 0xFF);
			
			if (size > sizeof(session->output_staticbuf))
				size = sizeof(session->output_staticbuf);
			
			Result res = I2CT(I2C_ReadDeviceRawMulti(devid, session->output_staticbuf, size));
#else
			Result res = I2C_NOT_IMPLEMENTED;
#endif

			cmdbuf[0] = IPC_MakeHeader(0x0015, 1, 2);
			cmdbuf[1] = res;
			cmdbuf[2] = IPC_Desc_StaticBuffer(size, 0);
			cmdbuf[3] = (u32)session->output_staticbuf;
		}
		break;
	default:
		RET_OS_INVALID_IPCARG
	}
}
