#include <3ds/synchronization.h>
#include <i2c/globals.h>
#include <3ds/result.h>
#include <3ds/types.h>
#include <3ds/svc.h>
#include <3ds/srv.h>
#include <i2c/ipc.h>
#include <3ds/os.h>
#include <memops.h>
#include <stdint.h>

#define countof(arr) (sizeof(arr) / sizeof(arr[0]))

// service constants

#define I2C_MAX_SESSIONS_PER_SERVICE 1
#ifdef N3DS
#define I2C_IPC_THREAD_STACKSIZE     0x800
#else
#define I2C_IPC_THREAD_STACKSIZE     0x400
#endif

static const struct
{
	const char *name;
	u8 len;
} I2C_ServiceConfigs[I2C_SERVICE_MAX] =
{
	{ .name = "i2c::MCU", .len = sizeof("i2c::MCU") - 1 },
	{ .name = "i2c::CAM", .len = sizeof("i2c::CAM") - 1 },
	{ .name = "i2c::LCD", .len = sizeof("i2c::LCD") - 1 },
	{ .name = "i2c::DEB", .len = sizeof("i2c::DEB") - 1 },
	{ .name = "i2c::HID", .len = sizeof("i2c::HID") - 1 },
	{ .name = "i2c::IR" , .len = sizeof("i2c::IR")  - 1 },
	{ .name = "i2c::EEP", .len = sizeof("i2c::EEP") - 1 },
#ifdef N3DS
	{ .name = "i2c::NFC", .len = sizeof("i2c::NFC") - 1 },
	{ .name = "i2c::QTM", .len = sizeof("i2c::QTM") - 1 },
#endif
};

__attribute__((section(".data.thread_stacks"), aligned(8))) static u8 I2C_ThreadStacks[I2C_SERVICE_MAX][I2C_IPC_THREAD_STACKSIZE] = { 0 };
__attribute__((section(".data.session_data"))) static I2C_SessionData I2C_SessionsData[I2C_SERVICE_MAX] = { 0 };

void _thread_start(void *);

Result startThread(Handle *thread, void (* function)(void *), void *arg, void *stack_top, s32 priority, s32 processor_id)
{
	if ((u32)(stack_top) & (0x8 - 1))
		return OS_MISALIGNED_ADDRESS;
	//_thread_start will pop these out
	((u32 *)stack_top)[-1] = (u32)function;
	((u32 *)stack_top)[-2] = (u32)arg;

	return svcCreateThread(thread, _thread_start, function, stack_top, priority, processor_id);
}

static inline void freeThread(Handle *thread)
{
	if (thread && *thread)
	{
		T(svcWaitSynchronization(*thread, -1))
		T(svcCloseHandle(*thread));
		*thread = 0;
	}
}

static inline I2C_SessionData *getNewSessionData(s32 service_index)
{
	I2C_SessionData *data = &I2C_SessionsData[service_index]; /* service_index = id */
	
	if (data->thread) {
		freeThread(&data->thread);
		_memset32_aligned(data, 0, sizeof(I2C_SessionData));
	}

	data->session_type = (I2C_SessionType)service_index;
	
	return data;
}

void I2C_SessionThreadMain(void *arg)
{
	I2C_SessionData *data = (I2C_SessionData *)arg;
	Result res = 0;
	s32 index = -1;
	
	IPC_StaticBuffer *staticbufs = getThreadStaticBuffers();
	
	staticbufs[0].desc = IPC_Desc_StaticBuffer(I2C_INPUT_STATICBUF_SIZE, 0);
	staticbufs[0].bufptr = data->input_staticbuf;
	staticbufs[1].desc = IPC_Desc_StaticBuffer(I2C_INPUT_STATICBUF_SIZE, 0);
	staticbufs[1].bufptr = data->input_staticbuf;

	getThreadCommandBuffer()[0] = 0xFFFF0000;

	while (true)
	{
		res = svcReplyAndReceive(&index, &data->session, 1, data->session);

		if (R_FAILED(res))
		{
			if (res != OS_REMOTE_SESSION_CLOSED)
				Err_Panic(res);

			break;
		}
		else if (index != 0)
			Err_Panic(OS_EXCEEDED_HANDLES_INDEX);

		I2C_HandleIPC(data);
	}

	T(svcCloseHandle(data->session))
}

static inline void initializeBSS()
{
	extern void *__bss_start__;
	extern void *__bss_end__;

	_memset32_aligned(__bss_start__, 0, (size_t)__bss_end__ - (size_t)__bss_end__);
}

#define SRV_NOTIF_REPLY(idx) (idx == 0) // handles[0]
#define SERVICE_REPLY(idx) (idx > 0 && idx < I2C_SERVICE_MAX + 1) // handles[1] until handles[9] (n3ds) or handles[7] (o3ds)

void I2C_Main()
{
	initializeBSS();

	T(srvInit());
	T(syncInit());

	/*
		handles[0]  = SRV notification handle
		handles[1]  = i2c::MCU server handle
		handles[2]  = i2c::CAM server handle
		handles[3]  = i2c::LCD server handle
		handles[4]  = i2c::DEB server handle
		handles[5]  = i2c::HID server handle
		handles[6]  = i2c::IR  server handle
		handles[7]  = i2c::EEP server handle
		for n3ds, in addition:
		handles[8]  = i2c::NFC server handle
		handles[9]  = i2c::QTM server handle
	*/
	Handle handles[1 + I2C_SERVICE_MAX];

	RecursiveLock_Init(&g_I2C_BusLocks[0]);
	RecursiveLock_Init(&g_I2C_BusLocks[1]);
	RecursiveLock_Init(&g_I2C_BusLocks[2]);
	T(svcCreateEvent(&g_I2C_BusInterrupts[0], RESET_ONESHOT));
	T(svcCreateEvent(&g_I2C_BusInterrupts[1], RESET_ONESHOT));
	T(svcCreateEvent(&g_I2C_BusInterrupts[2], RESET_ONESHOT));
	
	// handles[0] - srv notification event
	T(SRV_EnableNotification(&handles[0]));

	// handles[1] through handles[7] or handles[9] - services
	for (u8 i = 0, j = 1; i < I2C_SERVICE_MAX; i++, j++)
		T(SRV_RegisterService(&handles[j], I2C_ServiceConfigs[i].name, I2C_ServiceConfigs[i].len, I2C_MAX_SESSIONS_PER_SERVICE));
	
	T(svcBindInterrupt(0x54, g_I2C_BusInterrupts[0], 8, false));
	T(svcBindInterrupt(0x55, g_I2C_BusInterrupts[1], 8, false));
	T(svcBindInterrupt(0x5C, g_I2C_BusInterrupts[2], 8, false));
	
	while (true)
	{
		s32 index;

		Result res = svcWaitSynchronizationN(&index, handles, countof(handles), false, -1);

		if (R_FAILED(res))
			Err_Throw(res);

		if (SRV_NOTIF_REPLY(index)) // SRV event fired for notification
		{
			u32 notification_id = 0;
			T(SRV_ReceiveNotification(&notification_id))
			if (notification_id == 0x100) // terminate
				break;
		}
		else if (SERVICE_REPLY(index)) // service handle received request to create session
		{
			Handle session, thread;
			
			/* we only allow one active thread per service */
			I2C_SessionData *data = getNewSessionData(index - 1);

#ifdef N3DS
			s32 processor_id;
			switch (index - 1)
			{
			case I2C_SESSION_TYPE_CAM:
			case I2C_SESSION_TYPE_HID:
			case I2C_SESSION_TYPE_QTM:
				processor_id = 3;
				break;
			default:
				processor_id = -2;
			}
#else
			s32 processor_id = -2;
#endif
			
			T(svcAcceptSession(&session, handles[index]));
			data->session = session;
			
			T(startThread(&thread, &I2C_SessionThreadMain, data, I2C_ThreadStacks[index], 11, processor_id));

			data->thread = thread;
		}
		else // invalid index
			Err_Throw(I2C_INTERNAL_RANGE);
	}

	// wait and close thread handles
	for (u8 i = 0; i < I2C_SERVICE_MAX; i++)
		freeThread(&I2C_SessionsData[i].thread);
	
	T(svcCloseHandle(handles[0]));

	// unregister services
	for (u8 i = 0, j = 1; i < I2C_SERVICE_MAX; i++, j++)
	{
		T(SRV_UnregisterService(I2C_ServiceConfigs[i].name, I2C_ServiceConfigs[i].len));
		T(svcCloseHandle(handles[j]));
	}

	svcCloseHandle(g_I2C_BusInterrupts[0]);
	svcCloseHandle(g_I2C_BusInterrupts[1]);
	svcCloseHandle(g_I2C_BusInterrupts[2]);
	
	srvExit();
	syncExit();
}
