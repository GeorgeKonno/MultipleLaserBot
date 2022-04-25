#pragma once 

#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include <inttypes.h>
#if defined(_WIN32)
#include <mmsystem.h>
#include <sysinfoapi.h>
#include <WinSock2.h>
#include <windows.h>
#endif 

#if defined(_WIN32)
	//获取当前ns数
	inline uint64_t getStamp(void)
	{
		FILETIME		t;
		GetSystemTimeAsFileTime(&t);		//此函数接口获取的是100ns的时间戳 
		return ((((uint64_t)t.dwHighDateTime) << 32) | ((uint64_t)t.dwLowDateTime)) *
			100;
	}

	//获取到当前ms数 
	inline uint64_t getMS(void)
	{
		return GetTickCount();
	}

	//延时 
	inline void delayMS(uint32_t ms)
	{
		Sleep(ms);
	}

#else 
	//获取当前ns数
	inline uint64_t getStamp(void)
	{
		#if 1
			struct timespec	tim;
			clock_gettime(CLOCK_REALTIME, &tim);
			return static_cast<uint64_t>(tim.tv_sec) * 1000000000LL + tim.tv_nsec;
		#else
			struct timeval timeofday;
			gettimeofday(&timeofday, NULL);
			return static_cast<uint64_t>(timeofday.tv_sec) * 1000000000LL +
			static_cast<uint64_t>(timeofday.tv_usec) * 1000LL;
		#endif
	}

	//延时 
	inline void delayMS(uint32_t ms)
	{
		usleep(ms*1000);
	}

#endif 