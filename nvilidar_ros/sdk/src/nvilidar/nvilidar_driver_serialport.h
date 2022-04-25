#pragma once

#include "nvilidar_def.h"
#include "nvilidar_protocol.h"
#include "serial/nvilidar_serial.h"
#include "nvilidar_filter.h"
#include <string>
#include <vector>
#include <stdint.h>
#include <math.h>

#if defined(_WIN32)
#include <conio.h>
#include <WinSock2.h>
#include <windows.h>
#include <process.h>
#include <tlhelp32.h>
#include <sys/utime.h>
#include <io.h>
#include <direct.h>
#else
#include <assert.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#endif

//串口列表信息 
typedef struct 
{
	std::string portName;
	std::string description;
}NvilidarSerialPortInfo;

//---定义库信息 VS系列的生成库文件  
#ifdef WIN32
	#define NVILIDAR_DRIVER_SERIAL_API __declspec(dllexport)
#else
	#define NVILIDAR_DRIVER_SERIAL_API
#endif // ifdef WIN32

namespace nvilidar
{
    //lidar driver 
	class  NVILIDAR_DRIVER_SERIAL_API LidarDriverSerialport
    {
		public:
			LidarDriverSerialport();                //构造函数
			~LidarDriverSerialport();           //析构函数

			void LidarLoadConfig(Nvilidar_UserConfigTypeDef cfg);
			bool LidarIsConnected();			//雷达是否连接
			bool LidarGetScanState();			//获取传输状态  
			bool LidarInitialialize();			//雷达初始化 获取参数配置参数等 
			bool LidarCloseHandle();			//雷达断开连接  
			bool LidarTurnOn();				//启动扫描  
			bool LidarTurnOff();				//停止扫描


			//串口 版本号等信息
			std::string getSDKVersion();										//获取当前sdk版本号
			static std::vector<NvilidarSerialPortInfo> getPortList();			//获取机器串口列表
			bool StartScan(void);                           //启动扫图
			bool StopScan(void);                            //停止扫描
			bool Reset(void);								//雷达复位

			bool SetIntensities(const uint8_t has_intensity, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);     //设置是否有信号质量
			bool GetDeviceInfo(Nvilidar_DeviceInfo & info, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetScanMotorSpeed(uint16_t frequency_add, uint16_t &ret_frequency,    //设置雷达目标转速
													uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetSamplingRate(uint32_t rate_add, uint32_t &rate,
											uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);   //雷达采样率增加
			bool SetTrailingLevel(uint8_t tral_set, uint8_t &tral,
												uint32_t  timeout = NVILIDAR_DEFAULT_TIMEOUT);
			//获取配置信息
			bool GetLidarCfg(Nvilidar_StoreConfigTypeDef &info,
												uint32_t  timeout = NVILIDAR_DEFAULT_TIMEOUT);

			bool GetZeroOffsetAngle(int16_t &angle,			//设置0度偏移
												uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);
			bool SetZeroOffsetAngle(int16_t angle_set, int16_t &angle,
												uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT); //读取0度偏移
			//保存参数
			bool SaveCfg(bool &flag,uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);

			//最终输出数据  一圈点云的数据  
			bool LidarSamplingProcess(LidarScan &scan, uint32_t timeout = NVILIDAR_DEFAULT_TIMEOUT);

			////变量 
			Nvilidar_PackageStateTypeDef   lidar_state;				//雷达状态

		private:
			bool LidarConnect(std::string portname, uint32_t baud = 921600);  //串口初始化
			void LidarDisconnect();      //关闭串口
			bool SendSerial(const uint8_t *data, size_t size);      //发送串口接口  私有类
			void FlushSerial();		//刷新串口数据  
			bool SendCommand(uint8_t cmd, const void *payload = NULL,uint16_t payloadsize = 0);
			void NormalDataUnpack(uint8_t *buf, uint16_t len);		//解包（普通数据解包）
			void NormalDataAnalysis(Nvilidar_Protocol_NormalResponseData data);	
			bool PointDataUnpack(uint8_t *byte, uint16_t len);		//解包（点云数据解包）
			void PointDataAnalysis(Nvilidar_PointViewerPackageInfoTypeDef data);	
			
			//线程相关 
			bool createThread();		//创建线程 
			void closeThread();			//关闭线程 
			bool waitNormalResponse(uint32_t timeout = NVILIDAR_POINT_TIMEOUT);	//等待雷达应答 一圈的点云数据 
			void setNormalResponseUnlock();	//解锁 
			void setCircleResponseUnlock();	//解锁 
			void LidarSamplingData(CircleDataInfoTypeDef info, LidarScan &outscan);		//拆包 

			//----------------------串口类---------------------------

			nvilidar_serial::Nvilidar_Serial serialport;

			//-----------------------过滤信息------------------------
			nvilidar::LidarFilter lidar_filter;

			//-----------------------变量----------------------------
			Nvilidar_UserConfigTypeDef     lidar_cfg;				//雷达型号
			CircleDataInfoTypeDef		   circleDataInfo;			//一圈的点云数据 
			NvilidarRecvInfoTypeDef		   recv_info;				//接收信息 

			uint32_t    m_0cIndex = 0;                  //0度所用的index
			int32_t     m_last0cIndex = 0;              //0度所用的index
			uint32_t    m_differ0cIndex = 0;            //0度所用的index
			bool        m_first_circle_finish = false;  //first circle finish,case calc fault
			uint64_t	m_run_circles = 0;				//从启动开始  已经发了几包的数据  

			//---------------------线程相关---------------------------
			#if defined(_WIN32)
				HANDLE  _thread = NULL;
				HANDLE  _event_analysis;		//协议解析 
				HANDLE  _event_circle;			//一圈点数据信息 

				DWORD static WINAPI periodThread(LPVOID lpParameter);		//线程进程  定时调用接口 
			#else 
				pthread_t _thread = -1;
				pthread_cond_t _cond_analysis;
				pthread_mutex_t _mutex_analysis;
				pthread_cond_t _cond_point;
				pthread_mutex_t _mutex_point;
				static void *periodThread(void *lpParameter) ;
			#endif
    };
}
