#include <stdio.h>
#include <iostream>
#include "nvilidar_process.h"


using namespace std;
using namespace nvilidar;

#if defined(_MSC_VER)
#pragma comment(lib, "nvilidar_driver.lib")
#endif


int main()
{
	printf(" _   ___      _______ _      _____ _____          _____ \n");
	printf("| \\ | \\ \\    / /_   _| |    |_   _|  __ \\   /\\   |  __ \\\n");
	printf("|  \\| |\\ \\  / /  | | | |      | | | |  | | /  \\  | |__) |\n");
	printf("| . ` | \\ \\/ /   | | | |      | | | |  | |/ /\\ \\ |  _  / \n");
	printf("| |\\  |  \\  /   _| |_| |____ _| |_| |__| / ____ \\| | \\ \\\n");
	printf("|_| \\_|   \\/   |_____|______|_____|_____/_/    \\_\\_|  \\ \\\n");
	printf("\n");
	fflush(stdout);

	//初始化信号 用于命令行退出  
	nvilidar::sigInit();

	//应答超时 
	static uint32_t  no_response_times = 0;

	//初始化参数  网络和串口参数不相同 
#if 1
	nvilidar::LidarProcess lidar(USE_SERIALPORT,"/dev/nvilidar",921600);
#else 
	nvilidar::LidarProcess lidar(USE_SOCKET, "192.168.1.200", 8100);
#endif 

	//初始化雷达  包括读参 配参 
	if (false == lidar.LidarInitialialize())		
	{
		return 0;
	}

	//启动雷达  读写参数  
	bool ret = lidar.LidarTurnOn();

	//雷达点云图数据解析及处理
	while (ret && (nvilidar::isOK()))
	{
		LidarScan scan;

		if (lidar.LidarSamplingProcess(scan))
		{
			no_response_times = 0;

			for (size_t i = 0; i < scan.points.size(); i++)
			{
				//float angle = scan.points.at(i).angle;
				//float dis = scan.points.at(i).range;
				//printf("a:%f,d:%f\n", angle, dis);
			}
			nvilidar::console.message("Scan received[%llu]: %u ranges is [%f]Hz",
				scan.stamp, (unsigned int)scan.points.size(),
				1.0 / scan.config.scan_time);
		}
		else
		{
			no_response_times++;
			if (no_response_times >= 5)
			{
				no_response_times = 0;
				nvilidar::console.warning("Failed to get Lidar Data");

				break;
			}
		}

		delayMS(5);		//此处必须要加sleep 否则会占用超高cpu 
	}
	lidar.LidarTurnOff();      //停止扫描 
	nvilidar::console.message("lidar is stopping......");
	lidar.LidarCloseHandle();   //关闭连接 

	delayMS(100);

	return 0;
}
