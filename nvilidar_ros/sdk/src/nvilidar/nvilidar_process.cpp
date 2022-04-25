#include "nvilidar_process.h"
#include <list>
#include <string>
#include "myconsole.h"
#include "mytimer.h"
#include "mystring.h"
#include <iostream> 
#include <istream> 
#include <sstream>

namespace nvilidar
{
	LidarProcess::LidarProcess(LidarCommTypeEnum comm, std::string name_ip, uint32_t port_baud)
	{
		//通信模式 
		LidarCommType = comm;

		//雷达接口  首先初始化 
		Nvilidar_UserConfigTypeDef  cfg;
		//获取默认参数  如需要修改 可以进行修改  
		LidarDefaultUserConfig(cfg);

		//根据不同的通信接口 初始化不同的信息 
		if (USE_SERIALPORT == comm)
		{
			cfg.serialport_name = name_ip;		//串口名 
			cfg.serialport_baud = port_baud;	//串口波特率 

			lidar_serial.LidarLoadConfig(cfg);	//串口 
		}
		else if (USE_SOCKET == comm)
		{
			cfg.ip_addr = name_ip;
			cfg.lidar_udp_port = port_baud;
			
			lidar_udp.LidarLoadConfig(cfg);	//网络接口 
			lidar_net_cfg.LidarLoadConfig(cfg);	//配置参数  
		}
	}
	LidarProcess::~LidarProcess()
	{

	}

	//雷达初始化 读写参数等信息 
	bool LidarProcess::LidarInitialialize()
	{
		//根据不同的通信接口 初始化不同的信息 
		if (USE_SERIALPORT == LidarCommType)
		{
			return lidar_serial.LidarInitialialize();
		}
		else if (USE_SOCKET == LidarCommType)
		{
			return lidar_udp.LidarInitialialize();
		}
		return false;
	}

	//启动雷达 
	bool LidarProcess::LidarTurnOn()
	{
		//根据不同的通信接口 初始化不同的信息 
		if (USE_SERIALPORT == LidarCommType)
		{
			return lidar_serial.LidarTurnOn();
		}
		else if (USE_SOCKET == LidarCommType)
		{
			return lidar_udp.LidarTurnOn();
		}
		return false;
	}

	//停止雷达 
	bool LidarProcess::LidarTurnOff()
	{
		//根据不同的通信接口 初始化不同的信息 
		if (USE_SERIALPORT == LidarCommType)
		{
			return lidar_serial.LidarTurnOff();
		}
		else if (USE_SOCKET == LidarCommType)
		{
			return lidar_udp.LidarTurnOff();
		}
		return false;
	}

	//雷达轮询机制  
	bool LidarProcess::LidarSamplingProcess(LidarScan &scan, uint32_t timeout)
	{
		if (USE_SERIALPORT == LidarCommType)
		{
			return lidar_serial.LidarSamplingProcess(scan,timeout);
		}
		else if (USE_SOCKET == LidarCommType)
		{
			return lidar_udp.LidarSamplingProcess(scan, timeout);
		}
		return false;
	}

	//退出 
	void LidarProcess::LidarCloseHandle()
	{
		if (USE_SERIALPORT == LidarCommType)
		{
			lidar_serial.LidarCloseHandle();
		}
		else if (USE_SOCKET == LidarCommType)
		{
			lidar_udp.LidarCloseHandle();
		}
	}

	//=========================参数同步=================================

	//同步数据信息  相关信息 同步到雷达 
	void  LidarProcess::LidarParaSync(Nvilidar_UserConfigTypeDef &cfg)
	{
		cfg.storePara.samplingRate = (uint32_t)(cfg.sampling_rate * 1000);		// * 1000
		cfg.storePara.angleOffset = (uint16_t)(cfg.angle_offset * 64 + 0.5);	//角度偏移 	实际与雷达的  64倍 U16 	
		cfg.storePara.isHasSensitive = cfg.sensitive;							//是否带有信号质量 
		cfg.storePara.aimSpeed = (uint16_t)(cfg.aim_speed * 100 + 0.5);			//N Hz 实际与雷达的  100倍 U16 
		cfg.storePara.tailingLevel = cfg.tailing_level;							//拖尾等级 

		//ingnore array拆分 
		std::vector<float> elems;
		std::stringstream ss(cfg.ignore_array_string);
		std::string number;
		while (std::getline(ss, number, ',')) {
			elems.push_back(atof(number.c_str()));
		}
		cfg.ignore_array = elems;

		//看是否有需要过滤的数据 
		if (cfg.ignore_array.size() % 2)
		{
			nvilidar::console.error("ignore array is odd need be even");
		}
		for (uint16_t i = 0; i < cfg.ignore_array.size(); i++)
		{
			if (cfg.ignore_array[i] < -180.0 && cfg.ignore_array[i] > 180.0)
			{
				nvilidar::console.error("ignore array should be between 0 and 360");
			}
		}
	}

	//初始参数 
	void  LidarProcess::LidarDefaultUserConfig(Nvilidar_UserConfigTypeDef &cfg)
	{
		//配置参数 
		cfg.serialport_baud = 921600;
		cfg.serialport_name = "/dev/nvilidar";
		cfg.ip_addr = "192.168.1.200";	//192.168.1.200 为雷达默认IP 可更改 
		cfg.lidar_udp_port = 8100;				//8100为默认雷达传输用端口 不可更改 
		cfg.config_tcp_port = 8200;			//8200为默认雷达配置参数用端口 不可更改 
		cfg.frame_id = "laser_frame";
		cfg.resolution_fixed = false;		//非固定角分辨 
		cfg.auto_reconnect = false;			//自动重连 
		cfg.reversion = false;				//倒转 
		cfg.inverted = false;				//180度 
		cfg.angle_max = 180.0;
		cfg.angle_min = -180.0;
		cfg.range_max = 64.0;
		cfg.range_min = 0;
		cfg.aim_speed = 10.0;				//10Hz
		cfg.sampling_rate = 10;				//10k
		cfg.sensitive = false;				//数据不加信号质量 
		cfg.tailing_level = 6;			//拖尾等级  
		cfg.angle_offset = 0.0;				//角度偏移 
		cfg.single_channel = false;			//单通道 
		cfg.ignore_array_string = "";				//忽略部分角度信息 
		//过滤点信息 
		cfg.filter_jump_enable = true;		//使能跳动点过滤 
		cfg.filter_jump_value_min = 3;		//跳动点最小过滤值 
		cfg.filter_jump_value_max = 25;		//跳动点最大过滤值 

		LidarParaSync(cfg);
	}

	//==========================其它接口   获取串口列表=======================================
	std::string LidarProcess::LidarGetSerialList()
	{
		std::string port;       //选择的串口
		std::vector<NvilidarSerialPortInfo> ports = nvilidar::LidarDriverSerialport::getPortList();       //获取串口列表
		std::vector<NvilidarSerialPortInfo>::iterator it;

		//列表信息
		if (ports.empty())
		{
			nvilidar::console.show("Not Lidar was detected.");
			return 0;
		}
		else if (1 == ports.size())
		{
			it = ports.begin();
			port = (*it).portName;
		}
		else
		{
			int id = 0;
			for (it = ports.begin(); it != ports.end(); it++)
			{
				nvilidar::console.show("%d. %s  %s\n", id, it->portName.c_str(), it->description.c_str());
				id++;
			}
			while (1)
			{
				nvilidar::console.show("Please select the lidar port:");
				std::string number;
				std::cin >> number;

				//参数不合法 
				if ((size_t)atoi(number.c_str()) >= ports.size())
				{
					continue;
				}
				//参数配置 
				it = ports.begin();
				id = atoi(number.c_str());

				//查找  
				port = ports.at(id).portName;

				break;
			}
		}

		return port;
	}

	//================================其它接口  设置网络转接板参数配置=============================================
	bool LidarProcess::LidarSetNetConfig(std::string ip, std::string gateway, std::string mask)
	{
		Nvilidar_NetConfigTypeDef net_cfg;

		//建立连接  
		bool state = lidar_net_cfg.LidarNetConfigConnect();
		if (false == state)
		{
			nvilidar::console.warning("connect to config port error!");
			return 0;
		}

		//设置IP 
		//ip = "192.168.1.201";
		//gateway = "192.168.1.1";
		//mask = "255.255.255.0";
		state = lidar_net_cfg.LidarNetConfigWrite(ip, gateway, mask);
		if (false == state)
		{
			nvilidar::console.warning("set ip error!");
			return false;
		}
		delayMS(1000);
		//读取IP 
		state = lidar_net_cfg.LidarNetConfigRead(ip, gateway, mask);
		if (false == state)
		{
			nvilidar::console.warning("read ip error!");
			return false;
		}
		nvilidar::console.message("read net para:");
		nvilidar::console.message("ip:%s, gate:%s, mask:%s", ip.c_str(), gateway.c_str(), mask.c_str());

		//断开连接  
		lidar_net_cfg.LidarNetConfigDisConnect();

		return true;
	}

	//=============================ROS预留接口 重新加载参数========================================================
	void LidarProcess::LidarReloadPara(Nvilidar_UserConfigTypeDef cfg)
	{
		LidarParaSync(cfg);
		
		//根据不同的通信接口 初始化不同的信息 
		if (USE_SERIALPORT == LidarCommType)
		{
			lidar_serial.LidarLoadConfig(cfg);	//串口 
		}
		else if (USE_SOCKET == LidarCommType)
		{		
			lidar_udp.LidarLoadConfig(cfg);	//网络接口 
			lidar_net_cfg.LidarLoadConfig(cfg);	//配置参数  
		}
	}
}





