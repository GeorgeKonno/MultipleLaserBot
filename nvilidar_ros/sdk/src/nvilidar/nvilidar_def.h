#ifndef _NVILIDAR_DEF_H_
#define _NVILIDAR_DEF_H_

#include <stdint.h>
#include "nvilidar_protocol.h"
#include <string>


//======================================基本参数类型定义============================================ 

//SDK版本号 
#define NVILIDAR_SDKVerision     "1.0.5"

//PI def
#ifndef M_PI
#define M_PI        3.14159265358979323846
#endif 

//其它
#define NVILIDAR_DEFAULT_TIMEOUT     2000    //默认超时时间 正常协议  
#define NVILIDAR_POINT_TIMEOUT		 2000	 //一圈点云的超时时间 比如10hz 则该超时时间需要大于100ms 才可以保证不会出错 


//雷达型号
enum
{
   NVILIDAR_VP300      = 1,
   NVILIDAR_Tail,
};


//======================================基本数据类型定义============================================ 

//雷达信息
struct Nvilidar_PackageStateTypeDef
{
	bool m_CommOpen;              //串口开启标记
	bool m_Scanning;                //正在扫描出图
	uint8_t last_device_byte;       //上包接到的字节信息
};

//雷达本身参数信息 (存储在雷达内部)
struct  Nvilidar_StoreConfigTypeDef
{
	uint8_t     isHasSensitive;         //有信号质量信息
	uint16_t    aimSpeed;               //转速信息 x100
	uint32_t    samplingRate;           //采样率x1
	int16_t     angleOffset;            //角度偏移x64
	uint8_t     tailingLevel;          //拖尾等级
};

//数据信息 
struct Nvilidar_DeviceInfo
{
	std::string m_SoftVer;				//软件版本号 
	std::string m_HardVer;				//硬件版本号 
	std::string m_ProductName;			//产品名称  
	std::string m_SerialNum;			//序列号 
};

//雷达配置参数
struct  Nvilidar_UserConfigTypeDef
{
	std::string frame_id;				//ID
	std::string serialport_name;		//串口名 
	int    		serialport_baud;		//串口波特率 
	std::string ip_addr;				//IP地址 
	int    		lidar_udp_port;			//端口号 
	int    		config_tcp_port;		//配置端口号 
	bool		auto_reconnect;			//自动重连
    bool		reversion;				//倒置 反180度
	bool		inverted;				//镜像 左右反相 
	double		angle_max;				//最大角度值 
	double		angle_min;				//最小角度值 
	double		range_max;				//最小值  盲区 
	double		range_min;				//最大值  盲区 
	double 		aim_speed;				//转速  
	int			sampling_rate;			//采样率 
	bool		sensitive;				//是否带信号质量 
	int			tailing_level;			//拖尾等级 
	double 		angle_offset;			//角度偏移 
	bool     	single_channel;        	//单通道通信

	std::string ignore_array_string;	//过滤的部分 输入字符串 
	std::vector<float> ignore_array;	//过滤的部分 解后的容器信息 

	bool 		resolution_fixed;		//是否固定角分辨率 
	Nvilidar_DeviceInfo			deviceInfo;	//数据信息 
	Nvilidar_StoreConfigTypeDef	storePara;	//存储的参数信息 

	bool 		filter_jump_enable;		//是否允许过滤 
	int 		filter_jump_value_min;	//过滤点最小值 
	int 		filter_jump_value_max;	//过滤点最大值 
};

//共用体
union Nvilidar_PackageBufTypeDef
{
	uint8_t buf[1200];
	Nvilidar_Node_Package_Quality        pack_qua;
	Nvilidar_Node_Package_No_Quality     pack_no_qua;
};

//包信息
typedef struct 
{
	uint16_t packageIndex;         //单包采样点索引位置信息
	Nvilidar_PackageBufTypeDef  packageBuffer;    //包信息（实际内容）
	bool     packageErrFlag;       //包错误标记信息
	uint16_t packageCheckSumGet;   //校验值获取
	uint16_t packageCheckSumCalc;  //校验值计算
	uint16_t  packageFreq;          //雷达转速信息
	int16_t  packageTemp;          //雷达温度信息
	uint32_t packagePointTime;     //2点时间间隔
	uint16_t packageFirstAngle;    //起始采样角
	uint16_t packageLastAngle;     //结束采样角
	float    packageAngleDiffer;   //每2个点之间的角度差
	float    packageLastAngleDiffer; //最后一次算的2点角度的差值
	uint8_t  packagePointDistSize; //一个点对应的字节的大小信息
	bool     packageHas0CAngle;    //是否为0度角
	bool     packageHasTemp;       //是否当前位置为温度
	bool     packageHas0CFirst;    //第一个字节 判断是否是0度角
	bool     packageHasTempFirst;  //第一个字节 判断是否为温度信息
	uint16_t  package0CIndex;      //0度角索引（目前协议为非单独封包）
	uint64_t packageStamp;		   //接收完本包的时间 
	uint16_t packagePointNum;	   //一包的点数信息 
}Nvilidar_PointViewerPackageInfoTypeDef;

//接收信息 (只用于接收暂存 无其它用)
typedef struct
{
	bool	recvFinishFlag;
	Nvilidar_Protocol_DeviceInfo lidar_device_info;//雷达接收并且返回的数据  
	Nvilidar_Protocol_GetPara    lidar_get_para;	//获取参数信息 
	uint8_t     isHasSensitive;					//有信号质量信息
	uint16_t    aimSpeed;						//转速信息 x100
	uint32_t    samplingRate;					//采样率x1
	int16_t     angleOffset;					//角度偏移x64
	uint8_t     tailingLevel;					//拖尾等级
	uint8_t     saveFlag;						//是否保存成功了 
}NvilidarRecvInfoTypeDef;

//一圈点信息 
typedef struct
{
	uint64_t  startStamp;			//一圈起始时间戳 
	uint64_t  stopStamp;			//一圈结束时间戳 
	std::vector<Nvilidar_Node_Info>  lidarCircleNodePoints;	//一圈点云图数据
}CircleDataInfoTypeDef;



//======================================输出数据信息============================================ 
/**
 * @brief The Laser Point struct
 * @note angle unit: rad.\n
 * range unit: meter.\n
 */
typedef struct {
	/// lidar angle. unit(rad)
	float angle;
	/// lidar range. unit(m)
	float range;
	/// lidar intensity
	float intensity;
} NviLidarPoint;

/**
 * @brief A struct for returning configuration from the NVILIDAR
 * @note angle unit: rad.\n
 * time unit: second.\n
 * range unit: meter.\n
 */
typedef struct {
	/// Start angle for the laser scan [rad].  0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float min_angle;
	/// Stop angle for the laser scan [rad].   0 is forward and angles are measured clockwise when viewing NVILIDAR from the top.
	float max_angle;
	/// angle resoltuion [rad]
	float angle_increment;
	/// Scan resoltuion [s]
	float time_increment;
	/// Time between scans
	float scan_time;
	/// Minimum range [m]
	float min_range;
	/// Maximum range [m]
	float max_range;
} NviLidarConfig;


typedef struct {
	/// System time when first range was measured in nanoseconds
	uint64_t stamp;
	/// Array of lidar points
	std::vector<NviLidarPoint> points;
	/// Configuration of scan
	NviLidarConfig config;
} LidarScan;



#endif
