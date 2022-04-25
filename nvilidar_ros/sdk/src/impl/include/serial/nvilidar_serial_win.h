#if defined(_WIN32)

#ifndef _NVILIDAR_SERIAL_WIN
#define _NVILIDAR_SERIAL_WIN

#include <string>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <process.h> //_beginthreadex
#include <tchar.h>   //tchar
#include <WinSock2.h>
#include <windows.h>

#define NVILIDAR_SERIAL_API __declspec(dllexport)


//串口信息宏定义 
//校验 
#define ParityNone  0  ///< No Parity 无校验
#define ParityOdd   1   ///< Odd Parity 奇校验
#define ParityEven  2  ///< Even Parity 偶校验
#define ParityMark  3  ///< Mark Parity 1校验
#define ParitySpace  4 ///< Space Parity 0校验

//数据位
#define DataBits5  5 ///< 5 data bits 5位数据位
#define DataBits6  6 ///< 6 data bits 6位数据位
#define DataBits7  7 ///< 7 data bits 7位数据位
#define DataBits8  8  ///< 8 data bits 8位数据位

//停止位
#define	StopOne  0        ///< 1 stop bit 1位停止位
#define	StopOneAndHalf 1 ///< 1.5 stop bit 1.5位停止位 - This is only for the Windows platform
#define	StopTwo   2         ///< 2 stop bit 2位停止位

//流控
#define FlowNone  0    ///< No flow control 无流控制
#define FlowHardware  1 ///< Hardware(RTS / CTS) flow control 硬件流控制
#define FlowSoftware  2  ///< Software(XON / XOFF) flow control 软件流控制

namespace nvilidar_serial
{
    class NVILIDAR_SERIAL_API Nvilidar_Serial
    {
    public:
		Nvilidar_Serial();
        ~Nvilidar_Serial();
        void serialInit(std::string portName,
                      int baudRate = 512000,
                      int parity = ParityNone,
                      int dataBits = DataBits8,
                      int stopbits = StopOne,
                      int flowControl = FlowNone,
                      unsigned int readBufferSize = 512);

        bool serialOpen();
        void serialClose();
        bool isSerialOpen();        //串口是否打开 
        int  serialReadAvaliable(); //读可读字节的长度 
        int  serialReadData(const uint8_t *data,int len);
        int  serialWriteData(const uint8_t *data,int len);        //写数据  
        void serialSetFlowControl(int flow);
        void serialFlush();         //刷新数据 
    private:

        std::string m_portName;
        int m_baudRate;
        int m_parity;
        int m_dataBits;
        int m_stopbits;
        int m_flowControl;

        COMMCONFIG serialConfig;
        COMMTIMEOUTS serialConfigTimeout;
        HANDLE  serialHandle = INVALID_HANDLE_VALUE;

    };
};

#endif

#endif