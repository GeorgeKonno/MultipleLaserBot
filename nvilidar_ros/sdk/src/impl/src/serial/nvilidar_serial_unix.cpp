#if defined(__linux__)

#include "serial/nvilidar_serial_unix.h"
#include <errno.h>   /* Error number definitions */
#include <fcntl.h>   /* File control definitions */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <unistd.h>  /* UNIX standard function definitions */

#include <sys/ioctl.h> //ioctl

namespace nvilidar_serial
{
    Nvilidar_Serial::Nvilidar_Serial()
    {

    }

    Nvilidar_Serial::~Nvilidar_Serial()
    {
        serialClose();
    }

    //初始化 
    void Nvilidar_Serial::serialInit(std::string portName,int baudRate,int parity,
                                   int dataBits,int stopbits,
                                    int flowControl,unsigned int readBufferSize)
    {
        m_portName = portName; // portName;//串口 /dev/ttySn, USB /dev/ttyUSBn
        m_baudRate = baudRate;
        m_parity = parity;
        m_dataBits = dataBits;
        m_stopbits = stopbits;
        m_flowControl = flowControl;
    }

    //设置串口参数  
    int Nvilidar_Serial::serialSetpara(int fd,int baudRate,int parity,int dataBits,
                 int stopbits,int flowControl)
    {
        struct termios options;

        //获取终端属性
        if (tcgetattr(fd, &options) < 0)
        {
            perror("tcgetattr error");
            return -1;
        }

        //设置输入输出波特率
        int baudRateConstant = 0;
        baudRateConstant = rate2UnixBaud(baudRate);

        if (0 != baudRateConstant)
        {
            cfsetispeed(&options, baudRateConstant);
            cfsetospeed(&options, baudRateConstant);
        }
        else
        {
            // TODO: custom baudrate
            fprintf(stderr, "Unkown baudrate!\n");
            return -1;
        }

        //设置校验位
        switch (parity)
        {
            /*无奇偶校验位*/
            case ParityNone:
            case 'N':
                options.c_cflag &= ~PARENB; // PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~INPCK;  // INPCK：使奇偶校验起作用
                break;
            /*设置奇校验*/
            case ParityOdd:
                options.c_cflag |= PARENB; // PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag |= PARODD; // PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;  // INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP; // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
            /*设置偶校验*/
            case ParityEven:
                options.c_cflag |= PARENB;  // PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~PARODD; // PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;   // INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP; // ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
                /*设为空格,即停止位为2位*/
            case ParitySpace:
                options.c_cflag &= ~PARENB; // PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~CSTOPB; // CSTOPB：使用两位停止位
                break;
            default:
                fprintf(stderr, "Unkown parity!\n");
                return -1;
        }

        //设置数据位
        switch (dataBits)
        {
            case DataBits5:
                options.c_cflag &= ~CSIZE; //屏蔽其它标志位
                options.c_cflag |= CS5;
                break;
            case DataBits6:
                options.c_cflag &= ~CSIZE; //屏蔽其它标志位
                options.c_cflag |= CS6;
                break;
            case DataBits7:
                options.c_cflag &= ~CSIZE; //屏蔽其它标志位
                options.c_cflag |= CS7;
                break;
            case DataBits8:
                options.c_cflag &= ~CSIZE; //屏蔽其它标志位
                options.c_cflag |= CS8;
                break;
            default:
                fprintf(stderr, "Unkown bits!\n");
                return -1;
        }

        //停止位
        switch (stopbits)
        {
            case StopOne:
                options.c_cflag &= ~CSTOPB; // CSTOPB：使用两位停止位
                break;
            case StopOneAndHalf:
                fprintf(stderr, "POSIX does not support 1.5 stop bits!\n");
                return -1;
            case StopTwo:
                options.c_cflag |= CSTOPB; // CSTOPB：使用两位停止位
                break;
            default:
                fprintf(stderr, "Unkown stop!\n");
                return -1;
        }

        //控制模式
        options.c_cflag |= CLOCAL; //保证程序不占用串口
        options.c_cflag |= CREAD;  //保证程序可以从串口中读取数据

        //流控制
        switch (flowControl)
        {
            case FlowNone: ///< No flow control 无流控制
                options.c_cflag &= ~CRTSCTS;
                break;
            case FlowHardware: ///< Hardware(RTS / CTS) flow control 硬件流控制
                options.c_cflag |= CRTSCTS;
                break;
            case FlowSoftware: ///< Software(XON / XOFF) flow control 软件流控制
                options.c_cflag |= IXON | IXOFF | IXANY;
                break;
            default:
                fprintf(stderr, "Unkown c_flow!\n");
                return -1;
        }

        //设置输出模式为原始输出
        options.c_oflag &= ~OPOST; // OPOST：若设置则按定义的输出处理，否则所有c_oflag失效

        //设置本地模式为原始模式
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        /*
        *ICANON：允许规范模式进行输入处理
        *ECHO：允许输入字符的本地回显
        *ECHOE：在接收EPASE时执行Backspace,Space,Backspace组合
        *ISIG：允许信号
        */

        options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        /*
        *BRKINT：如果设置了IGNBRK，BREAK键输入将被忽略
        *ICRNL：将输入的回车转化成换行（如果IGNCR未设置的情况下）(0x0d => 0x0a)
        *INPCK：允许输入奇偶校验
        *ISTRIP：去除字符的第8个比特
        *IXON：允许输出时对XON/XOFF流进行控制 (0x11 0x13)
        */

        //设置等待时间和最小接受字符
        options.c_cc[VTIME] = 0; //可以在select中设置
        options.c_cc[VMIN] = 1;  //最少读取一个字符

        //如果发生数据溢出，只接受数据，但是不进行读操作
        tcflush(fd, TCIFLUSH);

        //激活配置
        if (tcsetattr(fd, TCSANOW, &options) < 0)
        {
            perror("tcsetattr failed");
            return -1;
        }

        return 0;
    }

    //打开串口 
    bool Nvilidar_Serial::serialOpen()
    {
        bool bRet = false;

        fd = open(m_portName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY); //非阻塞

        if (fd != -1)
        {
            // if(fcntl(fd,F_SETFL,FNDELAY) >= 0)//非阻塞，覆盖前面open的属性
            if (fcntl(fd, F_SETFL, 0) >= 0) // 阻塞，即使前面在open串口设备时设置的是非阻塞的，这里设为阻塞后，以此为准
            {
                // set param
                if (serialSetpara(fd, m_baudRate, m_parity, m_dataBits, m_stopbits, m_flowControl) == -1)
                {
                    fprintf(stderr, "uart set failed!\n");
                    // exit(EXIT_FAILURE);
                    bRet = false;  
                }
            }
            else
            {
                bRet = true;
            }
        }
        else 
        {
            // Could not open the port
            char str[256];
            snprintf(str, sizeof(str), "open port error: Unable to open %s", m_portName.c_str());
            perror(str);

            bRet = false;
        }

        if (true == bRet)
        {
            serialClose();
        }     

        return bRet;
    }

    //关闭串口
    void Nvilidar_Serial::serialClose()
    {
        if (isSerialOpen())
        {
            close(fd);
            fd = -1;
        }
    }

    //查看串口打开状态  
    bool Nvilidar_Serial::isSerialOpen()
    {
        return fd != -1;
    }

    //读有效字节长度  
    int Nvilidar_Serial::serialReadAvaliable()
    {
        int iRet = -1;
        if (isSerialOpen())
        {
            // read前获取可读的字节数,不区分阻塞和非阻塞
            ioctl(fd, FIONREAD, &iRet);
        }
        else
        {
            iRet = -1;
        }
        return iRet;
    }

    //读数据  
    int  Nvilidar_Serial::serialReadData(const uint8_t *data,int len)
    {
        int iRet = -1;
        if (isSerialOpen())
        {
            iRet = read(fd, (char *)data, len);
        }
        else
        {
            iRet = -1;
        }
        return iRet;
    }

    //写数据  
    int Nvilidar_Serial::serialWriteData(const uint8_t *data, int len)
    {
        int iRet = -1;

        if (isSerialOpen())
        {
            // Write N bytes of BUF to FD.  Return the number written, or -1
            iRet = write(fd, data, len);
        }
        else
        {
            iRet = -1;
        }

        return iRet;
    }

    //串口刷新  
    void Nvilidar_Serial::serialFlush()
    {
        //清空读写 
        if (isSerialOpen())
        {
            tcflush(fd,TCIOFLUSH);
        }
    }

    //波特率转换 
    int Nvilidar_Serial::rate2UnixBaud(int baudrate)
    {
        // https://jim.sh/ftx/files/linux-custom-baudrate.c

        #define B(x) \
            case x:  \
                return B##x

        switch (baudrate)
        {
            #ifdef B50
                    B(50);
            #endif
            #ifdef B75
                    B(75);
            #endif
            #ifdef B110
                    B(110);
            #endif
            #ifdef B134
                    B(134);
            #endif
            #ifdef B150
                    B(150);
            #endif
            #ifdef B200
                    B(200);
            #endif
            #ifdef B300
                    B(300);
            #endif
            #ifdef B600
                    B(600);
            #endif
            #ifdef B1200
                    B(1200);
            #endif
            #ifdef B1800
                    B(1800);
            #endif
            #ifdef B2400
                    B(2400);
            #endif
            #ifdef B4800
                    B(4800);
            #endif
            #ifdef B9600
                    B(9600);
            #endif
            #ifdef B19200
                    B(19200);
            #endif
            #ifdef B38400
                    B(38400);
            #endif
            #ifdef B57600
                    B(57600);
            #endif
            #ifdef B115200
                    B(115200);
            #endif
            #ifdef B230400
                    B(230400);
            #endif
            #ifdef B460800
                    B(460800);
            #endif
            #ifdef B500000
                    B(500000);
            #endif
            #ifdef B576000
                    B(576000);
            #endif
            #ifdef B921600
                    B(921600);
            #endif
            #ifdef B1000000
                    B(1000000);
            #endif
            #ifdef B1152000
                    B(1152000);
            #endif
            #ifdef B1500000
                    B(1500000);
            #endif
            #ifdef B2000000
                    B(2000000);
            #endif
            #ifdef B2500000
                    B(2500000);
            #endif
            #ifdef B3000000
                    B(3000000);
            #endif
            #ifdef B3500000
                    B(3500000);
            #endif
            #ifdef B4000000
                    B(4000000);
            #endif

            default:
                return 0;
        }

        #undef B
    }
}

#endif