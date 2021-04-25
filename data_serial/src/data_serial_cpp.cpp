#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <cstdlib>
#include <data_serial/data_protocol.h>
#include <string.h>

/*------------------------------------------------------------------
                             宏定义
  -----------------------------------------------------------------*/
//数据协议宏定义
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)	  ) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
#define DP_REQ_COMMAND_req_car_data   0x01
#define DP_REQ_COMMAND_req_version    0x02

/*------------------------------------------------------------------
                             函数声明
  -----------------------------------------------------------------*/
void DP_Data_Receive_Parsed(uint8_t *data_buf, uint8_t num);
void DP_Data_Receive_Prepare_Parsed(uint8_t data);
void DP_Send_Data(uint8_t *dataToSend, uint8_t length);

/*------------------------------------------------------------------
                             数据定义
  -----------------------------------------------------------------*/
//定义结构体
typedef struct{
	uint8_t req_command;
	int16_t speed;
	int16_t dir;
}car_ctl_t;

//定义变量
car_ctl_t car_ctl;							//定义控制信息存储结构体
serial::Serial data_serial_port;            //创建ROS串口通讯
uint8_t data_to_send[50];					//串口数据发送





/*------------------------------------------------------------------------
					   《数据协议》串口数据发送函数
------------------------------------------------------------------------*/
void DP_Send_Data(uint8_t *dataToSend , uint8_t length)
{
		data_serial_port.write(dataToSend, length);
}

/*------------------------------------------------------------------------
							《数据协议》
				根据数据协议进行---数据预解析---进行数据保存
				   串口每接收到一字节数据，则调用函数一次
							 数据会自行解析
------------------------------------------------------------------------*/
void DP_Data_Receive_Prepare_Parsed(uint8_t data)
{
	static uint8_t RxBuffer[50];
	static uint8_t _data_len = 0, _data_cnt = 0;
	static uint8_t state = 0;
	 
	if(state==0 && data==0xAA)                 	       //AA AA         RxBuffer[0]=0xAA;
	{
		state=1;
		RxBuffer[0] = data;
	}
	else if(state == 1 && data == 0xAA)                //              RxBuffer[0]=0xAF;
	{
		state = 2;
		RxBuffer[1] = data;
	}
	else if(state == 2 && data < 0XF1)                 // 范围在0~128  RxBuffer[3]=功能字             
	{
		state = 3;
		RxBuffer[2] = data;
	}
	else if(state == 3&& data < 50)                    //范围在0~50    RxBuffer[3]=数据长度
	{
		state = 4;
		RxBuffer[3] = data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state == 4 && _data_len > 0)               //  RxBuffer[4]开始保存数据  RxBuffer[]=数据，长度决定保存数据的多少 
	{
		_data_len--;
		RxBuffer[4+_data_cnt++] = data;
		if(_data_len == 0)
			state = 5;
	}
	else if(state == 5)                                //  检验  放在数据帧后面
	{
		state = 0;
		RxBuffer[4+_data_cnt] = data;
		DP_Data_Receive_Parsed(RxBuffer,_data_cnt+5);  //一帧数据保存完毕，进入函数进行 数据的保存处理
	}
	else
	{
		state = 0;
	}
}

/*------------------------------------------------------------------------
					     《数据协议》数据解析函数
------------------------------------------------------------------------*/
void DP_Data_Receive_Parsed(uint8_t *data_buf, uint8_t num)
{
	uint8_t sum = 0;
	uint8_t i = 0;
	uint8_t flag;
	
	//求和校验
	for(i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	
	if(!(sum == *(data_buf+num-1)))		return;					//判断sum
	if(!(*(data_buf) == 0xAA && *(data_buf+1) == 0xAA))		return;		//判断帧头
	
	if(*(data_buf+2) == 0X01)
	{
		car_ctl.req_command = *(data_buf+4);
		//if(*(data_buf+4)==0X01);

	}
	if(*(data_buf+2) == 0X02)        //判断数组的第三个数据   功能字
	{
		car_ctl.speed = (int16_t)(*(data_buf+4)<<8)|*(data_buf+5);
		car_ctl.dir	  = (int16_t)(*(data_buf+6)<<8)|*(data_buf+7);
	}
	
}

/*------------------------------------------------------------------------
							《数据协议》
							数据发送函数
		                根据协议规则来发送数据
							包含求和校验
------------------------------------------------------------------------*/

void DP_Send_RCData(uint16_t thr, uint16_t yaw, uint16_t rol, uint16_t pit, uint16_t aux1, 
						uint16_t aux2, uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6)
{
	uint8_t _cnt = 0;
	uint8_t i;
	uint8_t sum = 0;
	
	//帧头
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAA;
	//功能字
	data_to_send[_cnt++] =  0x03;
	//数据长度，先预留为0，后计算
	data_to_send[_cnt++] = 0;
	
	//数据内容
	data_to_send[_cnt++] = BYTE1(thr);
	data_to_send[_cnt++] = BYTE0(thr);
	data_to_send[_cnt++] = BYTE1(yaw);
	data_to_send[_cnt++] = BYTE0(yaw);
	data_to_send[_cnt++] = BYTE1(rol);
	data_to_send[_cnt++] = BYTE0(rol);
	data_to_send[_cnt++] = BYTE1(pit);
	data_to_send[_cnt++] = BYTE0(pit);
	data_to_send[_cnt++] = BYTE1(aux1);
	data_to_send[_cnt++] = BYTE0(aux1);
	data_to_send[_cnt++] = BYTE1(aux2);
	data_to_send[_cnt++] = BYTE0(aux2);
	data_to_send[_cnt++] = BYTE1(aux3);
	data_to_send[_cnt++] = BYTE0(aux3);
	data_to_send[_cnt++] = BYTE1(aux4);
	data_to_send[_cnt++] = BYTE0(aux4);
	data_to_send[_cnt++] = BYTE1(aux5);
	data_to_send[_cnt++] = BYTE0(aux5);
	data_to_send[_cnt++] = BYTE1(aux6);
	data_to_send[_cnt++] = BYTE0(aux6);
	
	//计算数据长度
	data_to_send[3] = _cnt-4;
	
	//求和校验
	for(i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	
	//存放求和校验值
	data_to_send[_cnt++] = sum;
	
	//数据发送(串口发送)
	DP_Send_Data(data_to_send, _cnt);

}
/*------------------------------------------------------------------------
							《数据协议》
							数据发送函数
		                	发送speed dir
------------------------------------------------------------------------*/

void ROS_PC_Send_Data(int16_t speed, int16_t dir)
{
	uint8_t _cnt = 0;
	uint8_t i;
	uint8_t sum = 0;
	
	//帧头
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	//功能字
	data_to_send[_cnt++] =  0x02;
	//数据长度，先预留为0，后计算
	data_to_send[_cnt++] = 0;
	
	//数据内容
	data_to_send[_cnt++] = BYTE1(speed);
	data_to_send[_cnt++] = BYTE0(speed);
	data_to_send[_cnt++] = BYTE1(dir);
	data_to_send[_cnt++] = BYTE0(dir);
	
	//计算数据长度
	data_to_send[3] = _cnt-4;
	
	//求和校验
	for(i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	
	//存放求和校验值
	data_to_send[_cnt++] = sum;
	
	//数据发送(串口发送)
	//Serial_Send_Data(data_to_send, _cnt);
    DP_Send_Data(data_to_send, _cnt);
}
void DP_Send_Command(uint8_t command)
{
	uint8_t _cnt = 0;
	uint8_t i;
	uint8_t sum = 0;
	
	//帧头
	data_to_send[_cnt++] = 0xAA;
	data_to_send[_cnt++] = 0xAF;
	//功能字
	data_to_send[_cnt++] =  0x01;
	//数据长度，先预留为0，后计算
	data_to_send[_cnt++] = 0;
	
	//数据内容
	data_to_send[_cnt++] = command;

	//计算数据长度
	data_to_send[3] = _cnt-4;
	
	//求和校验
	for(i = 0; i < _cnt; i++)
		sum += data_to_send[i];
	
	//存放求和校验值
	data_to_send[_cnt++] = sum;
	
	//数据发送(串口发送)
	//Serial_Send_Data(data_to_send, _cnt);
    DP_Send_Data(data_to_send, _cnt);
}




/*------------------------------------------------------------------
                         《ROS》回调函数
  -----------------------------------------------------------------*/
void write_callback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Writing to serial" << msg->data);
    data_serial_port.write(msg->data);  //发送串口

}


/*------------------------------------------------------------------
                              主函数
  -----------------------------------------------------------------*/
int main(int argc, char **argv)
{
    /* code for main function */
    size_t buf_num=0;
	int16_t count=0;
	int16_t test_count=0;
	uint8_t read_data[10];
	
    //初始化串口
    ros::init(argc, argv, "data_serial_port");
    //声明节点句柄
    ros::NodeHandle nh;
    ros::Subscriber write_sub = nh.subscribe("write", 1000, write_callback);
    //发布主题
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);


    try
    {
        //设置串口属性，并打开串口
        data_serial_port.setPort("/dev/ttyUSB0");
        data_serial_port.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        data_serial_port.setTimeout(to);
        data_serial_port.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port");
        return -1;
        /* code for Catch */
    }
    //检测串口是否已经打开，并给出提示信息
    if(data_serial_port.isOpen())
    {
        ROS_INFO_STREAM("Serial Port Initialized");
    }
    else
    {
        return -1;
    }
    //指定循环德频率
    ros::Rate loop_rate(100);

	 /* code for loop body */
	data_serial_port.flushInput();

    while (ros::ok())
    {
        /* code for loop body */
        if(data_serial_port.available())
        {
            //ROS_INFO_STREAM("Reading from serial port\n");
	
            buf_num = data_serial_port.available();
			data_serial_port.read(read_data, buf_num);
			for(int i=0; i<buf_num; i++){
				DP_Data_Receive_Prepare_Parsed(read_data[i]);
			}

			ROS_INFO("car_ctl.speed: [%d]\n " , car_ctl.speed);
			ROS_INFO("car_ctl.dir: [%d]\n " , car_ctl.dir);

			// ROS_INFO("Read: [%x][%x][%x][%x][%x][%x][%x][%x][%x] " , read_data[0],
			// read_data[1],
			// read_data[2],
			// read_data[3],
			// read_data[4],
			// read_data[5],
			// read_data[6],
			// read_data[7],
			// read_data[8]
			//);
			//std_msgs::String result;
			//result.data = data_serial_port.read(buf_num);
            //read_pub.publish(result);    
        }

        //处理ROS信息
        ros::spinOnce();
        loop_rate.sleep();

		count++;
		if( count>2 )
		{
			
			test_count++;
			ROS_PC_Send_Data(500+test_count, 10000-test_count);
			// if(test_count%100 == 0)
			// {
			// 	DP_Send_Command(DP_REQ_COMMAND_req_car_data);
			// }

			if(count>3){
				count = 0;
			}
			
		}
       std::string str("DP_REQ_COMMAND_req_car_data");
	   if(argc > 1){
		if(!str.compare(argv[1]))  //comapare 相等 输出为 0
		{
			DP_Send_Command(DP_REQ_COMMAND_req_car_data);
		}
	   }
		

    }
    
}


/*------------------------------------------------------------------

            	(int argc, char **argv) 参数说明

	argc: 参数个数，默认是为1个参数，输入的参数从2开始
	argv[i]： 对应第i个参数，argv[0]为执行文件的路径

  -----------------------------------------------------------------*/

