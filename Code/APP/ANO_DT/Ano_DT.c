//-----------------------------------------------------------------------------
// ANO_DT.c
//
//  Created on	: 2018-5-19
//      Author	: Divenire
//		version	: V1.1
//		brief	: 匿名上位机驱动V6
//-----------------------------------------------------------------------------
// Attention:
//
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "APP/App_include.h"
#include "BSP/Drv_include.h"
#include "Common/Com_include.h"
/////////////////////////////////////////////////////////////////////////////////////
/* Private variables ---------------------------------------------------------*/

s32 ParValList[100]; //参数列表


dt_flag_t f;              //需要发送数据的标志
uint8_t data_to_send[50]; //发送数据缓存
uint8_t checkdata_to_send, checksum_to_send;

//接收缓存，接收缓存区数量，接收完毕标志
static uint8_t DT_RxBuffer[256], DT_data_cnt = 0, ano_dt_data_ok;

/* Private define ------------------------------------------------------------*/

// 通信UART口定义
#if (ANO_DT_UART_PORT_SELECT == 1)
#define ANO_DT_UART_PORT UART1_BASE
#elif (ANO_DT_UART_PORT_SELECT == 2)
#define ANO_DT_UART_PORT UART2_BASE
#elif (ANO_DT_UART_PORT_SELECT == 3)
#define ANO_DT_UART_PORT UART3_BASE
#elif (ANO_DT_UART_PORT_SELECT == 4)
#define ANO_DT_UART_PORT UART4_BASE
#else
#error "Please define ANO_DT_UART_PORT!!!"
#endif

//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp) (*((char *)(&dwTemp) + 0))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

#define MYHWADDR 0x05
#define SWJADDR 0xAF

#define PARNUM 100

/* Private function prototypes -----------------------------------------------*/
//发送数据
static void ANO_DT_Send_Data(uint8_t *dataToSend, uint8_t length);
//根据协议的格式进行协议预解析
static void ANO_DT_Data_Receive_Prepare(uint8_t data);
//将收到的数据进行一次格式性解析
static void ANO_DT_Data_Receive_Anl_Task(void);
//对协议数据进行校验 实现相应功能
static void ANO_DT_Data_Receive_Anl(uint8_t *data_buf, uint8_t num);
//发送数据

static void ANO_DT_Send_Data(uint8_t *dataToSend, uint8_t length)
{
#if (ANO_DT_UART_PORT_SELECT == 1)
    Uart1_putbuff(dataToSend, length); //通过UART1发送
#elif (ANO_DT_UART_PORT_SELECT == 2)
    Uart2_putbuff(dataToSend, length); //通过UART2发送
#elif (ANO_DT_UART_PORT_SELECT == 3)
    Uart3_putbuff(dataToSend, length); //通过UART3发送
#elif (ANO_DT_UART_PORT_SELECT == 4)
    Uart4_putbuff(dataToSend, length); //通过UART4发送
#endif
}
//------------------------------------------------------------------------------
// Interrupt Function
// 中断函数
//------------------------------------------------------------------------------
#if (ANO_DT_UART_PORT_SELECT == 1)
void UART1_IntHandler()
#elif (ANO_DT_UART_PORT_SELECT == 2)
void UART2_IntHandler()
#elif (ANO_DT_UART_PORT_SELECT == 3)
void UART3_IntHandler()
#elif (ANO_DT_UART_PORT_SELECT == 4)
void UART4_IntHandler(void)
#endif
{
    uint32_t ui32Status;
    uint8_t temp;

    ui32Status = UARTIntStatus(ANO_DT_UART_PORT, true);
    UARTIntClear(ANO_DT_UART_PORT, ui32Status);
    while (UARTCharsAvail(ANO_DT_UART_PORT))
    {
        temp = UARTCharGetNonBlocking(ANO_DT_UART_PORT);
        ANO_DT_Data_Receive_Prepare(temp);
    }
}

void Bluetooth_Init()
{
#if (ANO_DT_UART_PORT_SELECT == 1)
    Uart1_Init(115200);
#elif (ANO_DT_UART_PORT_SELECT == 2)
    Uart2_Init(115200);
#elif (ANO_DT_UART_PORT_SELECT == 3)
    Uart3_Init(115200);
#elif (ANO_DT_UART_PORT_SELECT == 4)
    Uart4_Init(115200);
#endif
}

/////////////////////////////////////////////////////////////////////////////////////
//Data_Exchange函数处理各种数据发送请求，比如想实现每5ms发送一次传感器数据至上位机，即在此函数内实现
//此函数应由用户每1ms调用一次
extern float ultra_dis_lpf;
void ANO_DT_Data_Exchange(void)
{
	static u8 cnt = 0;
	static u8 senser_cnt 	= 10;
	static u8 senser2_cnt = 50;
	static u8 user_cnt 	  = 10;
	static u8 status_cnt 	= 15;
	static u8 rcdata_cnt 	= 20;
	static u8 motopwm_cnt	= 20;
	static u8 power_cnt		=	50;
	static u8 speed_cnt   = 50;
	static u8 location_cnt   = 200;

		
	if((cnt % senser_cnt) == (senser_cnt-1))
		f.send_senser = 1;

	if((cnt % senser2_cnt) == (senser2_cnt-1))
		f.send_senser2 = 1;	

	if((cnt % user_cnt) == (user_cnt-2))
		f.send_user = 1;
	
	if((cnt % status_cnt) == (status_cnt-1))
		f.send_status = 1;	
	
	if((cnt % rcdata_cnt) == (rcdata_cnt-1))
		f.send_rcdata = 1;	
	
	if((cnt % motopwm_cnt) == (motopwm_cnt-2))
		f.send_motopwm = 1;	
	
	if((cnt % power_cnt) == (power_cnt-2))
		f.send_power = 1;		
	
	if((cnt % speed_cnt) == (speed_cnt-3))
		f.send_speed = 1;		
	
	if((cnt % location_cnt) == (location_cnt-3))
	{
		f.send_location += 1;		
	}
	
	if(++cnt>200) cnt = 0;
/////////////////////////////////////////////////////////////////////////////////////
	if(f.send_version)
	{
		f.send_version = 0;
		ANO_DT_Send_Version(4,300,100,400,0);
	}
	else if(f.paraToSend < 0xffff)
	{
		ANO_DT_SendParame(f.paraToSend);
		f.paraToSend = 0xffff;
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_status)
	{
		f.send_status = 0;
		ANO_DT_Send_Status(imu_data.rol,imu_data.pit,imu_data.yaw,wcz_hei_fus.out,0,flag.fly_ready);	
	}	
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_speed)
	{
		f.send_speed = 0;
		ANO_DT_Send_Speed(loc_ctrl_1.fb[Y],loc_ctrl_1.fb[X],loc_ctrl_1.fb[Z]);
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_user)
	{
		f.send_user = 0;
		ANO_DT_Send_User();
	}
///////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser)
	{
		f.send_senser = 0;
		ANO_DT_Send_Senser(sensor.Acc[X],sensor.Acc[Y],sensor.Acc[Z],sensor.Gyro[X],sensor.Gyro[Y],sensor.Gyro[Z],mag.val[X],mag.val[Y],mag.val[Z]);
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_senser2)
	{
		f.send_senser2 = 0;
		ANO_DT_Send_Senser2(baro_height,ref_son_height);//原始数据
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_rcdata)
	{
		f.send_rcdata = 0;
		s16 CH_GCS[CH_NUM];
		
		for(u8 i=0;i<CH_NUM;i++)
		{
			if(Rc_Sbus_In[i]!=0 || Rc_Sbus_In[i] !=0 )//该通道有值
			{
				CH_GCS[i] = CH_N[i] + 1500;
			}
			else
			{
				CH_GCS[i] = 0;
			}
		}
		ANO_DT_Send_RCData(CH_GCS[1],CH_GCS[3],CH_GCS[2],CH_GCS[0],CH_GCS[4],CH_GCS[5],CH_GCS[6],CH_GCS[7],0,0);
	}	
/////////////////////////////////////////////////////////////////////////////////////	
	else if(f.send_motopwm)
	{
		f.send_motopwm = 0;
#if MOTORSNUM == 8
		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],motor[4],motor[5],motor[6],motor[7]);
#elif MOTORSNUM == 6
		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],motor[4],motor[5],0,0);
#elif MOTORSNUM == 4
		ANO_DT_Send_MotoPWM(motor[0],motor[1],motor[2],motor[3],0,0,0,0);
#else
		
#endif
	}	
/////////////////////////////////////////////////////////////////////////////////////
	else if(f.send_power)
	{
		f.send_power = 0;
		ANO_DT_Send_Power(Plane_Votage*100,0);
	}
	else if(f.send_location == 2)
	{
		
		f.send_location = 0;
		ANO_DT_Send_Location(0,0,0 *10000000,0  *10000000,0);
		
	}
	else if(f.send_vef)
	{
		ANO_DT_Send_VER();
		f.send_vef = 0;
	}
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
	ANO_DT_Data_Receive_Anl_Task();
/////////////////////////////////////////////////////////////////////////////////////				
/////////////////////////////////////////////////////////////////////////////////////
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Prepare函数是协议预解析，根据协议的格式，将收到的数据进行一次格式性解析，格式正确的话再进行数据解析
//移植时，此函数应由用户根据自身使用的通信方式自行调用，比如串口每收到一字节数据，则调用此函数一次
//此函数解析出符合格式的数据帧后，会自行调用数据解析函数

void ANO_DT_Data_Receive_Anl_Task()
{
    if (ano_dt_data_ok)
    {
        ANO_DT_Data_Receive_Anl(DT_RxBuffer, DT_data_cnt + 6);
        ano_dt_data_ok = 0;
    }
}

void ANO_DT_Data_Receive_Prepare(uint8_t data)
{
    static uint8_t _data_len = 0;
    static uint8_t state = 0;

    if (state == 0 && data == 0xAA) //帧头0xAA
    {
        state = 1;
        DT_RxBuffer[0] = data;
    }
    else if (state == 1 && data == 0xAF) //数据源，0xAF表示数据来自上位机
    {
        state = 2;
        DT_RxBuffer[1] = data;
    }
    else if (state == 2) //数据目的地
    {
        state = 3;
        DT_RxBuffer[2] = data;
    }
    else if (state == 3) //功能字
    {
        state = 4;
        DT_RxBuffer[3] = data;
    }
    else if (state == 4) //数据长度
    {
        state = 5;
        DT_RxBuffer[4] = data;
        _data_len = data;
        DT_data_cnt = 0;
    }
    else if (state == 5 && _data_len > 0)
    {
        _data_len--;
        DT_RxBuffer[5 + DT_data_cnt++] = data;
        if (_data_len == 0)
            state = 6;
    }
    else if (state == 6)
    {
        state = 0;
        DT_RxBuffer[5 + DT_data_cnt] = data;
        ano_dt_data_ok = 1; //ANO_DT_Data_Receive_Anl(DT_RxBuffer,DT_data_cnt+5);
    }
    else
        state = 0;
}
/////////////////////////////////////////////////////////////////////////////////////
//Data_Receive_Anl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//校验通过后对数据进行解析，实现相应功能
//此函数可以不用用户自行调用，由函数Data_Receive_Prepare自动调用
u16 flash_save_en_cnt = 0;
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	u8 sum = 0;
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0xAF))		return;		//判断帧头
	
	if(*(data_buf+2)==MYHWADDR)
	{
		if(*(data_buf+3)==0XE0)			//命令E0
		{
			switch(*(data_buf+5))		//CMD1
			{
				case 0x01:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x01)
						sensor.acc_CALIBRATE = 1;
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x02)
						sensor.gyr_CALIBRATE = 1;
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0x04)
						mag.mag_CALIBRATE = 1;
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xB0)//读取版本信息
						f.send_version = 1;
					break;
				case 0x02:
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAA)
					{
						PID_Rest();
						All_PID_Init();
						data_save();
					}
					if(*(data_buf+6)==0x00 && *(data_buf+7)==0xAB)
					{
						PID_Rest();
						All_PID_Init();
						Parame_Reset();
						data_save();
					}
					break;
				case 0xE1:
					f.paraToSend = (u16)(*(data_buf+6)<<8)|*(data_buf+7);	//读取参数
					break;
				default:
					break;
			}
			ANO_DT_SendCmd(SWJADDR,*(data_buf+5),(u16)(*(data_buf+6)<<8)|*(data_buf+7));
		}
		else if(*(data_buf+3)==0XE1)	//设置参数
		{
			u16 _paraNum = (u16)(*(data_buf+5)<<8)|*(data_buf+6);
			s32 _paraVal = (s32)(((*(data_buf+7))<<24) + ((*(data_buf+8))<<16) + ((*(data_buf+9))<<8) + (*(data_buf+10)));
			ANO_DT_GetParame(_paraNum,_paraVal);
		}
	}
}

void ANO_DT_SendCmd(uint8_t dest, uint8_t cmd1, uint16_t cmd2)
{
    uint8_t _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = dest;
    data_to_send[_cnt++] = 0xE0;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = cmd1;
    data_to_send[_cnt++] = BYTE1(cmd2);
    data_to_send[_cnt++] = BYTE0(cmd2);

    data_to_send[4] = _cnt - 5;

    uint8_t sum = 0, i;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_SendParame(uint16_t num)
{
    uint8_t _cnt = 0;
    int32_t data;
    if (num > PARNUM)
        return;
    ANO_DT_ParUsedToParList();
    data = ParValList[num];
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x09;
    data_to_send[_cnt++] = 0;
    data_to_send[_cnt++] = BYTE1(num);
    data_to_send[_cnt++] = BYTE0(num);
    data_to_send[_cnt++] = BYTE3(data);
    data_to_send[_cnt++] = BYTE2(data);
    data_to_send[_cnt++] = BYTE1(data);
    data_to_send[_cnt++] = BYTE0(data);

    data_to_send[4] = _cnt - 5;

    uint8_t i, sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_GetParame(uint16_t num, int32_t data)
{
    if (num > PARNUM)
        return;
    ParValList[num] = data;
    ANO_DT_ParListToParUsed();
    f.paraToSend = num; //将接收到的参数发回上位机进行双向验证
}
void ANO_DT_ParListToParUsed(void)
{
	Triz_Parame.set.pid_att_1level[ROL][KP] = (float) ParValList[PAR_PID_1_P] / 1000;
	Triz_Parame.set.pid_att_1level[ROL][KI] = (float) ParValList[PAR_PID_1_I] / 1000;
	Triz_Parame.set.pid_att_1level[ROL][KD] = (float) ParValList[PAR_PID_1_D] / 1000;
	Triz_Parame.set.pid_att_1level[PIT][KP] = (float) ParValList[PAR_PID_2_P] / 1000;
	Triz_Parame.set.pid_att_1level[PIT][KI] = (float) ParValList[PAR_PID_2_I] / 1000;
	Triz_Parame.set.pid_att_1level[PIT][KD] = (float) ParValList[PAR_PID_2_D] / 1000;
	Triz_Parame.set.pid_att_1level[YAW][KP] = (float) ParValList[PAR_PID_3_P] / 1000;
	Triz_Parame.set.pid_att_1level[YAW][KI] = (float) ParValList[PAR_PID_3_I] / 1000;
	Triz_Parame.set.pid_att_1level[YAW][KD] = (float) ParValList[PAR_PID_3_D] / 1000;
	
	Triz_Parame.set.pid_att_2level[ROL][KP] = (float) ParValList[PAR_PID_4_P] / 1000;
	Triz_Parame.set.pid_att_2level[ROL][KI] = (float) ParValList[PAR_PID_4_I] / 1000;
	Triz_Parame.set.pid_att_2level[ROL][KD] = (float) ParValList[PAR_PID_4_D] / 1000;
	Triz_Parame.set.pid_att_2level[PIT][KP] = (float) ParValList[PAR_PID_5_P] / 1000;
	Triz_Parame.set.pid_att_2level[PIT][KI] = (float) ParValList[PAR_PID_5_I] / 1000;
	Triz_Parame.set.pid_att_2level[PIT][KD] = (float) ParValList[PAR_PID_5_D] / 1000;
	Triz_Parame.set.pid_att_2level[YAW][KP] = (float) ParValList[PAR_PID_6_P] / 1000;
	Triz_Parame.set.pid_att_2level[YAW][KI] = (float) ParValList[PAR_PID_6_I] / 1000;
	Triz_Parame.set.pid_att_2level[YAW][KD] = (float) ParValList[PAR_PID_6_D] / 1000;
	
	Triz_Parame.set.pid_alt_1level[KP] = (float) ParValList[PAR_PID_7_P] / 1000;
	Triz_Parame.set.pid_alt_1level[KI] = (float) ParValList[PAR_PID_7_I] / 1000;
	Triz_Parame.set.pid_alt_1level[KD] = (float) ParValList[PAR_PID_7_D] / 1000;
	Triz_Parame.set.pid_alt_2level[KP] = (float) ParValList[PAR_PID_8_P] / 1000;
	Triz_Parame.set.pid_alt_2level[KI] = (float) ParValList[PAR_PID_8_I] / 1000;
	Triz_Parame.set.pid_alt_2level[KD] = (float) ParValList[PAR_PID_8_D] / 1000;
	
	Triz_Parame.set.pid_loc_1level[KP] = (float) ParValList[PAR_PID_9_P] / 1000; 
	Triz_Parame.set.pid_loc_1level[KI] = (float) ParValList[PAR_PID_9_I] / 1000; 
	Triz_Parame.set.pid_loc_1level[KD] = (float) ParValList[PAR_PID_9_D] / 1000; 
	Triz_Parame.set.pid_loc_2level[KP] = (float) ParValList[PAR_PID_10_P] / 1000; 
	Triz_Parame.set.pid_loc_2level[KI] = (float) ParValList[PAR_PID_10_I] / 1000; 
	Triz_Parame.set.pid_loc_2level[KD] = (float) ParValList[PAR_PID_10_D] / 1000; 
	
	if(ParValList[PAR_RCINMODE] == 0)
		Triz_Parame.set.pwmInMode = PWM;
	else
		Triz_Parame.set.pwmInMode = PPM;
	
	Triz_Parame.set.warn_power_voltage = (float) ParValList[PAR_LVWARN] / 10;
	Triz_Parame.set.return_home_power_voltage = (float) ParValList[PAR_LVRETN] / 10;
	Triz_Parame.set.lowest_power_voltage = (float) ParValList[PAR_LVDOWN] / 10;
	
	Triz_Parame.set.auto_take_off_height = ParValList[PAR_TAKEOFFHIGH];//cm
}
void ANO_DT_ParUsedToParList(void)
{
	ParValList[PAR_PID_1_P] = Triz_Parame.set.pid_att_1level[ROL][KP] * 1000;
	ParValList[PAR_PID_1_I] = Triz_Parame.set.pid_att_1level[ROL][KI] * 1000;
	ParValList[PAR_PID_1_D] = Triz_Parame.set.pid_att_1level[ROL][KD] * 1000;
	ParValList[PAR_PID_2_P] = Triz_Parame.set.pid_att_1level[PIT][KP] * 1000;
	ParValList[PAR_PID_2_I] = Triz_Parame.set.pid_att_1level[PIT][KI] * 1000;
	ParValList[PAR_PID_2_D] = Triz_Parame.set.pid_att_1level[PIT][KD] * 1000;
	ParValList[PAR_PID_3_P] = Triz_Parame.set.pid_att_1level[YAW][KP] * 1000;
	ParValList[PAR_PID_3_I] = Triz_Parame.set.pid_att_1level[YAW][KI] * 1000;
	ParValList[PAR_PID_3_D] = Triz_Parame.set.pid_att_1level[YAW][KD] * 1000;
	
	ParValList[PAR_PID_4_P] = Triz_Parame.set.pid_att_2level[ROL][KP] * 1000;
	ParValList[PAR_PID_4_I] = Triz_Parame.set.pid_att_2level[ROL][KI] * 1000;
	ParValList[PAR_PID_4_D] = Triz_Parame.set.pid_att_2level[ROL][KD] * 1000;
	ParValList[PAR_PID_5_P] = Triz_Parame.set.pid_att_2level[PIT][KP] * 1000;
	ParValList[PAR_PID_5_I] = Triz_Parame.set.pid_att_2level[PIT][KI] * 1000;
	ParValList[PAR_PID_5_D] = Triz_Parame.set.pid_att_2level[PIT][KD] * 1000;
	ParValList[PAR_PID_6_P] = Triz_Parame.set.pid_att_2level[YAW][KP] * 1000;
	ParValList[PAR_PID_6_I] = Triz_Parame.set.pid_att_2level[YAW][KI] * 1000;
	ParValList[PAR_PID_6_D] = Triz_Parame.set.pid_att_2level[YAW][KD] * 1000;
	
	ParValList[PAR_PID_7_P] = Triz_Parame.set.pid_alt_1level[KP] * 1000;
	ParValList[PAR_PID_7_I] = Triz_Parame.set.pid_alt_1level[KI] * 1000;
	ParValList[PAR_PID_7_D] = Triz_Parame.set.pid_alt_1level[KD] * 1000;
	ParValList[PAR_PID_8_P] = Triz_Parame.set.pid_alt_2level[KP] * 1000;
	ParValList[PAR_PID_8_I] = Triz_Parame.set.pid_alt_2level[KI] * 1000;
	ParValList[PAR_PID_8_D] = Triz_Parame.set.pid_alt_2level[KD] * 1000;
	
	ParValList[PAR_PID_9_P] = Triz_Parame.set.pid_loc_1level[KP] * 1000;
	ParValList[PAR_PID_9_I] = Triz_Parame.set.pid_loc_1level[KI] * 1000;
	ParValList[PAR_PID_9_D] = Triz_Parame.set.pid_loc_1level[KD] * 1000;
	ParValList[PAR_PID_10_P] = Triz_Parame.set.pid_loc_2level[KP] * 1000;
	ParValList[PAR_PID_10_I] = Triz_Parame.set.pid_loc_2level[KI] * 1000;
	ParValList[PAR_PID_10_D] = Triz_Parame.set.pid_loc_2level[KD] * 1000;

	if(Triz_Parame.set.pwmInMode == PWM)
		ParValList[PAR_RCINMODE] = 0;
	else
		ParValList[PAR_RCINMODE] = 1;

	ParValList[PAR_LVWARN] = Triz_Parame.set.warn_power_voltage * 10;
	ParValList[PAR_LVRETN] = Triz_Parame.set.return_home_power_voltage * 10;
	ParValList[PAR_LVDOWN] = Triz_Parame.set.lowest_power_voltage * 10;
	
	ParValList[PAR_TAKEOFFHIGH] = Triz_Parame.set.auto_take_off_height;
}

void ANO_DT_Send_VER(void)
{
	u8 temp[14];
	temp[0] = 0xAA;
	temp[1] = 0xAA;
	temp[2] = 0x00;
	temp[3] = 9;
	temp[4] = HW_TYPE;
	temp[5] = HW_VER/256;
	temp[6] = HW_VER%256;
	temp[7] = SOFT_VER/256;
	temp[8] = SOFT_VER%256;
	temp[9] = PT_VER/256;
	temp[10] = PT_VER%256;
	temp[11] = BL_VER/256;
	temp[12] = BL_VER%256;
	u8 check_sum = 0;
	for(u8 i=0;i<13;i++)
		check_sum += temp[i];
	temp[13] = check_sum;
	
	ANO_DT_Send_Data(temp,14);
}

void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x00;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=hardware_type;
	data_to_send[_cnt++]=BYTE1(hardware_ver);
	data_to_send[_cnt++]=BYTE0(hardware_ver);
	data_to_send[_cnt++]=BYTE1(software_ver);
	data_to_send[_cnt++]=BYTE0(software_ver);
	data_to_send[_cnt++]=BYTE1(protocol_ver);
	data_to_send[_cnt++]=BYTE0(protocol_ver);
	data_to_send[_cnt++]=BYTE1(bootloader_ver);
	data_to_send[_cnt++]=BYTE0(bootloader_ver);
	
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;
	
	ANO_DT_Send_Data(data_to_send, _cnt);
}


void ANO_DT_Send_Speed(float x_s, float y_s, float z_s)
{
    uint8_t _cnt = 0;
    volatile int16_t _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x0B;
    data_to_send[_cnt++] = 0;

    _temp = (int)(0.1f * x_s);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(0.1f * y_s);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(0.1f * z_s);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[4] = _cnt - 5;

    uint8_t i, sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Location(uint8_t state,uint8_t sat_num,int32_t lon,int32_t lat,float back_home_angle)
{
	uint8_t _cnt=0;
	volatile int16_t _temp;
	volatile int32_t _temp2;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x04;
	data_to_send[_cnt++]=0;

	data_to_send[_cnt++]=state;
	data_to_send[_cnt++]=sat_num;

	_temp2 = lon;//经度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	_temp2 = lat;//纬度
	data_to_send[_cnt++]=BYTE3(_temp2);
	data_to_send[_cnt++]=BYTE2(_temp2);
	data_to_send[_cnt++]=BYTE1(_temp2);
	data_to_send[_cnt++]=BYTE0(_temp2);

	_temp = (s16)(100 *back_home_angle);
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);

	data_to_send[4] = _cnt-5;

	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);

}

void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, int32_t alt, uint8_t fly_model, uint8_t armed)
{
    uint8_t _cnt = 0;
    volatile int16_t _temp;
    volatile int32_t _temp2 = alt;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x01;
    data_to_send[_cnt++] = 0;

    _temp = (int)(angle_rol * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_pit * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = (int)(angle_yaw * 100);
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[_cnt++] = BYTE3(_temp2);
    data_to_send[_cnt++] = BYTE2(_temp2);
    data_to_send[_cnt++] = BYTE1(_temp2);
    data_to_send[_cnt++] = BYTE0(_temp2);

    data_to_send[_cnt++] = fly_model;

    data_to_send[_cnt++] = armed;

    data_to_send[4] = _cnt - 5;

    uint8_t i, sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser(int16_t a_x, int16_t a_y, int16_t a_z, int16_t g_x, int16_t g_y, int16_t g_z, int16_t m_x, int16_t m_y, int16_t m_z)
{
    uint8_t _cnt = 0;
    volatile int16_t _temp;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x02;
    data_to_send[_cnt++] = 0;

    _temp = a_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = a_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = g_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = g_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    _temp = m_x;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_y;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    _temp = m_z;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);
    /////////////////////////////////////////
    _temp = 0;
    data_to_send[_cnt++] = BYTE1(_temp);
    data_to_send[_cnt++] = BYTE0(_temp);

    data_to_send[4] = _cnt - 5;

    uint8_t i, sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_Senser2(int32_t bar_alt, int32_t csb_alt)
{
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x07;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE3(bar_alt);
    data_to_send[_cnt++] = BYTE2(bar_alt);
    data_to_send[_cnt++] = BYTE1(bar_alt);
    data_to_send[_cnt++] = BYTE0(bar_alt);

    data_to_send[_cnt++] = BYTE3(csb_alt);
    data_to_send[_cnt++] = BYTE2(csb_alt);
    data_to_send[_cnt++] = BYTE1(csb_alt);
    data_to_send[_cnt++] = BYTE0(csb_alt);

    data_to_send[4] = _cnt - 5;

    uint8_t i, sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];
    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}
void ANO_DT_Send_RCData(uint16_t thr, uint16_t yaw, uint16_t rol, uint16_t pit, uint16_t aux1, uint16_t aux2, uint16_t aux3, uint16_t aux4, uint16_t aux5, uint16_t aux6)
{
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x03;
    data_to_send[_cnt++] = 0;
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

    data_to_send[4] = _cnt - 5;

    uint8_t sum = 0, i;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_Power(uint16_t votage, uint16_t current)
{
	uint8_t _cnt=0;
	uint16_t temp;

	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0x05;
	data_to_send[_cnt++]=0;

	temp = votage;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = current;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);

	data_to_send[4] = _cnt-5;

	uint8_t sum = 0;
	for(uint8_t i=0;i<_cnt;i++)
		sum += data_to_send[i];

	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_MotoPWM(uint16_t m_1, uint16_t m_2, uint16_t m_3, uint16_t m_4, uint16_t m_5, uint16_t m_6, uint16_t m_7, uint16_t m_8)
{
    uint8_t _cnt = 0;

    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0x06;
    data_to_send[_cnt++] = 0;

    data_to_send[_cnt++] = BYTE1(m_1);
    data_to_send[_cnt++] = BYTE0(m_1);
    data_to_send[_cnt++] = BYTE1(m_2);
    data_to_send[_cnt++] = BYTE0(m_2);
    data_to_send[_cnt++] = BYTE1(m_3);
    data_to_send[_cnt++] = BYTE0(m_3);
    data_to_send[_cnt++] = BYTE1(m_4);
    data_to_send[_cnt++] = BYTE0(m_4);
    data_to_send[_cnt++] = BYTE1(m_5);
    data_to_send[_cnt++] = BYTE0(m_5);
    data_to_send[_cnt++] = BYTE1(m_6);
    data_to_send[_cnt++] = BYTE0(m_6);
    data_to_send[_cnt++] = BYTE1(m_7);
    data_to_send[_cnt++] = BYTE0(m_7);
    data_to_send[_cnt++] = BYTE1(m_8);
    data_to_send[_cnt++] = BYTE0(m_8);

    data_to_send[4] = _cnt - 5;

    uint8_t i, sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_SendString(char *str, uint8_t len)
{
    uint8_t i, _cnt = 0;
    data_to_send[_cnt++] = 0xAA;
    data_to_send[_cnt++] = MYHWADDR;
    data_to_send[_cnt++] = SWJADDR;
    data_to_send[_cnt++] = 0xA0;
    data_to_send[_cnt++] = 0;
    for (i = 0; i < len; i++)
        data_to_send[_cnt++] = *(str + i);

    data_to_send[4] = _cnt - 5;

    uint8_t sum = 0;
    for (i = 0; i < _cnt; i++)
        sum += data_to_send[i];

    data_to_send[_cnt++] = sum;

    ANO_DT_Send_Data(data_to_send, _cnt);
}

void ANO_DT_Send_User()
{
	u8 _cnt=0;
	vs16 _temp;
	
	data_to_send[_cnt++]=0xAA; 
	data_to_send[_cnt++]=MYHWADDR;
	data_to_send[_cnt++]=SWJADDR;
	data_to_send[_cnt++]=0xf1; //用户数据
	data_to_send[_cnt++]=0;
////////////////////////////////////////	

extern s32 sensor_val_ref[];
	_temp = (s16)(sensor_val_ref[A_Z]);//          //1
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	

	_temp = (s16)(wcz_acc_fus.out);//         //2
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	_temp = (s16)(wcz_spe_fus.out);//         //3
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	_temp = (s16)(wcz_hei_fus.out);//         //4
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	
	
	_temp = (s16)(wcz_ref_speed);//         //5
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);	

	_temp = (s16)(wcz_ref_height);//         //6
	data_to_send[_cnt++]=BYTE1(_temp);
	data_to_send[_cnt++]=BYTE0(_temp);
	
	// _temp = (s16)100;
	// data_to_send[_cnt++]=BYTE1(_temp);
	// data_to_send[_cnt++]=BYTE0(_temp);	
	
	// _temp = (s16)200;
	// data_to_send[_cnt++]=BYTE1(_temp);
	// data_to_send[_cnt++]=BYTE0(_temp);	

	// _temp = (s16)300;
	// data_to_send[_cnt++]=BYTE1(_temp);
	// data_to_send[_cnt++]=BYTE0(_temp);	

	// _temp = (s16)400;
	// data_to_send[_cnt++]=BYTE1(_temp);
	// data_to_send[_cnt++]=BYTE0(_temp);	
	
	// _temp = (s16)400;
	// data_to_send[_cnt++]=BYTE1(_temp);
	// data_to_send[_cnt++]=BYTE0(_temp);	


//	extern float speed_test[];
//	_temp = (s16)(speed_test[0]);//         //7
//	data_to_send[_cnt++]=BYTE1(_temp);
//	data_to_send[_cnt++]=BYTE0(_temp);
////////////////////////////////////////
	data_to_send[4] = _cnt-5;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;

	ANO_DT_Send_Data(data_to_send, _cnt);
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/
