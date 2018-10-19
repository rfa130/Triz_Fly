#ifndef _ANO_DT_H_
#define	_ANO_DT_H_

#include "include.h"

//================系统===================
#define HW_TYPE	05
#define HW_VER	1
#define SOFT_VER 32
#define BL_VER	0
#define PT_VER	400

#define PAR_PID_1_P		1
#define PAR_PID_1_I		2
#define PAR_PID_1_D		3
#define PAR_PID_2_P		4
#define PAR_PID_2_I		5
#define PAR_PID_2_D		6
#define PAR_PID_3_P		7
#define PAR_PID_3_I		8
#define PAR_PID_3_D		9
#define PAR_PID_4_P		10
#define PAR_PID_4_I		11
#define PAR_PID_4_D		12
#define PAR_PID_5_P		13
#define PAR_PID_5_I		14
#define PAR_PID_5_D		15
#define PAR_PID_6_P		16
#define PAR_PID_6_I		17
#define PAR_PID_6_D		18
#define PAR_PID_7_P		19
#define PAR_PID_7_I		20
#define PAR_PID_7_D		21
#define PAR_PID_8_P		22
#define PAR_PID_8_I		23
#define PAR_PID_8_D		24
#define PAR_PID_9_P		25
#define PAR_PID_9_I		26
#define PAR_PID_9_D		27
#define PAR_PID_10_P		28
#define PAR_PID_10_I		29
#define PAR_PID_10_D		30
#define PAR_PID_11_P		31
#define PAR_PID_11_I		32
#define PAR_PID_11_D		33
#define PAR_PID_12_P		34
#define PAR_PID_12_I		35
#define PAR_PID_12_D		36
#define PAR_PID_13_P		37
#define PAR_PID_13_I		38
#define PAR_PID_13_D		39
#define PAR_PID_14_P		40
#define PAR_PID_14_I		41
#define PAR_PID_14_D		42
#define PAR_PID_15_P		43
#define PAR_PID_15_I		44
#define PAR_PID_15_D		45
#define PAR_PID_16_P		46
#define PAR_PID_16_I		47
#define PAR_PID_16_D		48
#define PAR_PID_17_P		49
#define PAR_PID_17_I		50
#define PAR_PID_17_D		51
#define PAR_PID_18_P		52
#define PAR_PID_18_I		53
#define PAR_PID_18_D		54

#define PAR_RCINMODE		61
#define PAR_UNLOCKPWM		62
#define PAR_LVWARN			63
#define PAR_LVRETN			64
#define PAR_LVDOWN			65
#define PAR_CENPOSX			66
#define PAR_CENPOSY			67
#define PAR_CENPOSZ			68
#define PAR_TAKEOFFHIGH		69

#if (ANO_DT_UART_PORT_SELECT == 1)
void UART1_IntHandler(void);
#elif (ANO_DT_UART_PORT_SELECT == 2)
void URT2_IntHandler(void);
#elif (ANO_DT_UART_PORT_SELECT == 3)
void UART3_IntHandler(void);
#elif (ANO_DT_UART_PORT_SELECT == 4)
void UART4_IntHandler(void);
#endif


typedef struct
{
		uint8_t msg_id;
		uint8_t msg_data;
		uint8_t send_check;
		uint8_t send_version;
		uint8_t send_status;
		uint8_t send_senser;
		uint8_t send_senser2;
		uint8_t send_rcdata;
		uint8_t send_offset;
		uint8_t send_motopwm;
		uint8_t send_power;
		uint8_t send_user;
		uint8_t send_speed;
		uint8_t send_location;
		uint8_t send_vef;
		uint16_t send_parame;
		uint16_t paraToSend;
} dt_flag_t;

extern int32_t ParValList[100];
extern dt_flag_t f;

extern void Bluetooth_Init(void);
void ANO_DT_Data_Receive_Anl_Task(void);
void ANO_DT_Send_VER(void);
void ANO_DT_Data_Exchange(void);
void ANO_DT_Data_Receive_Prepare(u8 data);
void ANO_DT_Data_Receive_Anl(u8 *data_buf,u8 num);
void ANO_DT_Send_Version(u8 hardware_type, u16 hardware_ver,u16 software_ver,u16 protocol_ver,u16 bootloader_ver);
void ANO_DT_Send_Status(float angle_rol, float angle_pit, float angle_yaw, s32 alt, u8 fly_model, u8 armed);
void ANO_DT_Send_Senser(s16 a_x,s16 a_y,s16 a_z,s16 g_x,s16 g_y,s16 g_z,s16 m_x,s16 m_y,s16 m_z);
void ANO_DT_Send_Senser2(s32 bar_alt,s32 csb_alt);
void ANO_DT_Send_RCData(u16 thr,u16 yaw,u16 rol,u16 pit,u16 aux1,u16 aux2,u16 aux3,u16 aux4,u16 aux5,u16 aux6);
void ANO_DT_Send_Power(u16 votage, u16 current);
void ANO_DT_Send_MotoPWM(u16 m_1,u16 m_2,u16 m_3,u16 m_4,u16 m_5,u16 m_6,u16 m_7,u16 m_8);
void ANO_DT_Send_PID(u8 group,float p1_p,float p1_i,float p1_d,float p2_p,float p2_i,float p2_d,float p3_p,float p3_i,float p3_d);
void ANO_DT_Send_User(void);
void ANO_DT_Send_Speed(float,float,float);
void ANO_DT_Send_Location(u8 state,u8 sat_num,s32 lon,s32 lat,float back_home_angle);
void ANO_DT_SendCenterPos(float x,float y,float z);
void ANO_DT_SendParame(u16 num);
void ANO_DT_GetParame(u16 num,s32 data);
void ANO_DT_ParListToParUsed(void);
void ANO_DT_ParUsedToParList(void);
void ANO_DT_SendCmd(u8 dest, u8 cmd1, u16 cmd2);
void ANO_DT_SendString(char *str, u8 len);

#endif

