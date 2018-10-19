/*******************************************
Copyright:Triz
Author:Criss
Date:2018.5.17
********************************************/
#include "BSP\Drv_include.h"

/*DRV_LED结构定义：
State：当前飞机状态
Srate_Old：之前飞机状态
Accurancy：设置灯总共有多少数级亮度
Broghtness[LED_NUM]：设置各个颜色灯的亮度等级
*/
struct DRV_LED LED = {
    0, 0, 20, {0, 0, 0}};
/***********************************
*功能：初始化LED灯所用引脚
*参数：无
************************************/
void Drv_LED_Init(void)
{
    uint8_t i;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, LED_USER);    
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, LED_RED);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, LED_BLUE);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, LED_GREEN);
    for (i = 0; i < LED_NUM; i++)
        LED.Brightness[i] = 1;
    LED.State = 1;
    LED.Accuracy=20;
}
/***********************************
*功能：维持灯Brightness所设置的亮度（在1ms滴答定时器中调用）
*参数：无
************************************/
void Drv_LED_1ms(void)
{
    uint8_t i = 0;
    static uint8_t count[LED_NUM];
    for (i = 0; i < LED_NUM; i++)
    {
        if (count[i] < LED.Brightness[i])
        {
            switch (i)
            {
            case 0:
                LED_USER_ON;
            case 1:
                LED_GREEN_ON;
                break;
            case 2:
                LED_RED_ON;
                break;
            case 3:
                LED_BLUE_ON;
                break;
            }
        }
        else if (count[i] >= LED.Brightness[i])
        {
            switch (i)
            {
            case 0:
                LED_USER_OFF;
            case 1:
                LED_GREEN_OFF;
                break;
            case 2:
                LED_RED_OFF;
                break;
            case 3:
                LED_BLUE_OFF;
                break;
            }
        }
        count[i]++;
        if (count[i] > LED.Accuracy)
            count[i] = 0;
    }
}
/***********************************
*功能：产生一个周期为2T的呼吸灯
*参数：dT_ms:调用该函数的周期
i:选择不同颜色的灯，及：Red,Blue,Green
T:呼吸灯半周期的时间
*返回值：完成一次完整周期，返回1
************************************/
uint8_t Drv_LED_Breath(uint8_t dT_ms, uint8_t i, uint16_t dT)
{
    uint8_t f = 0;
    static uint8_t dir[LED_NUM];
    switch (dir[i])
    {
        //当dir为0时，代表亮度增加
    case 0:
        LED.Brightness[i] += ((double)LED.Accuracy * (double)dT_ms / (double)dT);
        if (LED.Brightness[i] > LED.Accuracy)
        {
            LED.Brightness[i] = LED.Accuracy;
            dir[i] = 1;
        }
        break;
        //当dir为1时，代表亮度增加
    case 1:
        LED.Brightness[i] -= ((double)LED.Accuracy * (double)dT_ms / (double)dT);
        if (LED.Brightness[i] <= 0)
        {
            //LED.Brightness[i] = 0;
            dir[i] = 0;
            f = 1;
        }
        break;
    default:
        dir[i] = 0;
        break;
    }
    return f;
}
static uint16_t ms_count[LED_NUM];
static uint16_t group_count[LED_NUM];
/***********************************
*功能：产生不同亮度，不同闪烁频率的状态灯
*参数：dT_ms:调该用函数的周期
i:选择不同颜色的灯，及：Red_Led,Blue_Led,Green_Led
light:灯的亮度级别（最大亮度级为DRV_LED结构中Accurancy所设置的值）
on_ms:灯亮的时间
off_ms:灯灭的时间
group_num:每一组灯一共亮灭多少次
goutp_T_ms:当一组完成，组与组之间间隔的时间
*返回值：完成一次完整周期，返回1
************************************/

uint8_t Drv_LED_Flash(uint8_t dT_ms, uint8_t i, uint8_t lb, uint8_t group_n, uint16_t on_ms, uint16_t off_ms, uint16_t group_dT_ms)
{
    uint8_t f;
    if(group_count[i] < group_n)   //组数没到
	{
		if(ms_count[i]<on_ms)
		{
			LED.Brightness[i] = lb;
		}
		else if(ms_count[i]<(on_ms+off_ms))
		{
			LED.Brightness[i] = 0;
		}
		if(ms_count[i]>=(on_ms+off_ms))
		{
			group_count[i] ++;
			ms_count[i] = 0;
		}
	}
	else						//进入组间隔
	{
		if(ms_count[i]<group_dT_ms)
		{
			LED.Brightness[i] = 0;
		}
		else
		{
			group_count[i] = 0;
			ms_count[i] = 0;
			f = 1; //流程完成1次
		}
	}
	
	ms_count[i] += (dT_ms);        //计时
	return (f); //0，未完成，1完成

}

// uint8_t Drv_LED_Flash(uint8_t dT_ms, uint8_t i, uint8_t light, uint16_t on_ms, uint16_t off_ms, uint16_t group_num, uint16_t group_T_ms)
// {
//     uint8_t f;
//     if (group_count[i] < group_num) //完成暗灭次数小于每组要求次数
//     {
//         if (ms_count[i] < on_ms) //在亮的时间
//         {
//             LED.Brightness[i] = light;
//         }
//         else if (ms_count[i] < (on_ms + off_ms)) //灭的时间中
//         {
//             LED.Brightness[i] = 0;
//         }
//         else if (ms_count[i] >= (on_ms + off_ms))
//         {
//             group_count[i]++; //完成一次暗灭，暗灭次数加一
//             ms_count[i] = 0;  //重新计时
//         }
//     }
//     else if (ms_count[i] < group_T_ms) //若在组与组之间的间隔时间中，灯灭，等待下一组
//     {
//         LED.Brightness[i] = 0;
//     }
//     else if (ms_count[i] >= group_T_ms) //准备下一组
//     {
//         group_count[i] = 0; //每组中暗灭次数清零
//         ms_count[i] = 0;    //重新计时
//         f = 1;              //成功完成一组
//     }
//     ms_count[i] += (dT_ms); //计时时间增加
//     return f;
// }

/***********************************
*功能：状态改变时，清空之前LED的配置
*参数：无
************************************/
static u16 led_times;
void LED_Bright_Rest(void) //熄灭所有灯，清零各灯计时数，次数
{
    led_times=0;
    //LED.State_Old = LED.State;
    for (int i = 0; i < LED_NUM; i++)
    {
        LED.Brightness[i] = 0;
        ms_count[i] = 0;
        group_count[i] = 0;
    }
}

void led_cnt_res_check()
{
		if(LED.State != LED.State_Old)
		{
			LED_Bright_Rest();
			LED.State_Old = LED.State;		
		}

}
extern u8 height_ctrl_mode;
/***********************************
*功能：判断LED灯的状态，根据LED灯不同的状态，显示相应的状态灯
*参数：dT_ms:调用该函数的周期
************************************/
void Drv_LED_Task(uint8_t dT_ms)
{
    u8 j=0;	
    static uint16_t k;
    led_cnt_res_check();
    switch (LED.State) //判断LED灯的状态，不同的状态，响应不同的状态灯（根据需要自行添加，调用呼吸灯和闪烁灯函数）
    {
    case 0:
			{
				
			}
		break;
    case 1://没电
			Drv_LED_Flash(dT_ms,R_led,20,1,60,60,0);//调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			
		break;
    case 2://校准gyro
			for(u8 i=0;i<LED_NUM;i++)
			{			
				j = Drv_LED_Flash(dT_ms,i,20,4,50,50,0);
				if(j) //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
				{
					LED.State = 0;
				}
			}
		break;	
    case 3://校准acc
			for(u8 i=0;i<LED_NUM;i++)
			{			
				j = Drv_LED_Flash(dT_ms,i,20,8,50,50,0);
				if(j) //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
				{
					LED.State = 0;
				}
			}			
		break;
		case 4://校准水平面
				k -= Drv_LED_Flash(dT_ms,((k) %4),20,1,240,20,0);
				if(k<=0) k = 20000;			
		break;
		case 5: //校准罗盘step1
					{
						Drv_LED_Breath(dT_ms,G_led,300);

					}			
		break;	
		case 6: //校准罗盘step2
					{
						for(u8 i=0;i<2;i++)
						{
							Drv_LED_Flash(dT_ms,R_led-i,20,1,100,100,000) ;
						}	
					}						
		break;
		case 7: //校准罗盘step3
					{
						Drv_LED_Breath(dT_ms,B_led,300);

					}			
		break;
		case 8: //错误
					{

						led_times += Drv_LED_Flash(dT_ms,R_led,20,1,2500,0,0);
						if(led_times == 1)
						{
							LED.State = 0;
						}

					}						
		break;
		case 9: //对频
			
		break;		
		case 10: //等待
			for(u8 i=0;i<LED_NUM;i++)
			{	
				Drv_LED_Flash(dT_ms,i,20,3,60,60,200);//调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			}			
		break;
		case 11://无信号
//			LED.Brightness[1] = 20;
			Drv_LED_Breath(dT_ms,R_led,500);
//			for(u8 i=0;i<LED_NUM;i++)
//			{		
//				Drv_LED_Breath(dT_ms,i,500);
//			}			
		break;	
		case 12://翻滚
			
		break;
		case 13: //闪1次B_led
					{
						Drv_LED_Flash(dT_ms,0,20,1,400,100,300) ;
						j = Drv_LED_Flash(dT_ms,B_led,20,1,400,100,300) ;
						
						if(j)
						{
							LED.State = 0;
						}
					}		
		break;						
		case 14: //闪2次B_led				
					{
						Drv_LED_Flash(dT_ms,0,20,1,400,100,300) ;
						j = Drv_LED_Flash(dT_ms,B_led,20,2,200,200,300) ;
						
						if(j)
						{
							LED.State = 0;
						}
					}			
		break;		
		case 15://闪3次B_led
					{
						LED.Brightness[0] = 20;
						j = Drv_LED_Flash(dT_ms,B_led,20,3,200,200,300) ;
						
						if(j)
						{
							LED.State= 0;
						}
					}		
		break;
		case 16://校准罗盘，未平
					{
						for(u8 i=0;i<2;i++)
						{
							Drv_LED_Flash(dT_ms,R_led+i,20,1,100,100,000) ;
						}
	

					}		
		break;
		case 17://正确、保存数据
					{

						led_times += Drv_LED_Flash(dT_ms,G_led,20,1,2500,0,0);
						if(led_times == 1)
						{
							LED.State = 0;
						}

					}			
		break;
		case 18://禁止解锁
					{

						led_times += Drv_LED_Flash(dT_ms,G_led,20,1,600,400,0);
						if(led_times == 3)
						{
							LED.State = 0;
						}

					}			
		break;
					
		case 23: //闪1次R_led
					{
						Drv_LED_Flash(dT_ms,0,20,1,400,100,300) ;
						j = Drv_LED_Flash(dT_ms,R_led,20,1,400,100,300) ;
						
						if(j)
						{
							LED.State = 0;
						}
					}		
		break;						
		case 24: //闪2次R_led				
					{
						Drv_LED_Flash(dT_ms,0,20,1,400,100,300) ;
						j = Drv_LED_Flash(dT_ms,R_led,20,2,200,200,300) ;
						
						if(j)
						{
							LED.State = 0;
						}
					}			
		break;		
		case 25://闪3次R_led
					{
						LED.Brightness[0] = 20;
						j = Drv_LED_Flash(dT_ms,R_led,20,3,200,200,300) ;
						
						if(j)
						{
							LED.State= 0;
						}
					}		
		break;
		case 80://惯性传感器异常
					{

						Drv_LED_Flash(dT_ms,R_led,20,2,100,100,800);


					}			
		break;
		case 81://电子罗盘异常
					{

						Drv_LED_Flash(dT_ms,R_led,20,3,100,100,800);


					}			
		break;
		case 82://气压计异常
					{

						Drv_LED_Flash(dT_ms,R_led,20,4,100,100,800);


					}			
		break;
///////////////////////
		case 116:		
		
		break;
		case 117:
			
		break;
		case 118:
			
		break;		
		case 119: 
			
		break;	
		case 120: 
			
		break;	
		case 121://mode1
		{
			for(u8 i=0;i<LED_NUM;i++)
			{			
				Drv_LED_Flash(dT_ms,i,20,1,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}
		}
		break;
		case 122://mode2 
			for(u8 i=0;i<LED_NUM;i++)
			{			
				Drv_LED_Flash(dT_ms,i,20,2,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}
		break;		
		case 123://mode3 
			for(u8 i=0;i<LED_NUM;i++)
			{			
				Drv_LED_Flash(dT_ms,i,20,3,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}		
		break;	
		case 124://mode4 
			for(u8 i=0;i<LED_NUM;i++)
			{			
				Drv_LED_Flash(dT_ms,i,20,4,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}		
		break;	
		case 125: 
		
		break;		
		case 126: 
		
		break;	
		case 127: 
		
		break;	
		case 128: 
		
		break;		
		case 129: 
		
		break;	
		case 130: 
		
		break;
		case 131://mode1
			{			
				Drv_LED_Flash(dT_ms,G_led,20,1,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}		
		break;
		case 132://mode2 
			{			
				Drv_LED_Flash(dT_ms,G_led,20,2,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}		
		break;		
		case 133://mode3 
			{			
				Drv_LED_Flash(dT_ms,G_led,20,3,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}			
		break;	
		case 134://mode4 
			{			
				Drv_LED_Flash(dT_ms,G_led,20,4,100,100,600); //调用周期（s）,led编号，亮度（0-20），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;

			}			
		break;			
		default:break;
    }
}
