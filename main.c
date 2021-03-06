/**
  ******************************************************************************
  * @file    main.c
  * @author  casic 203
  * @version V1.0
  * @date    2015-07-27
  * @brief   
  * @attention
  *
  ******************************************************************************
  */ 
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "bsp_TiMbase.h" 
//#include "modbus.h"
#include "bsp_SysTick.h"
#include "string.h"
#include "gprs.h"
#include "bsp_rtc.h"
#include "bsp_date.h"
#include "WatchDog.h"
#include "AiderProtocol.h"
#include "SPI_Flash.h"
#include "common.h"
#include "DS2780.h"
#include "433_Wiminet.h"
#include "math.h"
#include "Sensor.h"
//#define  FILTER_ORDER    6                     //定义数据滤波深度，用于对数据平滑处理,不应小于5

struct    rtc_time        systmtime;           //RTC时钟设置结构体
struct    DeviceSet       DeviceConfig ={0x00};//液位计配置信息结构体
struct    Config_RegPara  ConfigData   ={0x00};//定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
struct    DeviceStatus    SensorMonitorStatus  ={0x00};  //定义液位监测仪设备状态
u16       WWDOG_Feed =0x1FFF;                  //窗口看门狗复位周期为：XX*1.8s = 7.6min
char      PowerOffReset =0;                    //掉电重启标志位
u8        DataSendFlag =0;
//u8        DataCollectBkCount =0;               //备份数据采集计数器
//u8        SensorDataCount = FILTER_ORDER;       //传感数据计数器，标示当前采集到数据数量
u8        DataCollectCount =1;                 //数据采集计数器
char      Usart1_recev_buff[100] ={'\0'};      //USART1接收缓存
u16       Usart1_recev_count =0;               //USART1发送计数器
u8        Uart4_rev_buff[100]={0x00};          //RS485串口接收缓存
u8        Uart4_rev_count=0;                   //RS485串口接收计数器
u8        Uart4_rev_comflag=0;                 //接收完成状态标志
vu8       Uart_rev_finish=0;                   //串口接收完成标志变量
//u32       WorkingTime =0;                    //设备运行使用的时间
u8        DMA_UART3_RECEV_FLAG =0;             //USART3 DMA接收标志变量
u8        Usart2_send_buff[SENDBUFF_SIZE]={'\0'};       //USART2发送缓存
u8        DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE] ={0x00};
static    struct DMA_USART3_RecevConfig  DMA_USART3_RecevIndicator; 
u8        DataRequestFlag =0;                  //数据查询标志变量，1表示接收到服务器查询命令     
//u8        DatRev_OK =0 ;                       //服务器成功接收数据标志变量

extern    char   Usart3_recev_buff[RECEIVEBUFF_SIZE];
extern    u16    Usart3_recev_count;

/*
////volatile unsigned char Uart4_send_ready_flag=0;  //发送就绪状态标志
////volatile unsigned char Crc_counter = 0;          //RS485校验计数器
////volatile unsigned char *Uart4_send_pointer = Uart4_send_buff;//RS485串口发送指针
//extern  char  Usart1_recev_buff[300];
////extern  char  Usart1_send_buff[];
//extern  uint16_t  Usart1_recev_count;
////extern  uint8_t  Usart1_send_count;
////extern u8  DataCollectCache[13][4];   //液位数据采集缓存，最多13组数据；浮点数HEX格式存储，低字节在前，高字节在后
//extern u8  DataCollectCount;          //数据采集计数器
////extern  float LevelData_Float[FILTER_ORDER];       //采集到的临时液位数据，浮点型
//uint32_t  time=0 ;                   // ms 计时变量  
//char      Usart1_send_buff[300]={'\0'};       //USART1发送缓存
//uint8_t   Usart1_send_count=0;                 //USART1发送计数器
//uint32_t  Tic_IWDG=0;                //独立看门狗喂狗时间设置
//extern  char  Usart3_send_buff[];
//extern  uint8_t  Usart3_send_count;                     
//extern  struct  Config_RegPara   ConfigData;  //定义的下发配置参数，HEX格式，方便在系统进入休眠模式之前写入BKP寄存器
//extern  float  Data_Liquid_Level;
*/

//extern void     Delay(uint32_t nCount);         
extern void     RecvBuffInit_USART3(void);
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3接收数据监测与数据解析
int  DMA_UART3_RecevDataGet(void);
extern  u8 SensorStatusUpload(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress,struct DeviceStatus* pSensorStatus, struct rtc_time* pTime, u8 OpType);
void  DeviceSleep(u8 DeviceSleepDelay);
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : 从DMA接收存储器中提取有效数据，放入Usart3_recev_buff[],便于后续数据解析
* Input          : None
* Output         : None
* Return         : 接收数据长度
*******************************************************************************/
int  DMA_UART3_RecevDataGet(void)
{
   int i=0,j=0;
	 u16 DMA_RecevLength =0;
	
	 memset(Usart3_recev_buff, 0x00, sizeof(Usart3_recev_buff));
	 DMA_USART3_RecevIndicator.CurrentDataStartNum = DMA_USART3_RecevIndicator.NextDataStartNum ;
	  
	 i = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
	 if(DMA_USART3_RecevIndicator.DMA_RecevCount <i)
	 {
     DMA_RecevLength =i -DMA_USART3_RecevIndicator.DMA_RecevCount;
   }
	 else
	 {
     DMA_RecevLength = RECEIVEBUFF_SIZE -DMA_USART3_RecevIndicator.DMA_RecevCount + i;
   }
   DMA_USART3_RecevIndicator.DMA_RecevCount = i;
	
	 if((DMA_USART3_RecevIndicator.CurrentDataStartNum + DMA_RecevLength-1) < RECEIVEBUFF_SIZE)
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1;     
   }
	 else
	 {
     DMA_USART3_RecevIndicator.CurrentDataEndNum =(DMA_USART3_RecevIndicator.CurrentDataStartNum +DMA_RecevLength-1) -RECEIVEBUFF_SIZE;  
   }
//	 printf("\r\nDMA UART2 Recev Data Start Num:%d---End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //测试使用
	 if(DMA_USART3_RecevIndicator.CurrentDataEndNum ==(RECEIVEBUFF_SIZE-1))
	 {
	   DMA_USART3_RecevIndicator.NextDataStartNum = 0;
   }
	 else
	 {
		 DMA_USART3_RecevIndicator.NextDataStartNum = DMA_USART3_RecevIndicator.CurrentDataEndNum + 1;
   }	
   //////////////////////////Data Copy///////////////////////////////////////////////////////////////////
   if(DMA_RecevLength !=0)
	 {
     j =DMA_USART3_RecevIndicator.CurrentDataStartNum;
		 if(DMA_USART3_RecevIndicator.CurrentDataEndNum >DMA_USART3_RecevIndicator.CurrentDataStartNum)
		 {
			 for(i=0; i<DMA_RecevLength; i++,j++)
			 {
					Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];	
			 }
		 }
		 else
		 {
			 for(i=0; i<DMA_RecevLength; i++)
			 {
					if( j<(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j++;				
					}
					else if( j==(RECEIVEBUFF_SIZE-1) )
					{
						 Usart3_recev_buff[i] =DMA_USART3_RecevBuff[j];
						 j =0;				 
					}
			  } 
      }
    }
	  return DMA_RecevLength;
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress)
{
	int DataLength =0;
	int i=0;
	u16 StateFlag =0;
	
  if(DMA_UART3_RECEV_FLAG==1)
  {
//		 DMA_Cmd(DMA1_Channel6, DISABLE);           //关闭DMA防止处理期间有数据
//		 USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);   //向USART2发送数据前，先打开USART2接收空闲中断，便于监测数据接收完成
		 DataLength = DMA_UART3_RecevDataGet();
//		 DMA_UART3_RECEV_FLAG =0;
//		 DMA_Cmd(DMA1_Channel6, ENABLE);            //开启DMA 
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);             //测试使用
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);           //测试使用
			  Usart3_recev_count =DataLength;                          //数据长度赋值，下一步数据解析会用到，缺少则无法解析，考虑软件优化改为参数传递而非全局变量形式
			  for(i=0;i<DataLength;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);               //测试使用
        }
        StateFlag =Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //服务器下发数据解析    //
        //对接收数据类型进行指示 		
		 }
		 else
		 {
        printf("\r\nNo data\r\n");
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //关闭DMA防止处理期间有数据
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //复位DMA数据接收BUFF
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //开启DMA 
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //向USART2发送数据前，先打开USART2接收空闲中断，便于监测数据接收完成
  } 
	return StateFlag;
}


/*******************************************************************************
功能：去除最大最小值，对剩余数据求平均
输入：浮点数数组，处理数据数量
返回：处理后的平均值
编写：
编写日期：XX年XX月XX日
版本：v0.1
********************************************************************************/
//float GetAverageData(float* pSensorData, u8 SensorData_Count,float* pDiffer)
//{
//	float DataTemp[FILTER_ORDER] ={0.0};
//  float Temp = 0.0;
//  float AverageData = 0.0;
//  float Dvalue = 0.0;          //最大值与最小值之差
//  u8    i=0,j=0;
//  
//  if(SensorData_Count==0)
//	{
//     return 0;
//  }
//  else if(SensorData_Count==1)
//	{
//     return pSensorData[0];
//  }
//  
//  for(i=0;i<SensorData_Count;i++)
//  {
//    DataTemp[i] = pSensorData[i];
////  	printf("\r\n**%f**\r\n##", DataTemp[i]);    //测试使用
//  }
//	
//  for(i=SensorData_Count-1;i>=1;i--)
//	{ 
//		for(j=SensorData_Count-1;j>=SensorData_Count-i;j--)    //从小到大顺序排序
//		{
//       if(DataTemp[j] < DataTemp[j-1])
//			 {
//          Temp = DataTemp[j-1];
//				  DataTemp[j-1] = DataTemp[j];
//				  DataTemp[j] =  Temp;
//       }
//    }
//  }

//	Temp =0;                            //复位累加器
//	Dvalue =DataTemp[SensorData_Count-1]-DataTemp[0];
//	*pDiffer =Dvalue ;                   //返回最大最小差值
//	
//	if((Dvalue > 0.1) &&(SensorData_Count>=4))           //当最大值与最小值之差超过10cm时，去除2个偏差较大值
//	{
//		for(i=1;i<SensorData_Count-1;i++)
//		{
//			 Temp = Temp + DataTemp[i];
//		}
//		AverageData = Temp/(SensorData_Count-2);
//	}
//	else                                  
//	{
//    for(i=0;i<SensorData_Count;i++)  
//		{
//			 Temp = Temp + DataTemp[i];
//		}
//		AverageData = Temp/SensorData_Count;
//  }
//	return AverageData;
//}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART1_ReceiveReset(void)
{
  Usart1_recev_count =0;
	memset(Usart1_recev_buff,0x00,sizeof(Usart1_recev_buff));
	Uart_rev_finish=0;         
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void UART4_ReceiveReset(void)
{
  Uart4_rev_count =0;
	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));
	Uart_rev_finish=0;         
}
/*******************************************************************************
* Function Name  : XX
* Description    : XX
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
//float Temperature_DataGet(void)
//{
//   u8     i=0,j=0; 	
//   u8     TemperatureReadCmd[8]={0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};           //读温度数据指令,最后两位为CRC校验码
//   u8     TemperatureReadResp[3]={0x01,0x03,0x02};   //数据应答指示:01,03,02,T_H,T_L,CrcL,CrcH
//   u8*    pUart4Send =  TemperatureReadCmd;
//   u8*    pDataRead  =NULL; 
//   short  TempShort=0;           //用于负值转换
//   float  Temper =0;             //温度数据暂存
//   float  DataTempArry[FILTER_ORDER] ={0};

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //接收标志变量复位
//     Uart4_rev_count   =0;   //接收计数器清零
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //清空数据接收BUFF
//		 
//     DIR485_Send();                           //485发送使能
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, TemperatureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //等待发送完毕 
//		 }
//		 Delay_us(450);                  //在数据接收完成后，延迟一段时间再切换收发控制开关，默认延时350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //测试使用
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //检查是否收到温度传感器应答
//				 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
//				 {
//           TempShort =pDataRead[3]*256 +pDataRead[4];
//					 DataTempArry[j]  =TempShort;
//					 DataTempArry[j] = DataTempArry[j]/10;		 
//					 
////					 DataTempArry[j] =(pDataRead[3]*256 +pDataRead[4])/10;
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Temperature is :%.1f ℃\r\n",DataTempArry[j]);                 //测试使用
//     		   #endif
//         }
//				 break;
//      }
//			else
//			{
//				Delay_ms(100);   
//			}
//    }	 
//	}
//  Temper = GetAverageData(DataTempArry, FILTER_ORDER);  //滤除最大最小值，取得平�
//	printf("\r\nTemperature\r\n");                 //测试使用�
//  for(i=0;i<FILTER_ORDER;i++)
//	{
//     printf("--%.1f--",DataTempArry[i]);                 //测试使用
//  }
//	return  Temper;		
//}
*/
/*******************************************************************************
* Function Name  : XX
* Description    : 适用于昆仑中大IP68温度传感器数据采集，针对二院前期安装设备，数据采集指令略有差异
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//float Temperature_DataGet(u8*  SenserStatus)
//{
//   u8     i=0,j=0; 	
//   u8     TemperatureReadCmd[8]={0xC0,0x03,0x00,0x01,0x00,0x01,0xC5,0x1B};           //读温度数据指令,最后两位为CRC校验码
//   u8     TemperatureReadResp[3]={0xC0,0x03,0x02};   //数据应答指示:01,03,02,T_H,T_L,CrcL,CrcH
//   u8*    pUart4Send =  TemperatureReadCmd;
//   u8*    pDataRead  =NULL; 
//	 u8     RespCount  =0;
//   float  Temper =0;   //温度数据暂存
//   float  DataTempArry[FILTER_ORDER] ={0};
//   short  TempShort =0;
//   float  MaxDifference =0;  //最大值与最小值差值

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //接收标志变量复位
//     Uart4_rev_count   =0;   //接收计数器清零
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //清空数据接收BUFF
//		 
//     DIR485_Send();                           //485发送使能
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, TemperatureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //等待发送完毕 
//		 }
//		 Delay_us(300);                  //在数据接收完成后，延迟一段时间再切换收发控制开关，默认延时350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //测试使用
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //检查是否收到温度传感器应答
//				 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
//				 {
//           TempShort =(pDataRead[3]*256 +pDataRead[4]);
//					 Temper =TempShort;
//					 DataTempArry[j] =Temper/10; 		 
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Temperature is :%f ℃\r\n", DataTempArry[j]);                 //测试使用
//     		   #endif
//					 RespCount++;	
//					 *SenserStatus =0;
//         }
//				 else
//				 {
//           *SenserStatus =2;
//         }
//				 break;
//      }
//			else
//			{
//				*SenserStatus =1;
//				Delay_ms(100);   
//			}
//    }	 
//	}
//	if(RespCount>=1)
//	{
//     *SenserStatus =0;    //允许存在丢数据情况
//  }
//  Temper = GetAverageData(DataTempArry, RespCount, &MaxDifference);  //滤除最大最小值，取得平均
//	Temper = Temper*10;        //精度控制在1位小数，单位℃
//	Temper =(int)Temper;       //精度控制在1位小数，单位℃
//	Temper =Temper/10;         //精度控制在1位小数，单位℃
//	
//	if((Temper<-20)||(Temper>200))                 //温度传感器数据超量程报异常
//	{
//     *SenserStatus =2;
//  }
//	printf("\r\nTemperature\r\n");                 //测试使用
//  for(i=0;i<FILTER_ORDER;i++)
//	{
//     printf("--%.1f--",DataTempArry[i]);         //测试使用
//  }
//	return  Temper;		
//}

/*******************************************************************************
* Function Name  : XX
* Description    : 适用于昆仑中大IP68压力传感器数据采集
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//float Pressure_DataGet(u8*  SenserStatus)
//{
//   u8     i=0,j=0; 	
//   u8     PressureReadCmd[8]={0x01,0x04,0x00,0x02,0x00,0x02,0xD0,0x0B};           //读压力数据指令,最后两位为CRC校验码
//   u8     PressureReadResp[3]={0x01,0x04,0x04};   //数据应答指示:01,04,04,P_1,P_2,P_3,P_4,CrcL,CrcH    //浮点表示，高位字节在前，低位字节在后
//   u8*    pUart4Send = PressureReadCmd;
//   u8*    pDataRead  =NULL; 
//   u8     RespCount =0;
////   short  TempShort=0;           //用于负值转换
//   float  Pressure =0;   //压力数据暂存
//   float  DataTempArry[FILTER_ORDER] ={0};
//   union  Hfloat  PressureTemp;
//   float  MaxDifference =0;  //最大值与最小值差值

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //接收标志变量复位
//     Uart4_rev_count   =0;   //接收计数器清零
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //清空数据接收BUFF
//		 
//     DIR485_Send();                           //485发送使能
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, PressureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //等待发送完毕 
//		 }
//		 Delay_us(350);                  //在数据接收完成后，延迟一段时间再切换收发控制开关，默认延时350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //测试使用
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)PressureReadResp, sizeof(Uart4_rev_buff), sizeof(PressureReadResp));  //检查是否收到温度传感器应答
//			 	 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-10)))
//				 {
//           for(i=0;i<4;i++)
//					 {			
//              PressureTemp.Data_Hex[3-i] =pDataRead[3+i];                     //高位字节在前
////						  #if DEBUG_TEST
////							printf("\r\n--%x-- \r\n", pDataRead[3+i]);  //测试使用
////							#endif
//           }
//					 DataTempArry[j] =PressureTemp.Data_Float;
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Pressure is :%.3f kPa\r\n", DataTempArry[j]);  //测试使用
//     		   #endif
//					 RespCount++;
//					 *SenserStatus =0;
//         }
//         else
//				 {
//           *SenserStatus =2;
//         }
//				 break;
//       }
//			 else
//			 {
//				 *SenserStatus =1;
//			 	 Delay_ms(100);   
//		   }
//     }	 
//	 }
//	 if(RespCount>=1)
//	 {
//      *SenserStatus =0;   //允许存在丢数据情况
//   }
//   Pressure = GetAverageData(DataTempArry, RespCount, &MaxDifference);  //滤除最大最小值，取得平均
//	 Pressure =(int)Pressure;                                //精度控制在3位小数，单位kPa
//	 Pressure =Pressure/1000;                                //精度控制在3位小数，单位MPa
//   printf("\r\nSoothData:%f MPa\r\n",Pressure);            //测试使用
//	 if((Pressure<-0.1)||(Pressure>1.6))                     //压力数据超量程报异常
//	 {
//     *SenserStatus =2;
//   }
//	 if(Pressure<0)
//	 {
//      Pressure=0;                                      //负值修正，去除零漂造成的影响
//   }
//	 printf("\r\nPressure\r\n");                           //测试使用
//   for(i=0;i<FILTER_ORDER;i++)
//	 {
//     printf("--%.3f--",DataTempArry[i]);                 //测试使用
//   }
//	 return  Pressure;		
//}		
		
/*******************************************************************************
* Function Name  : void SenserDataCollect1(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
* Description    : 适用于老板子
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
//void SenserDataCollect1(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
//{
//			u32       TimCount_Current =0;
//			u8        i=0;   //测试使用

//			#if DEBUG_TEST
//			printf("DataCollectCount:%d\r\n",DataCollectCount);  //测试使用
//			#endif
//			
////			if(DataCollectCount == DeviceConfig.CollectNum)    //第一次采集数据，记录采集时间信息，后续考虑每次是否采用真实时间
//			{
//				pGetData->DataCount =0;                            //对液位数据缓存计数器赋初值                       
//				TimCount_Current = RTC_GetCounter();
//				Time_Display(TimCount_Current,&systmtime); 
//						
//			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
//				DeviceConfig.Time_Min  =systmtime.tm_min;
//				DeviceConfig.Time_Hour =systmtime.tm_hour;
//			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
//			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
//			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //对上传年份去基数修正				
//    		#if DEBUG_TEST
//				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //测试使用
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //测试使用
//    		#endif
//			}		
//			i = pGetData->DataCount;
//    	pGetData->TemperatureData[i] = Temperature_DataGet();
//			Delay_ms(800);
//			pGetData->PressureData[i]    = Pressure_DataGet();
//			if(i==0)
//			{
//        pGetData->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);       //第一组数据采集时间
//      }
//		  else
//			{
//        pGetData->CollectTime[i] =(pGetData->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //后续每组数据采集时间为在前一组数据基础上增加采集间隔
//      }
//			pGetData->DataCount =(pGetData->DataCount)+1;
//     	DataCollectCount--;	

//}
*/

/*******************************************************************************
* Function Name  : XX
* Description    : 适用于新板子
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void Sensor_DataCollect( struct SenserData* pGetData, struct DeviceStatus* pMonitorStatus)
//{
//   	 u32       TimCount_Current =0;
//		 u8        i=0;                     //测试使用
////      char      PressureSensorZeroAdjust[8]={0x01,0x06,0x00,0x05,0x00,0x00,0x99,0xCB};
////      char      PressureSensorAccessCode[8]={0x01,0x06,0x00,0x0A,0x38,0x79,0x7B,0xEA};
//	   char*     pRecevBuff  =NULL; 
//		 u8        PressStatus =3;        //压力传感器状态标志变量，用于检测传感器是否工作正常，0表示工作正常，1表示失效,3表示未启用
//     u8        TemperatueStatus =3;   //温度传感器状态标志变量，用于检测传感器是否工作正常，0表示工作正常，1表示失效,3表示未启用
//           
//		 TimCount_Current = RTC_GetCounter();
//		 Time_Display(TimCount_Current,&systmtime); 				
//		 DeviceConfig.Time_Sec  =systmtime.tm_sec;
//		 DeviceConfig.Time_Min  =systmtime.tm_min;
//		 DeviceConfig.Time_Hour =systmtime.tm_hour;
//		 DeviceConfig.Time_Mday =systmtime.tm_mday;	
//		 DeviceConfig.Time_Mon  =systmtime.tm_mon;                                
//		 DeviceConfig.Time_Year =systmtime.tm_year-2000; //对上传年份去基数修正
// 		                                     		
//     #if DEBUG_TEST
//		 printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //测试使用
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //测试使用
//     #endif
//			
//		 //先开启静压传感器，采集静压式传感器数据，
//     PowerON_485();             //开启485模块电源
//		 Delay_ms(500);  	
//		 PowerON_12V();	            //开启12V总电源
//		 #if DEBUG_TEST
//		 printf("\n\r***----12V DC-DC Power ON----***\r\n");   //测试使用
//		 #endif
//		 Delay_ms(1000);     
//		 PowerON_Pressure();        //开启静压传感器电源
//		 #if DEBUG_TEST
//		 printf("\n\r***----Pressure Sensor Power ON----***\r\n");   //测试使用
//		 #endif
//		 for(i=0;i<80;i++)          //增加延时，使静压探头稳定，同时监测串口控制命令输入
//		 {
//				Delay_ms(100);   

//				pRecevBuff = Find_SpecialString(Usart1_recev_buff, "BATT Recover" ,sizeof(Usart1_recev_buff), 12);  //检查是否收到静压传感器零点重置命令，用于传感器零点校准
//				if(pRecevBuff !=NULL)
//				{
//						DataRead_From_Flash(0,9,0, &i,1);                //从Flash中读出电池容量系数备份数据
//						if(i<=113)
//						{
//							 DataWrite_To_Flash(0,1,0,  &i,1);             //将备份区数据更新到电池容量系数数据存储区
//							 #if DEBUG_TEST
//							 printf("\r\n------------BATT Recover Sucess!------------\r\n");                 //测试使用
//						   #endif
//						}
//						#if DEBUG_TEST
//						printf("\r\n------------Warning:BATT Recover Data too big!------------\r\n");      //测试使用
//						#endif
//						pRecevBuff =NULL;
//				}				
//				UART1_ReceiveReset();
//		 }	  
//		 DIR485_Receive();
//		 Delay_ms(500); 
////		 UART1_ReceiveReset();
//		 
////		 i = pGetData->DataCount;
//		 pGetData->PressureData[0]   =  Pressure_DataGet(&PressStatus);
//		 pMonitorStatus->PressSensorStatus =PressStatus;
//		 PowerOFF_Pressure();                     //关闭静压传感器电源
//		 #if DEBUG_TEST
//		 printf("\n\r***----Pressure Sensor Power OFF----***\r\n");   //测试使用
//		 #endif
//		 Delay_ms(500);
//		 PowerON_TemperatureSensor();             //打开温度传感器电源
//		 #if DEBUG_TEST
//		 printf("\n\r***----Temperature Sensor Power ON----***\r\n");   //测试使用
//		 #endif
//		 Delay_ms(3000);
//		 pGetData->TemperatureData[0] = Temperature_DataGet(&TemperatueStatus);
//		 pMonitorStatus->TemperatureSensorStatus =TemperatueStatus;
//		 Delay_ms(100);
//		 PowerOFF_TemperatureSensor();            //关闭温度传感器电源
//		 #if DEBUG_TEST
//		 printf("\n\r***----Temperature Sensor Power OFF----***\r\n");   //测试使用
//		 #endif
//		 PowerOFF_12V();
//		 #if DEBUG_TEST
//		 printf("\n\r***----12V DC-DC Power OFF----***\r\n");   //测试使用
//		 #endif
//		 Delay_ms(100);
//		 PowerOFF_485();                          //当液位数据采集完成时，关闭Uart转485模块电源
//		 pGetData->CollectTime[0] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);       //数据采集时间

//	
//}


/*******************************************************************************
* Function Name  :Power_SX1278_Init()
* Description    : 初始化433模块
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  Power_SX1278_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* config GPIOA clock */
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE);
  /* Configure PowerEN_3.8V(PA.07) as output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

}

/*******************************************************************************
* Function Name  : void SX1287_Init(u16 sNodeID)
* Description    : 初始化433模块
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SX1287_Init(u16 sNodeID)
{
  u16  UartConfig   =0;
	u16  CentralFreq  =0;
	u16  NodeID       =0;
	u8   ExpFactor    =0;
	u8   ExpBW        =0;
	u8   WorkMode     =0;
	u8   NetID        =0;
	u8   PowerGrade   =0;
	u8   BreathPeriod =0;
	u8   WakeTime     =0;
  u8   BaudRate     =0;
	u8   VerifyType   =0; 
  u16  Temp =0;	
	
//	Power_SX1278_Init();
	////////////////////配置433模块参数 //////////////////// 
//	SetNodeSerialPort(4, 0);                           //设置433模块串口波特率：9600,校验类型:无校验
	SetNodeCentralFrequency(439);                      //设置433模块载波中心频率：434MHz
	SetNodeFrequencyExpandFactor(11);                  //设置433模块扩频因子：2048
	SetNodeFrequencyExpandBandwidth(7);                //设置433模块扩频带宽：125K
	SetNodeWorkMode(2);                                //设置433模块工作模式：节点模式
	SetNodeID (sNodeID);                               //设置433模块ID：输入节点ID
	SetNodeNetworkID (66);                              //设置433模块网络ID：19
	SetNodeSendPowerGrade (7);                         //设置433模块发射功率等级：20dBm
	SetNodeBreathPeriod (0);                           //设置433模块呼吸周期:2s
	SetNodeBreathTime (5);                             //设置433模块呼吸时间：64ms
	
	PowerOFF_433();                                   //重启433模块，使配置生效        
	Delay_ms(5000);
	PowerON_433(); 
	////////////////////读取433模块参数 //////////////////// 
//u8  GetNodeReceiveSignalEnergy ();                 //获取433模块上一帧数据接收信号强度
  UartConfig =GetNodeSerialPortConfig();              //获取433模块串口配置参数
  BaudRate   =UartConfig>>8;
  VerifyType =UartConfig &0xFF;
  printf("\r\nBaud Rate:%d---Verify Type:%d\r\n",BaudRate,VerifyType);//测试使用
  CentralFreq =GetNodeCentralFrequency ();               //获取433模块载波频率参数
  printf("\r\nCentral Frequency :%d MHz\r\n",(CentralFreq+1));   //测试使用
  ExpFactor   =GetNodeFrequencyExpandFactor();           //获取433模块扩频因子参数
  switch(ExpFactor )
  {
			case 7:  
			{
				Temp=128;
				break;
			}
			case 8:  
			{
				Temp=256;
				break;
			}
			case 9:  
			{
				Temp=512;
				break;
			}		
			case 10:  
			{
				Temp=1024;
				break;
			}
			case 11:  
			{
				Temp=2048;
				break;
			}	
			case 12:  
			{
				Temp=4096;
				break;
			}
			default:
			{
				Temp=0;
				break;
			}		
  }
  printf("\r\nNode Frequency Expand Factor:%d \r\n",Temp);   //测试使用
  ExpBW =GetNodeFrequencyExpandBandwidth ();                 //获取433模块扩频带宽参数
	switch( ExpBW )
  {
			case 6:  
			{
				Temp=63;       //62.5约等于63
				break;
			}
			case 7:  
			{
				Temp=125;
				break;
			}
			case 8:  
			{
				Temp=256;
				break;
			}		
			case 9:  
			{
				Temp=512;
				break;
			}
			default:
			{
				Temp=0;
				break;
			}		
  }
	printf("\r\nNode Frequency Expand Bandwidth:%dKHz\r\n",Temp);   //测试使用
  WorkMode = GetNodeWorkMode ();                                  //获取433模块工作模式参数
	switch( WorkMode )
  {
			case 0:  
			{
	      printf("\r\n433 Module Work Mode is: Standard\r\n");   //测试使用
				break;
			}
			case 1:  
			{
				printf("\r\n433 Module Work Mode is: Center\r\n");    //测试使用
				break;
			}
			case 2:  
			{
				printf("\r\n433 Module Work Mode is: Node\r\n");    //测试使用
				break;
			}		
			default:
			{
				printf("\r\n433 Module Work Mode is: Unknown\r\n");    //测试使用
				break;
			}		
  }
  NodeID =GetNodeID ();                                 //获取433模块节点ID
	printf("\r\n433 Module Node ID is: %x\r\n",NodeID);   //测试使用
  NetID =GetNetworkID ();                               //获取433模块网络ID
	printf("\r\n433 Module Network ID is: %x\r\n",NetID); //测试使用
  PowerGrade = GetNodeSendPowerGrade ();                //获取433模块发射功率
	printf("\r\n433 Module Send Power Grade is: %d\r\n",PowerGrade); //测试使用
  BreathPeriod = GetNodeBreathPeriod ();                //获取433模块呼吸周期
	switch( BreathPeriod )
  {
			case 0:  
			{
				Temp=2;       
				break;
			}
			case 1:  
			{
				Temp=4;
				break;
			}
			case 2:  
			{
				Temp=6;
				break;
			}		
			case 3:  
			{
				Temp=8;
				break;
			}
			case 4:  
			{
				Temp=10;
				break;
			}
			default:
			{
				Temp=0;
				break;
			}		
  }
	printf("\r\nNode Breath Period:%d s\r\n",Temp );   //测试使用
  WakeTime  =  GetNodeBreathTime ();                 //获取433模块呼吸时间
	switch( WakeTime )
  {
			case 0:  
			{
				Temp=2;       
				break;
			}
			case 1:  
			{
				Temp=4;
				break;
			}
			case 2:  
			{
				Temp=8;
				break;
			}		
			case 3:  
			{
				Temp=16;
				break;
			}
			case 4:  
			{
				Temp=32;
				break;
			}
			case 5:  
			{
				Temp=64;
				break;
			}			
			default:
			{
				Temp=0;
				break;
			}		
  }
	printf("\r\nNode Wake Time:%d ms\r\n",Temp );   //测试使用

}
/*******************************************************************************
* Function Name  : XXX
* Description    : 设备进入休眠状态
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DeviceSleep(u8 DeviceSleepDelay)
{
  u16 WakeupTime  =0;  //下一次唤醒时间
	u32 SleepTime   =0;  //本次设备休眠时间	
	u32 CurrentTime =0;  //RTC当前时间
	u32 RelativeCurrentTime =0;  //当前时间(相对值，可能超过1440)
	u8  i=0;
	
  CurrentTime = RTC_GetCounter();                   //获取系统当前时间
  Time_Display(CurrentTime,&systmtime);    

  CurrentTime = RTC_GetCounter();
	Time_Display(CurrentTime,&systmtime); 
	CurrentTime =	systmtime.tm_hour*60+systmtime.tm_min;
	if((DeviceConfig.CollectStartTime + DeviceConfig.CollectPeriod* (DeviceConfig.CollectNum-1))<1440)            
	{ 
		  if(CurrentTime <DeviceConfig.CollectStartTime)
			{
        WakeupTime =DeviceConfig.CollectStartTime ;     //下一次设备唤醒时刻,以分钟为单位，每天24小时，0~1440 
				SleepTime  =(WakeupTime-CurrentTime)*60;        //单位秒
      }
			else if(CurrentTime >(DeviceConfig.CollectStartTime+ (DeviceConfig.CollectNum-1)*DeviceConfig.CollectPeriod))
			{
        WakeupTime =DeviceConfig.CollectStartTime ;//下一次设备唤醒时刻,以分钟为单位，每天24小时，0~1440
				SleepTime  =(WakeupTime +1440 -CurrentTime)*60;  //单位秒   
      }
			else
			{
        for(i=0;i<(DeviceConfig.CollectNum-1);i++)
				{
	        if((CurrentTime>=(DeviceConfig.CollectStartTime+ i*DeviceConfig.CollectPeriod))&&(CurrentTime<=(DeviceConfig.CollectStartTime+ (i+1)*DeviceConfig.CollectPeriod)))  
			    {  
				    WakeupTime =DeviceConfig.CollectStartTime +(i+1)*DeviceConfig.CollectPeriod; //下一次设备唤醒时刻,以分钟为单位，每天24小时，0~1440
				    SleepTime  =(WakeupTime-CurrentTime)*60;        //单位秒
						break;
          }
        }
      }
//		  printf("\r\n---1--DeviceConfig.CollectNum:%d-----CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,CurrentTime,WakeupTime);                 //测试使用
			printf("\r\nDeviceConfig.CollectNum:%d--%d--%d------CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,DeviceConfig.CollectPeriod,DeviceConfig.CollectStartTime,CurrentTime,WakeupTime);                 //测试使用
				
			DeviceSleepDelay =DeviceSleepDelay %DEVICE_DELAY_MAX;  //防止出现溢出
			SleepTime =SleepTime+ DeviceSleepDelay*20;             //根据设备编号不同增加相应延时，避免通过433网络上传数据时发生碰撞，单位秒
		  gotoSleep(SleepTime);
	}
	else
	{   
			if((CurrentTime <DeviceConfig.CollectStartTime)&&(CurrentTime >((DeviceConfig.CollectStartTime + DeviceConfig.CollectPeriod* (DeviceConfig.CollectNum-1))%1440)))  
			{
        WakeupTime =DeviceConfig.CollectStartTime ;     //下一次设备唤醒时刻,以分钟为单位，每天24小时，0~1440 
				SleepTime  =(WakeupTime-CurrentTime)*60;        //单位秒
//        printf("\r\n---1--\r\n");
      }
			else
			{
        RelativeCurrentTime =CurrentTime;
				for(i=0;i<(DeviceConfig.CollectNum-1);i++)
				{
					  if((DeviceConfig.CollectStartTime+ i*DeviceConfig.CollectPeriod)>=1440)
						{
                RelativeCurrentTime =CurrentTime +1440;
            }
					  if((RelativeCurrentTime>=(DeviceConfig.CollectStartTime+ i*DeviceConfig.CollectPeriod))&&(RelativeCurrentTime<=(DeviceConfig.CollectStartTime+ (i+1)*DeviceConfig.CollectPeriod)))  
						{  
							 WakeupTime =(DeviceConfig.CollectStartTime +(i+1)*DeviceConfig.CollectPeriod); //下一次设备唤醒时刻,以分钟为单位，每天24小时，0~1440
							 SleepTime  =(WakeupTime-RelativeCurrentTime)*60;        //单位秒
//							 printf("\r\n---2--\r\n");
							 break;
						}
//						printf("\r\n---0--\r\n");			
        }
//				printf("\r\n---3--\r\n");
      }
		  printf("\r\nDeviceConfig.CollectNum:%d--%d--%d------CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,DeviceConfig.CollectPeriod,DeviceConfig.CollectStartTime,CurrentTime,WakeupTime);                 //测试使用
			//根据设备编号不同增加相应延时，避免通过433网络上传数据时发生碰撞，单位秒
			DeviceSleepDelay =DeviceSleepDelay %DEVICE_DELAY_MAX; //防止出现溢出
			SleepTime =SleepTime+ DeviceSleepDelay*20;       //向后推迟20s*ID
	
			
//			if(DeviceSleepDelay%2 ==0)   //偶数
//			{
//        SleepTime =SleepTime+ DeviceSleepDelay*20;       //偶数向后推迟20s
//      }
//			else                         //奇数
//			{
//				 if(SleepTime >DeviceSleepDelay)
//				 {
//             SleepTime =SleepTime -DeviceSleepDelay*20;  //奇数提前20s
//         }
//				 else
//				 {
//             SleepTime =SleepTime;
//         }
//      }
			gotoSleep(SleepTime);			
   }
}
/*******************************************************************************
* Function Name  : void PeripheralInit( void )
* Description    : 初始化端口及外设
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit(u8* pDevID)
{
//	u8    SWD_STATUS   =0;
//	u8    TempArray2[2]={0};
//	u16   ShortTemp2   =0;
	
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//首先开启窗口看门狗，计数器值为7f,窗口寄存器为5f,分频数为8	
	USART1_Config();      /* USART1 配置模式为 9600 8-N-1，  中断接收 */
	USART2_Config();      /* USART2 配置模式为 9600 8-N-1，  中断接收 */
	USART3_Config();      /* USART3 配置模式为 9600 8-N-1，  中断接收 */
//	USART4_Config();      /* UART4  配置模式为 9600 8-N-1，  中断接收 */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();
  USART2_DMA_Config();
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //配置串口DMA1接收
//  TIM2_Configuration();       /* 定时器TIM2参数配置 */	
//	TIM2_NVIC_Configuration();  /* 设置定时器TIM2的中断优先级 */
//	TIM3_Configuration();       /* 定时器TIM3参数配置 */	
//	TIM3_NVIC_Configuration();  /* 设置定时器TIM3的中断优先级 */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //暂时关闭定时器TIM3
	
	RTC_NVIC_Config();                 /* 配置RTC秒中断优先级 */
	RTC_CheckAndConfig(&systmtime);
//	Time_Show(&systmtime);             /* Display time in infinite loop */
//	WorkingTime =RTC_GetCounter();     //采集设备启动时间
  SysTick_Init();
  PowerON_Flash();                     //打开Flash电源 
  Delay_ms(100); 
//	SPI_FLASH_Init();
//	PowerON_GPRS();                    //打开GPRS模块电源，****必须打开，查明原因;后续电路板焊接正确以后此步骤可以省略
//	Delay_ms(100);
//	PowerON_TemperatureSensor();       //打开超声波探头电源，在TCP连接正确建立以后再开启探头电源，后续如果需要备份历史数据，则需要做相应修改
//	Delay_ms(100);   
//  PowerON_485();                     //打开485电源
//	Delay_ms(100);
//清除液位计刚启动时串口的乱码,考虑是否有必要 //临时使用
//	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));    //临时使用
	
 	DMA_USART3_RecevIndicator.CurrentDataStartNum =0;  //初始化当前接收数据开始位置
	DMA_USART3_RecevIndicator.CurrentDataEndNum =0;    //初始化当前接收数据结束位置
	DMA_USART3_RecevIndicator.NextDataStartNum =0;     //初始化下一次接收数据开始位置
	DMA_USART3_RecevIndicator.DMA_RecevCount =0;
	///////////////////必须将模块配置成接收状态，否则串口接收不到数据
	HIGH_433_SET();              //433模块SET管脚拉高，切换到接收模式
	Delay_ms(100);
  LOW_433_EN();                //433模块EN管脚拉低，切换到高速模式
	Delay_ms(100);
	
	USART_GetFlagStatus(USART1,USART_FLAG_TC);       //串口硬件复位之后，发送首字节之前，先读一下USART_SR,防止数据发送时首字节被覆盖
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //串口硬件复位之后，发送首字节之前，先读一下USART_SR,防止数据发送时首字节被覆盖
//	USART_GetFlagStatus(UART4,USART_FLAG_TC);        //串口硬件复位之后，发送首字节之前，先读一下USART_SR,防止数据发送时首字节被覆盖
	Uart_rev_finish=0;
	
	#if DEBUG_TEST
	printf("\n\r****************************************************************\r\n");   //测试使用
	printf("\n\r*-----欢迎使用BIRMM-CorrE-A保温层下腐蚀环境监测仪!-------------*\r\n");   //测试使用
	printf("\n\r*-----硬件版本号：BIRMM-CorrE V1.1---------------------------------*\r\n");   //测试使用
  printf("\n\r*-----软件版本号：BIRMM-CorrE-A_V1.1-------------------------------*\r\n");   //测试使用
	printf("\n\r*-----设备编号：%.2x%.2x%.2x%.2x%.2x%.2x-----------------------------------*\r\n",pDevID[0],pDevID[1],pDevID[2],pDevID[3],pDevID[4],pDevID[5]);   //测试使用
	printf("\n\r*-----版权所有：中国航天科工二院智慧管网技术研究与发展中心-----*\r\n");   //测试使用
	printf("\n\r****************************************************************\r\n");   //测试使用
	#endif
/*	
	//////////读出静压探头累计读数失败次数，超声波探头累计读数失败次数，433模块数据传输累计失败次数
//  DataRead_From_Flash(1,4,0, TempArray2,2);            //从Flash中读出静压传感器数据采集失败次数
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****Press senser data collect fail counter:%3d -----****\r\n",ShortTemp2);   //测试使用
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,5,0, TempArray2,2);            //从Flash中读出超声波传感器数据采集失败次数
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****Ultrasonic senser data collect fail counter:%3d ****\r\n",ShortTemp2);   //测试使用
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,6,0, TempArray2,2);            //从Flash中读出3G模块拨号失败次数
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****433 Modle Communication fail counter:%3d -------****\r\n",ShortTemp2);   //测试使用
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,7,0, TempArray2,2);            //从Flash中读出3G模块上一次通信信号强度
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****433 Modle latest receive signal density:%3d dBm  ***\r\n",(short)ShortTemp2);   //测试使用
//	printf("\n\r********************************************************\r\n");                     //测试使用
//	#endif
//	
//	DataRead_From_Flash(1,0,0, &SWD_STATUS,1);     //从Flash中读取SWD端口复用状态标志
//	if(SWD_STATUS==1)                     
//	{
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE); 
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);    //禁止SWD接口下载功能，用作GPIO，可通过长按复位下载程序或者通过串口下发指定解除占用;
//		#if DEBUG_TEST
//		printf("\n\r<<<<!!!!!!!!!!!----SWD-Lock----!!!!!!!!!!!>>>>\r\n");   //测试使用
//		#endif		
//	}
//	else
//	{
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);  
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
//		#if DEBUG_TEST
//		printf("\n\r<<<<!!!!!!!!!!!----SWD-Unlock----!!!!!!!!!!!>>>>\r\n");   //测试使用
//		#endif
//	}
*/
	ConfigData_Init(&DeviceConfig);         //从Flash中读取配置参数

}


/*******************************************************************************
* Function Name  : int main(void)
* Description    : 主函数
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

int main( void )
{
//	#ifdef DEBUG
//  debug();
//  #endif
	
  int16_t  ambient_temperature =0;
  int16_t  residual_capacity=0;
  u8  g=0;
	u8  HistoricalDataCount =0;//内存中保存历史数据数量     //历史数据上报相关
	u8  HistoricalDataSendBuff[100]={0};                    //历史数据上报相关
  u16 HistoricalDataRecevFlag=0;                          //历史数据上报相关
  u8  HistoricalDataSendCounter =0;                       //历史数据上报相关
	u8  i=0;
	u8  RouteControl =1;      //路由控制变量，默认使用数据集中器进行数据传输
	u8  CollectDataEnable =0; //数据采集使能变量，1允许数据采集，否则不允许	
	u32 CurrentTime_RTC =0;   //RTC当前时间
	u16 TimeDiff =0;          //时间差（单位：分钟），用于判断是否发生RTC唤醒，具体设置参考设备数据与不同设备之间的延时，如设为10分钟（+/-），
	                          //即，对于20秒设备延时来说最多支持10*3*2 =60台设备    
	u16 TimeGet =0;
		                                      
	u8  MonitorDeviceID[6] ={0x92,0x20,0x17,0x01,0x92,0x01}; //初始化设备ID号
  u16 NodeAddr =0x0000;
//  struct SenserData   sPerceptionData;       //传感器数据
  u8  OpTypeCode =0;                         //操作类型编码

	NodeAddr =MonitorDeviceID[4]*256 +MonitorDeviceID[5];     //提取设备ID号最后面两个字节作为节点地址
  PeripheralInit(MonitorDeviceID);                           //初始化外设
  if (PowerOffReset ==1)          
  {
    printf("\r\n掉电重启，重新初始化库仑计\r\n");                 //测试使用   
		Set_register_ds2780();    //掉电后对库仑计重新初始化
		Delay_ms(1000); 
	  for(g=0;g<3;g++)
		{
			ambient_temperature = get_temperature();
			printf("\n\r Temperature:%d (℃)\r\n",ambient_temperature);
			Delay_ms(1000); 
		}
		residual_capacity=get_ACR_capacity(ambient_temperature);
		set_ACR(residual_capacity);
		printf("\n\r ACR_capacity:%d mAh\r\n",residual_capacity);
		DS2780_CapacityInit();    //掉电后重新写电池容量
		DS2780_Test();            //掉电后将电池容量系数重新写入Flash
		printf("\r\n 掉电重启过程结束\r\n");
  
	  SX1287_Init(NodeAddr);    //433模块参数初始化                          
	  //////设备上电注册///////////
//		Delay_ms(2000);   
//		DeviceStartupRequest(Usart2_send_buff, DeviceID, NodeAddr);  
  }	
	
	for(i=0;i<20;i++)
	{
		 Delay_ms(200);                                //参考探头上电后数据稳定时间，作相应调整
		 if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
		 {
				DMA_UART3_RecevDetect(MonitorDeviceID, NodeAddr);  
				break;
		 }
  }	
	while(1)
  {
		
		WWDOG_Feed =0x1FFF;                              //窗口看门狗喂狗,定时4分20秒，第二字节约1秒变化一次，即0x09AF变到0x099F约耗时1秒		
		CurrentTime_RTC = RTC_GetCounter();                   //获取系统当前时间
		Time_Display(CurrentTime_RTC,&systmtime);  
	  
	  printf(" \r\n当前时间为: %d年 %d月 %d日   %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,    
		                             systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);            /* 输出公历时间 */
	                   
		//当定时时间到或者有查询数据时才采集数据并上报数据
//		TimeDiff =abs(systmtime.tm_hour*60 +systmtime.tm_min- DeviceConfig.CollectStartTime)%DeviceConfig.CollectPeriod;
		TimeGet =systmtime.tm_hour*60 +systmtime.tm_min;
		for(i=0;i<DeviceConfig.CollectNum;i++)
		{
       TimeDiff =abs(TimeGet -((DeviceConfig.CollectStartTime +DeviceConfig.CollectPeriod*i)%1440));
			 if(TimeDiff<RTC_WAKUP_GAP)
			 {
         CollectDataEnable =1;
       }
    }
   	CollectDataEnable =1;//测试使用
		
		if((CollectDataEnable ==1)||((DataRequestFlag &0x0F) ==0x01))
		{		
		    DeviceConfig.Time_Sec  =systmtime.tm_sec;     //记录数据采集时间
		    DeviceConfig.Time_Min  =systmtime.tm_min;
		    DeviceConfig.Time_Hour =systmtime.tm_hour;
		    DeviceConfig.Time_Mday =systmtime.tm_mday;	
		    DeviceConfig.Time_Mon  =systmtime.tm_mon;    
			  if(systmtime.tm_year >=2000)
				{
					 DeviceConfig.Time_Year =systmtime.tm_year-2000; //对上传年份去基数修正
				}
				else
				{
					 DeviceConfig.Time_Year =0;
        }
			  Sensor_Collect();                       //采集数据
    }
    else
		{
       DS2780_Test();                           //打印电池电量信息，便于现场诊断
       printf("\r\n433 Module Network ID is: %d (十进制)\r\n",GetNetworkID ()); //获取433模块网络ID，帮助问题定位 				
			 DeviceSleep(MonitorDeviceID[5]);
    }
		
		for(i=0;i<5;i++)
		{
			 Delay_ms(200);                                //参考探头上电后数据稳定时间，作相应调整
			 if(DMA_UART3_RECEV_FLAG==1)                   //查询数据接收情况
			 {
				 DMA_UART3_RecevDetect(MonitorDeviceID, NodeAddr);  
				 break;
			 }
    }	
		if((DataRequestFlag &0x0F) ==0x01)   // 获取数据查询状态，低4位
		{
       OpTypeCode =2;
			 RouteControl =DataRequestFlag>>4; // 获取路由标志变量，高4位
    }
		else
		{
       OpTypeCode =4;
    }				 
   //首先获取历史数据
		DataRead_From_Flash(0,10,0, &HistoricalDataCount,1);             //获取历史数据长度
		if((HistoricalDataCount!=0)&&(HistoricalDataCount<=100))
		{
       #if DEBUG_TEST	 
			 printf("\r\n Historical data length:%d!\r\n",HistoricalDataCount); //测试使用
			 #endif
			 DataRead_From_Flash(0,11,0, HistoricalDataSendBuff,HistoricalDataCount);             //获取历史数据数量
			 if(DeviceConfig.RetryNum>0)
	     {
         HistoricalDataSendCounter =DeviceConfig.RetryNum;
       }
			 else
			 {
					 HistoricalDataSendCounter =3;
			 }
			 for(i=0; i<HistoricalDataSendCounter; i++)
			 {
			    HistoricalDataRecevFlag =SendMessage(HistoricalDataSendBuff, HistoricalDataCount, MonitorDeviceID, NodeAddr);
				  if(HistoricalDataRecevFlag ==0x058C)     //成功接收到TrapResponse
					{
							#if DEBUG_TEST	 
							printf("\r\n Historical data upload success!\r\n"); //测试使用
							#endif
							break;     
				  }
			 }
    }
		
		TrapRequest(Usart2_send_buff, MonitorDeviceID, NodeAddr, OpTypeCode,RouteControl);  //发送最新数据
    		
		printf("\r\n433 Module Network ID is: %d (十进制)\r\n",GetNetworkID ()); //获取433模块网络ID，帮助问题定位 	
		DeviceSleep(MonitorDeviceID[5]);
//		gotoSleep(60*3);   //测试使用

	}  
}
#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{ 
  /* User can add his own implementation to report the file name and line number */
 
  printf("\n\r Wrong parameter value detected on\r\n");
  printf("       file  %s\r\n", file);
  printf("       line  %d\r\n", line);
    
  /* Infinite loop */
  /* while (1)
  {
  } */
}
#endif
/*********************************************END OF FILE***************************************/
