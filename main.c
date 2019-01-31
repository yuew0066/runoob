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
//#define  FILTER_ORDER    6                     //���������˲���ȣ����ڶ�����ƽ������,��ӦС��5

struct    rtc_time        systmtime;           //RTCʱ�����ýṹ��
struct    DeviceSet       DeviceConfig ={0x00};//Һλ��������Ϣ�ṹ��
struct    Config_RegPara  ConfigData   ={0x00};//������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
struct    DeviceStatus    SensorMonitorStatus  ={0x00};  //����Һλ������豸״̬
u16       WWDOG_Feed =0x1FFF;                  //���ڿ��Ź���λ����Ϊ��XX*1.8s = 7.6min
char      PowerOffReset =0;                    //����������־λ
u8        DataSendFlag =0;
//u8        DataCollectBkCount =0;               //�������ݲɼ�������
//u8        SensorDataCount = FILTER_ORDER;       //�������ݼ���������ʾ��ǰ�ɼ�����������
u8        DataCollectCount =1;                 //���ݲɼ�������
char      Usart1_recev_buff[100] ={'\0'};      //USART1���ջ���
u16       Usart1_recev_count =0;               //USART1���ͼ�����
u8        Uart4_rev_buff[100]={0x00};          //RS485���ڽ��ջ���
u8        Uart4_rev_count=0;                   //RS485���ڽ��ռ�����
u8        Uart4_rev_comflag=0;                 //�������״̬��־
vu8       Uart_rev_finish=0;                   //���ڽ�����ɱ�־����
//u32       WorkingTime =0;                    //�豸����ʹ�õ�ʱ��
u8        DMA_UART3_RECEV_FLAG =0;             //USART3 DMA���ձ�־����
u8        Usart2_send_buff[SENDBUFF_SIZE]={'\0'};       //USART2���ͻ���
u8        DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE] ={0x00};
static    struct DMA_USART3_RecevConfig  DMA_USART3_RecevIndicator; 
u8        DataRequestFlag =0;                  //���ݲ�ѯ��־������1��ʾ���յ���������ѯ����     
//u8        DatRev_OK =0 ;                       //�������ɹ��������ݱ�־����

extern    char   Usart3_recev_buff[RECEIVEBUFF_SIZE];
extern    u16    Usart3_recev_count;

/*
////volatile unsigned char Uart4_send_ready_flag=0;  //���;���״̬��־
////volatile unsigned char Crc_counter = 0;          //RS485У�������
////volatile unsigned char *Uart4_send_pointer = Uart4_send_buff;//RS485���ڷ���ָ��
//extern  char  Usart1_recev_buff[300];
////extern  char  Usart1_send_buff[];
//extern  uint16_t  Usart1_recev_count;
////extern  uint8_t  Usart1_send_count;
////extern u8  DataCollectCache[13][4];   //Һλ���ݲɼ����棬���13�����ݣ�������HEX��ʽ�洢�����ֽ���ǰ�����ֽ��ں�
//extern u8  DataCollectCount;          //���ݲɼ�������
////extern  float LevelData_Float[FILTER_ORDER];       //�ɼ�������ʱҺλ���ݣ�������
//uint32_t  time=0 ;                   // ms ��ʱ����  
//char      Usart1_send_buff[300]={'\0'};       //USART1���ͻ���
//uint8_t   Usart1_send_count=0;                 //USART1���ͼ�����
//uint32_t  Tic_IWDG=0;                //�������Ź�ι��ʱ������
//extern  char  Usart3_send_buff[];
//extern  uint8_t  Usart3_send_count;                     
//extern  struct  Config_RegPara   ConfigData;  //������·����ò�����HEX��ʽ��������ϵͳ��������ģʽ֮ǰд��BKP�Ĵ���
//extern  float  Data_Liquid_Level;
*/

//extern void     Delay(uint32_t nCount);         
extern void     RecvBuffInit_USART3(void);
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3�������ݼ�������ݽ���
int  DMA_UART3_RecevDataGet(void);
extern  u8 SensorStatusUpload(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress,struct DeviceStatus* pSensorStatus, struct rtc_time* pTime, u8 OpType);
void  DeviceSleep(u8 DeviceSleepDelay);
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : ��DMA���մ洢������ȡ��Ч���ݣ�����Usart3_recev_buff[],���ں������ݽ���
* Input          : None
* Output         : None
* Return         : �������ݳ���
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
//	 printf("\r\nDMA UART2 Recev Data Start Num:%d---End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //����ʹ��
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
//		 DMA_Cmd(DMA1_Channel6, DISABLE);           //�ر�DMA��ֹ�����ڼ�������
//		 USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);   //��USART2��������ǰ���ȴ�USART2���տ����жϣ����ڼ�����ݽ������
		 DataLength = DMA_UART3_RecevDataGet();
//		 DMA_UART3_RECEV_FLAG =0;
//		 DMA_Cmd(DMA1_Channel6, ENABLE);            //����DMA 
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);             //����ʹ��
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);           //����ʹ��
			  Usart3_recev_count =DataLength;                          //���ݳ��ȸ�ֵ����һ�����ݽ������õ���ȱ�����޷���������������Ż���Ϊ�������ݶ���ȫ�ֱ�����ʽ
			  for(i=0;i<DataLength;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);               //����ʹ��
        }
        StateFlag =Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //�������·����ݽ���    //
        //�Խ����������ͽ���ָʾ 		
		 }
		 else
		 {
        printf("\r\nNo data\r\n");
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //�ر�DMA��ֹ�����ڼ�������
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //��λDMA���ݽ���BUFF
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //����DMA 
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //��USART2��������ǰ���ȴ�USART2���տ����жϣ����ڼ�����ݽ������
  } 
	return StateFlag;
}


/*******************************************************************************
���ܣ�ȥ�������Сֵ����ʣ��������ƽ��
���룺���������飬������������
���أ�������ƽ��ֵ
��д��
��д���ڣ�XX��XX��XX��
�汾��v0.1
********************************************************************************/
//float GetAverageData(float* pSensorData, u8 SensorData_Count,float* pDiffer)
//{
//	float DataTemp[FILTER_ORDER] ={0.0};
//  float Temp = 0.0;
//  float AverageData = 0.0;
//  float Dvalue = 0.0;          //���ֵ����Сֵ֮��
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
////  	printf("\r\n**%f**\r\n##", DataTemp[i]);    //����ʹ��
//  }
//	
//  for(i=SensorData_Count-1;i>=1;i--)
//	{ 
//		for(j=SensorData_Count-1;j>=SensorData_Count-i;j--)    //��С����˳������
//		{
//       if(DataTemp[j] < DataTemp[j-1])
//			 {
//          Temp = DataTemp[j-1];
//				  DataTemp[j-1] = DataTemp[j];
//				  DataTemp[j] =  Temp;
//       }
//    }
//  }

//	Temp =0;                            //��λ�ۼ���
//	Dvalue =DataTemp[SensorData_Count-1]-DataTemp[0];
//	*pDiffer =Dvalue ;                   //���������С��ֵ
//	
//	if((Dvalue > 0.1) &&(SensorData_Count>=4))           //�����ֵ����Сֵ֮���10cmʱ��ȥ��2��ƫ��ϴ�ֵ
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
//   u8     TemperatureReadCmd[8]={0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};           //���¶�����ָ��,�����λΪCRCУ����
//   u8     TemperatureReadResp[3]={0x01,0x03,0x02};   //����Ӧ��ָʾ:01,03,02,T_H,T_L,CrcL,CrcH
//   u8*    pUart4Send =  TemperatureReadCmd;
//   u8*    pDataRead  =NULL; 
//   short  TempShort=0;           //���ڸ�ֵת��
//   float  Temper =0;             //�¶������ݴ�
//   float  DataTempArry[FILTER_ORDER] ={0};

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //���ձ�־������λ
//     Uart4_rev_count   =0;   //���ռ���������
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //������ݽ���BUFF
//		 
//     DIR485_Send();                           //485����ʹ��
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, TemperatureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //�ȴ�������� 
//		 }
//		 Delay_us(450);                  //�����ݽ�����ɺ��ӳ�һ��ʱ�����л��շ����ƿ��أ�Ĭ����ʱ350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //����ʹ��
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //����Ƿ��յ��¶ȴ�����Ӧ��
//				 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
//				 {
//           TempShort =pDataRead[3]*256 +pDataRead[4];
//					 DataTempArry[j]  =TempShort;
//					 DataTempArry[j] = DataTempArry[j]/10;		 
//					 
////					 DataTempArry[j] =(pDataRead[3]*256 +pDataRead[4])/10;
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Temperature is :%.1f ��\r\n",DataTempArry[j]);                 //����ʹ��
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
//  Temper = GetAverageData(DataTempArry, FILTER_ORDER);  //�˳������Сֵ��ȡ��ƽ�
//	printf("\r\nTemperature\r\n");                 //����ʹ���
//  for(i=0;i<FILTER_ORDER;i++)
//	{
//     printf("--%.1f--",DataTempArry[i]);                 //����ʹ��
//  }
//	return  Temper;		
//}
*/
/*******************************************************************************
* Function Name  : XX
* Description    : �����������д�IP68�¶ȴ��������ݲɼ�����Զ�Ժǰ�ڰ�װ�豸�����ݲɼ�ָ�����в���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//float Temperature_DataGet(u8*  SenserStatus)
//{
//   u8     i=0,j=0; 	
//   u8     TemperatureReadCmd[8]={0xC0,0x03,0x00,0x01,0x00,0x01,0xC5,0x1B};           //���¶�����ָ��,�����λΪCRCУ����
//   u8     TemperatureReadResp[3]={0xC0,0x03,0x02};   //����Ӧ��ָʾ:01,03,02,T_H,T_L,CrcL,CrcH
//   u8*    pUart4Send =  TemperatureReadCmd;
//   u8*    pDataRead  =NULL; 
//	 u8     RespCount  =0;
//   float  Temper =0;   //�¶������ݴ�
//   float  DataTempArry[FILTER_ORDER] ={0};
//   short  TempShort =0;
//   float  MaxDifference =0;  //���ֵ����Сֵ��ֵ

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //���ձ�־������λ
//     Uart4_rev_count   =0;   //���ռ���������
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //������ݽ���BUFF
//		 
//     DIR485_Send();                           //485����ʹ��
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, TemperatureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //�ȴ�������� 
//		 }
//		 Delay_us(300);                  //�����ݽ�����ɺ��ӳ�һ��ʱ�����л��շ����ƿ��أ�Ĭ����ʱ350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //����ʹ��
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //����Ƿ��յ��¶ȴ�����Ӧ��
//				 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
//				 {
//           TempShort =(pDataRead[3]*256 +pDataRead[4]);
//					 Temper =TempShort;
//					 DataTempArry[j] =Temper/10; 		 
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Temperature is :%f ��\r\n", DataTempArry[j]);                 //����ʹ��
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
//     *SenserStatus =0;    //������ڶ��������
//  }
//  Temper = GetAverageData(DataTempArry, RespCount, &MaxDifference);  //�˳������Сֵ��ȡ��ƽ��
//	Temper = Temper*10;        //���ȿ�����1λС������λ��
//	Temper =(int)Temper;       //���ȿ�����1λС������λ��
//	Temper =Temper/10;         //���ȿ�����1λС������λ��
//	
//	if((Temper<-20)||(Temper>200))                 //�¶ȴ��������ݳ����̱��쳣
//	{
//     *SenserStatus =2;
//  }
//	printf("\r\nTemperature\r\n");                 //����ʹ��
//  for(i=0;i<FILTER_ORDER;i++)
//	{
//     printf("--%.1f--",DataTempArry[i]);         //����ʹ��
//  }
//	return  Temper;		
//}

/*******************************************************************************
* Function Name  : XX
* Description    : �����������д�IP68ѹ�����������ݲɼ�
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//float Pressure_DataGet(u8*  SenserStatus)
//{
//   u8     i=0,j=0; 	
//   u8     PressureReadCmd[8]={0x01,0x04,0x00,0x02,0x00,0x02,0xD0,0x0B};           //��ѹ������ָ��,�����λΪCRCУ����
//   u8     PressureReadResp[3]={0x01,0x04,0x04};   //����Ӧ��ָʾ:01,04,04,P_1,P_2,P_3,P_4,CrcL,CrcH    //�����ʾ����λ�ֽ���ǰ����λ�ֽ��ں�
//   u8*    pUart4Send = PressureReadCmd;
//   u8*    pDataRead  =NULL; 
//   u8     RespCount =0;
////   short  TempShort=0;           //���ڸ�ֵת��
//   float  Pressure =0;   //ѹ�������ݴ�
//   float  DataTempArry[FILTER_ORDER] ={0};
//   union  Hfloat  PressureTemp;
//   float  MaxDifference =0;  //���ֵ����Сֵ��ֵ

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //���ձ�־������λ
//     Uart4_rev_count   =0;   //���ռ���������
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //������ݽ���BUFF
//		 
//     DIR485_Send();                           //485����ʹ��
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, PressureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //�ȴ�������� 
//		 }
//		 Delay_us(350);                  //�����ݽ�����ɺ��ӳ�һ��ʱ�����л��շ����ƿ��أ�Ĭ����ʱ350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //����ʹ��
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)PressureReadResp, sizeof(Uart4_rev_buff), sizeof(PressureReadResp));  //����Ƿ��յ��¶ȴ�����Ӧ��
//			 	 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-10)))
//				 {
//           for(i=0;i<4;i++)
//					 {			
//              PressureTemp.Data_Hex[3-i] =pDataRead[3+i];                     //��λ�ֽ���ǰ
////						  #if DEBUG_TEST
////							printf("\r\n--%x-- \r\n", pDataRead[3+i]);  //����ʹ��
////							#endif
//           }
//					 DataTempArry[j] =PressureTemp.Data_Float;
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Pressure is :%.3f kPa\r\n", DataTempArry[j]);  //����ʹ��
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
//      *SenserStatus =0;   //������ڶ��������
//   }
//   Pressure = GetAverageData(DataTempArry, RespCount, &MaxDifference);  //�˳������Сֵ��ȡ��ƽ��
//	 Pressure =(int)Pressure;                                //���ȿ�����3λС������λkPa
//	 Pressure =Pressure/1000;                                //���ȿ�����3λС������λMPa
//   printf("\r\nSoothData:%f MPa\r\n",Pressure);            //����ʹ��
//	 if((Pressure<-0.1)||(Pressure>1.6))                     //ѹ�����ݳ����̱��쳣
//	 {
//     *SenserStatus =2;
//   }
//	 if(Pressure<0)
//	 {
//      Pressure=0;                                      //��ֵ������ȥ����Ư��ɵ�Ӱ��
//   }
//	 printf("\r\nPressure\r\n");                           //����ʹ��
//   for(i=0;i<FILTER_ORDER;i++)
//	 {
//     printf("--%.3f--",DataTempArry[i]);                 //����ʹ��
//   }
//	 return  Pressure;		
//}		
		
/*******************************************************************************
* Function Name  : void SenserDataCollect1(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
* Description    : �������ϰ���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
//void SenserDataCollect1(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
//{
//			u32       TimCount_Current =0;
//			u8        i=0;   //����ʹ��

//			#if DEBUG_TEST
//			printf("DataCollectCount:%d\r\n",DataCollectCount);  //����ʹ��
//			#endif
//			
////			if(DataCollectCount == DeviceConfig.CollectNum)    //��һ�βɼ����ݣ���¼�ɼ�ʱ����Ϣ����������ÿ���Ƿ������ʵʱ��
//			{
//				pGetData->DataCount =0;                            //��Һλ���ݻ������������ֵ                       
//				TimCount_Current = RTC_GetCounter();
//				Time_Display(TimCount_Current,&systmtime); 
//						
//			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
//				DeviceConfig.Time_Min  =systmtime.tm_min;
//				DeviceConfig.Time_Hour =systmtime.tm_hour;
//			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
//			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
//			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //���ϴ����ȥ��������				
//    		#if DEBUG_TEST
//				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //����ʹ��
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //����ʹ��
//    		#endif
//			}		
//			i = pGetData->DataCount;
//    	pGetData->TemperatureData[i] = Temperature_DataGet();
//			Delay_ms(800);
//			pGetData->PressureData[i]    = Pressure_DataGet();
//			if(i==0)
//			{
//        pGetData->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);       //��һ�����ݲɼ�ʱ��
//      }
//		  else
//			{
//        pGetData->CollectTime[i] =(pGetData->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //����ÿ�����ݲɼ�ʱ��Ϊ��ǰһ�����ݻ��������Ӳɼ����
//      }
//			pGetData->DataCount =(pGetData->DataCount)+1;
//     	DataCollectCount--;	

//}
*/

/*******************************************************************************
* Function Name  : XX
* Description    : �������°���
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void Sensor_DataCollect( struct SenserData* pGetData, struct DeviceStatus* pMonitorStatus)
//{
//   	 u32       TimCount_Current =0;
//		 u8        i=0;                     //����ʹ��
////      char      PressureSensorZeroAdjust[8]={0x01,0x06,0x00,0x05,0x00,0x00,0x99,0xCB};
////      char      PressureSensorAccessCode[8]={0x01,0x06,0x00,0x0A,0x38,0x79,0x7B,0xEA};
//	   char*     pRecevBuff  =NULL; 
//		 u8        PressStatus =3;        //ѹ��������״̬��־���������ڼ�⴫�����Ƿ���������0��ʾ����������1��ʾʧЧ,3��ʾδ����
//     u8        TemperatueStatus =3;   //�¶ȴ�����״̬��־���������ڼ�⴫�����Ƿ���������0��ʾ����������1��ʾʧЧ,3��ʾδ����
//           
//		 TimCount_Current = RTC_GetCounter();
//		 Time_Display(TimCount_Current,&systmtime); 				
//		 DeviceConfig.Time_Sec  =systmtime.tm_sec;
//		 DeviceConfig.Time_Min  =systmtime.tm_min;
//		 DeviceConfig.Time_Hour =systmtime.tm_hour;
//		 DeviceConfig.Time_Mday =systmtime.tm_mday;	
//		 DeviceConfig.Time_Mon  =systmtime.tm_mon;                                
//		 DeviceConfig.Time_Year =systmtime.tm_year-2000; //���ϴ����ȥ��������
// 		                                     		
//     #if DEBUG_TEST
//		 printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //����ʹ��
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //����ʹ��
//     #endif
//			
//		 //�ȿ�����ѹ���������ɼ���ѹʽ���������ݣ�
//     PowerON_485();             //����485ģ���Դ
//		 Delay_ms(500);  	
//		 PowerON_12V();	            //����12V�ܵ�Դ
//		 #if DEBUG_TEST
//		 printf("\n\r***----12V DC-DC Power ON----***\r\n");   //����ʹ��
//		 #endif
//		 Delay_ms(1000);     
//		 PowerON_Pressure();        //������ѹ��������Դ
//		 #if DEBUG_TEST
//		 printf("\n\r***----Pressure Sensor Power ON----***\r\n");   //����ʹ��
//		 #endif
//		 for(i=0;i<80;i++)          //������ʱ��ʹ��ѹ̽ͷ�ȶ���ͬʱ��⴮�ڿ�����������
//		 {
//				Delay_ms(100);   

//				pRecevBuff = Find_SpecialString(Usart1_recev_buff, "BATT Recover" ,sizeof(Usart1_recev_buff), 12);  //����Ƿ��յ���ѹ�������������������ڴ��������У׼
//				if(pRecevBuff !=NULL)
//				{
//						DataRead_From_Flash(0,9,0, &i,1);                //��Flash�ж����������ϵ����������
//						if(i<=113)
//						{
//							 DataWrite_To_Flash(0,1,0,  &i,1);             //�����������ݸ��µ��������ϵ�����ݴ洢��
//							 #if DEBUG_TEST
//							 printf("\r\n------------BATT Recover Sucess!------------\r\n");                 //����ʹ��
//						   #endif
//						}
//						#if DEBUG_TEST
//						printf("\r\n------------Warning:BATT Recover Data too big!------------\r\n");      //����ʹ��
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
//		 PowerOFF_Pressure();                     //�رվ�ѹ��������Դ
//		 #if DEBUG_TEST
//		 printf("\n\r***----Pressure Sensor Power OFF----***\r\n");   //����ʹ��
//		 #endif
//		 Delay_ms(500);
//		 PowerON_TemperatureSensor();             //���¶ȴ�������Դ
//		 #if DEBUG_TEST
//		 printf("\n\r***----Temperature Sensor Power ON----***\r\n");   //����ʹ��
//		 #endif
//		 Delay_ms(3000);
//		 pGetData->TemperatureData[0] = Temperature_DataGet(&TemperatueStatus);
//		 pMonitorStatus->TemperatureSensorStatus =TemperatueStatus;
//		 Delay_ms(100);
//		 PowerOFF_TemperatureSensor();            //�ر��¶ȴ�������Դ
//		 #if DEBUG_TEST
//		 printf("\n\r***----Temperature Sensor Power OFF----***\r\n");   //����ʹ��
//		 #endif
//		 PowerOFF_12V();
//		 #if DEBUG_TEST
//		 printf("\n\r***----12V DC-DC Power OFF----***\r\n");   //����ʹ��
//		 #endif
//		 Delay_ms(100);
//		 PowerOFF_485();                          //��Һλ���ݲɼ����ʱ���ر�Uartת485ģ���Դ
//		 pGetData->CollectTime[0] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);       //���ݲɼ�ʱ��

//	
//}


/*******************************************************************************
* Function Name  :Power_SX1278_Init()
* Description    : ��ʼ��433ģ��
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
* Description    : ��ʼ��433ģ��
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
	////////////////////����433ģ����� //////////////////// 
//	SetNodeSerialPort(4, 0);                           //����433ģ�鴮�ڲ����ʣ�9600,У������:��У��
	SetNodeCentralFrequency(439);                      //����433ģ���ز�����Ƶ�ʣ�434MHz
	SetNodeFrequencyExpandFactor(11);                  //����433ģ����Ƶ���ӣ�2048
	SetNodeFrequencyExpandBandwidth(7);                //����433ģ����Ƶ����125K
	SetNodeWorkMode(2);                                //����433ģ�鹤��ģʽ���ڵ�ģʽ
	SetNodeID (sNodeID);                               //����433ģ��ID������ڵ�ID
	SetNodeNetworkID (66);                              //����433ģ������ID��19
	SetNodeSendPowerGrade (7);                         //����433ģ�鷢�书�ʵȼ���20dBm
	SetNodeBreathPeriod (0);                           //����433ģ���������:2s
	SetNodeBreathTime (5);                             //����433ģ�����ʱ�䣺64ms
	
	PowerOFF_433();                                   //����433ģ�飬ʹ������Ч        
	Delay_ms(5000);
	PowerON_433(); 
	////////////////////��ȡ433ģ����� //////////////////// 
//u8  GetNodeReceiveSignalEnergy ();                 //��ȡ433ģ����һ֡���ݽ����ź�ǿ��
  UartConfig =GetNodeSerialPortConfig();              //��ȡ433ģ�鴮�����ò���
  BaudRate   =UartConfig>>8;
  VerifyType =UartConfig &0xFF;
  printf("\r\nBaud Rate:%d---Verify Type:%d\r\n",BaudRate,VerifyType);//����ʹ��
  CentralFreq =GetNodeCentralFrequency ();               //��ȡ433ģ���ز�Ƶ�ʲ���
  printf("\r\nCentral Frequency :%d MHz\r\n",(CentralFreq+1));   //����ʹ��
  ExpFactor   =GetNodeFrequencyExpandFactor();           //��ȡ433ģ����Ƶ���Ӳ���
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
  printf("\r\nNode Frequency Expand Factor:%d \r\n",Temp);   //����ʹ��
  ExpBW =GetNodeFrequencyExpandBandwidth ();                 //��ȡ433ģ����Ƶ�������
	switch( ExpBW )
  {
			case 6:  
			{
				Temp=63;       //62.5Լ����63
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
	printf("\r\nNode Frequency Expand Bandwidth:%dKHz\r\n",Temp);   //����ʹ��
  WorkMode = GetNodeWorkMode ();                                  //��ȡ433ģ�鹤��ģʽ����
	switch( WorkMode )
  {
			case 0:  
			{
	      printf("\r\n433 Module Work Mode is: Standard\r\n");   //����ʹ��
				break;
			}
			case 1:  
			{
				printf("\r\n433 Module Work Mode is: Center\r\n");    //����ʹ��
				break;
			}
			case 2:  
			{
				printf("\r\n433 Module Work Mode is: Node\r\n");    //����ʹ��
				break;
			}		
			default:
			{
				printf("\r\n433 Module Work Mode is: Unknown\r\n");    //����ʹ��
				break;
			}		
  }
  NodeID =GetNodeID ();                                 //��ȡ433ģ��ڵ�ID
	printf("\r\n433 Module Node ID is: %x\r\n",NodeID);   //����ʹ��
  NetID =GetNetworkID ();                               //��ȡ433ģ������ID
	printf("\r\n433 Module Network ID is: %x\r\n",NetID); //����ʹ��
  PowerGrade = GetNodeSendPowerGrade ();                //��ȡ433ģ�鷢�书��
	printf("\r\n433 Module Send Power Grade is: %d\r\n",PowerGrade); //����ʹ��
  BreathPeriod = GetNodeBreathPeriod ();                //��ȡ433ģ���������
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
	printf("\r\nNode Breath Period:%d s\r\n",Temp );   //����ʹ��
  WakeTime  =  GetNodeBreathTime ();                 //��ȡ433ģ�����ʱ��
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
	printf("\r\nNode Wake Time:%d ms\r\n",Temp );   //����ʹ��

}
/*******************************************************************************
* Function Name  : XXX
* Description    : �豸��������״̬
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DeviceSleep(u8 DeviceSleepDelay)
{
  u16 WakeupTime  =0;  //��һ�λ���ʱ��
	u32 SleepTime   =0;  //�����豸����ʱ��	
	u32 CurrentTime =0;  //RTC��ǰʱ��
	u32 RelativeCurrentTime =0;  //��ǰʱ��(���ֵ�����ܳ���1440)
	u8  i=0;
	
  CurrentTime = RTC_GetCounter();                   //��ȡϵͳ��ǰʱ��
  Time_Display(CurrentTime,&systmtime);    

  CurrentTime = RTC_GetCounter();
	Time_Display(CurrentTime,&systmtime); 
	CurrentTime =	systmtime.tm_hour*60+systmtime.tm_min;
	if((DeviceConfig.CollectStartTime + DeviceConfig.CollectPeriod* (DeviceConfig.CollectNum-1))<1440)            
	{ 
		  if(CurrentTime <DeviceConfig.CollectStartTime)
			{
        WakeupTime =DeviceConfig.CollectStartTime ;     //��һ���豸����ʱ��,�Է���Ϊ��λ��ÿ��24Сʱ��0~1440 
				SleepTime  =(WakeupTime-CurrentTime)*60;        //��λ��
      }
			else if(CurrentTime >(DeviceConfig.CollectStartTime+ (DeviceConfig.CollectNum-1)*DeviceConfig.CollectPeriod))
			{
        WakeupTime =DeviceConfig.CollectStartTime ;//��һ���豸����ʱ��,�Է���Ϊ��λ��ÿ��24Сʱ��0~1440
				SleepTime  =(WakeupTime +1440 -CurrentTime)*60;  //��λ��   
      }
			else
			{
        for(i=0;i<(DeviceConfig.CollectNum-1);i++)
				{
	        if((CurrentTime>=(DeviceConfig.CollectStartTime+ i*DeviceConfig.CollectPeriod))&&(CurrentTime<=(DeviceConfig.CollectStartTime+ (i+1)*DeviceConfig.CollectPeriod)))  
			    {  
				    WakeupTime =DeviceConfig.CollectStartTime +(i+1)*DeviceConfig.CollectPeriod; //��һ���豸����ʱ��,�Է���Ϊ��λ��ÿ��24Сʱ��0~1440
				    SleepTime  =(WakeupTime-CurrentTime)*60;        //��λ��
						break;
          }
        }
      }
//		  printf("\r\n---1--DeviceConfig.CollectNum:%d-----CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,CurrentTime,WakeupTime);                 //����ʹ��
			printf("\r\nDeviceConfig.CollectNum:%d--%d--%d------CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,DeviceConfig.CollectPeriod,DeviceConfig.CollectStartTime,CurrentTime,WakeupTime);                 //����ʹ��
				
			DeviceSleepDelay =DeviceSleepDelay %DEVICE_DELAY_MAX;  //��ֹ�������
			SleepTime =SleepTime+ DeviceSleepDelay*20;             //�����豸��Ų�ͬ������Ӧ��ʱ������ͨ��433�����ϴ�����ʱ������ײ����λ��
		  gotoSleep(SleepTime);
	}
	else
	{   
			if((CurrentTime <DeviceConfig.CollectStartTime)&&(CurrentTime >((DeviceConfig.CollectStartTime + DeviceConfig.CollectPeriod* (DeviceConfig.CollectNum-1))%1440)))  
			{
        WakeupTime =DeviceConfig.CollectStartTime ;     //��һ���豸����ʱ��,�Է���Ϊ��λ��ÿ��24Сʱ��0~1440 
				SleepTime  =(WakeupTime-CurrentTime)*60;        //��λ��
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
							 WakeupTime =(DeviceConfig.CollectStartTime +(i+1)*DeviceConfig.CollectPeriod); //��һ���豸����ʱ��,�Է���Ϊ��λ��ÿ��24Сʱ��0~1440
							 SleepTime  =(WakeupTime-RelativeCurrentTime)*60;        //��λ��
//							 printf("\r\n---2--\r\n");
							 break;
						}
//						printf("\r\n---0--\r\n");			
        }
//				printf("\r\n---3--\r\n");
      }
		  printf("\r\nDeviceConfig.CollectNum:%d--%d--%d------CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,DeviceConfig.CollectPeriod,DeviceConfig.CollectStartTime,CurrentTime,WakeupTime);                 //����ʹ��
			//�����豸��Ų�ͬ������Ӧ��ʱ������ͨ��433�����ϴ�����ʱ������ײ����λ��
			DeviceSleepDelay =DeviceSleepDelay %DEVICE_DELAY_MAX; //��ֹ�������
			SleepTime =SleepTime+ DeviceSleepDelay*20;       //����Ƴ�20s*ID
	
			
//			if(DeviceSleepDelay%2 ==0)   //ż��
//			{
//        SleepTime =SleepTime+ DeviceSleepDelay*20;       //ż������Ƴ�20s
//      }
//			else                         //����
//			{
//				 if(SleepTime >DeviceSleepDelay)
//				 {
//             SleepTime =SleepTime -DeviceSleepDelay*20;  //������ǰ20s
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
* Description    : ��ʼ���˿ڼ�����
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit(u8* pDevID)
{
//	u8    SWD_STATUS   =0;
//	u8    TempArray2[2]={0};
//	u16   ShortTemp2   =0;
	
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//���ȿ������ڿ��Ź���������ֵΪ7f,���ڼĴ���Ϊ5f,��Ƶ��Ϊ8	
	USART1_Config();      /* USART1 ����ģʽΪ 9600 8-N-1��  �жϽ��� */
	USART2_Config();      /* USART2 ����ģʽΪ 9600 8-N-1��  �жϽ��� */
	USART3_Config();      /* USART3 ����ģʽΪ 9600 8-N-1��  �жϽ��� */
//	USART4_Config();      /* UART4  ����ģʽΪ 9600 8-N-1��  �жϽ��� */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();
  USART2_DMA_Config();
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //���ô���DMA1����
//  TIM2_Configuration();       /* ��ʱ��TIM2�������� */	
//	TIM2_NVIC_Configuration();  /* ���ö�ʱ��TIM2���ж����ȼ� */
//	TIM3_Configuration();       /* ��ʱ��TIM3�������� */	
//	TIM3_NVIC_Configuration();  /* ���ö�ʱ��TIM3���ж����ȼ� */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //��ʱ�رն�ʱ��TIM3
	
	RTC_NVIC_Config();                 /* ����RTC���ж����ȼ� */
	RTC_CheckAndConfig(&systmtime);
//	Time_Show(&systmtime);             /* Display time in infinite loop */
//	WorkingTime =RTC_GetCounter();     //�ɼ��豸����ʱ��
  SysTick_Init();
  PowerON_Flash();                     //��Flash��Դ 
  Delay_ms(100); 
//	SPI_FLASH_Init();
//	PowerON_GPRS();                    //��GPRSģ���Դ��****����򿪣�����ԭ��;������·�庸����ȷ�Ժ�˲������ʡ��
//	Delay_ms(100);
//	PowerON_TemperatureSensor();       //�򿪳�����̽ͷ��Դ����TCP������ȷ�����Ժ��ٿ���̽ͷ��Դ�����������Ҫ������ʷ���ݣ�����Ҫ����Ӧ�޸�
//	Delay_ms(100);   
//  PowerON_485();                     //��485��Դ
//	Delay_ms(100);
//���Һλ�Ƹ�����ʱ���ڵ�����,�����Ƿ��б�Ҫ //��ʱʹ��
//	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));    //��ʱʹ��
	
 	DMA_USART3_RecevIndicator.CurrentDataStartNum =0;  //��ʼ����ǰ�������ݿ�ʼλ��
	DMA_USART3_RecevIndicator.CurrentDataEndNum =0;    //��ʼ����ǰ�������ݽ���λ��
	DMA_USART3_RecevIndicator.NextDataStartNum =0;     //��ʼ����һ�ν������ݿ�ʼλ��
	DMA_USART3_RecevIndicator.DMA_RecevCount =0;
	///////////////////���뽫ģ�����óɽ���״̬�����򴮿ڽ��ղ�������
	HIGH_433_SET();              //433ģ��SET�ܽ����ߣ��л�������ģʽ
	Delay_ms(100);
  LOW_433_EN();                //433ģ��EN�ܽ����ͣ��л�������ģʽ
	Delay_ms(100);
	
	USART_GetFlagStatus(USART1,USART_FLAG_TC);       //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR,��ֹ���ݷ���ʱ���ֽڱ�����
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR,��ֹ���ݷ���ʱ���ֽڱ�����
//	USART_GetFlagStatus(UART4,USART_FLAG_TC);        //����Ӳ����λ֮�󣬷������ֽ�֮ǰ���ȶ�һ��USART_SR,��ֹ���ݷ���ʱ���ֽڱ�����
	Uart_rev_finish=0;
	
	#if DEBUG_TEST
	printf("\n\r****************************************************************\r\n");   //����ʹ��
	printf("\n\r*-----��ӭʹ��BIRMM-CorrE-A���²��¸�ʴ���������!-------------*\r\n");   //����ʹ��
	printf("\n\r*-----Ӳ���汾�ţ�BIRMM-CorrE V1.1---------------------------------*\r\n");   //����ʹ��
  printf("\n\r*-----����汾�ţ�BIRMM-CorrE-A_V1.1-------------------------------*\r\n");   //����ʹ��
	printf("\n\r*-----�豸��ţ�%.2x%.2x%.2x%.2x%.2x%.2x-----------------------------------*\r\n",pDevID[0],pDevID[1],pDevID[2],pDevID[3],pDevID[4],pDevID[5]);   //����ʹ��
	printf("\n\r*-----��Ȩ���У��й�����ƹ���Ժ�ǻ۹��������о��뷢չ����-----*\r\n");   //����ʹ��
	printf("\n\r****************************************************************\r\n");   //����ʹ��
	#endif
/*	
	//////////������ѹ̽ͷ�ۼƶ���ʧ�ܴ�����������̽ͷ�ۼƶ���ʧ�ܴ�����433ģ�����ݴ����ۼ�ʧ�ܴ���
//  DataRead_From_Flash(1,4,0, TempArray2,2);            //��Flash�ж�����ѹ���������ݲɼ�ʧ�ܴ���
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****Press senser data collect fail counter:%3d -----****\r\n",ShortTemp2);   //����ʹ��
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,5,0, TempArray2,2);            //��Flash�ж������������������ݲɼ�ʧ�ܴ���
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****Ultrasonic senser data collect fail counter:%3d ****\r\n",ShortTemp2);   //����ʹ��
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,6,0, TempArray2,2);            //��Flash�ж���3Gģ�鲦��ʧ�ܴ���
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****433 Modle Communication fail counter:%3d -------****\r\n",ShortTemp2);   //����ʹ��
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,7,0, TempArray2,2);            //��Flash�ж���3Gģ����һ��ͨ���ź�ǿ��
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****433 Modle latest receive signal density:%3d dBm  ***\r\n",(short)ShortTemp2);   //����ʹ��
//	printf("\n\r********************************************************\r\n");                     //����ʹ��
//	#endif
//	
//	DataRead_From_Flash(1,0,0, &SWD_STATUS,1);     //��Flash�ж�ȡSWD�˿ڸ���״̬��־
//	if(SWD_STATUS==1)                     
//	{
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE); 
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);    //��ֹSWD�ӿ����ع��ܣ�����GPIO����ͨ��������λ���س������ͨ�������·�ָ�����ռ��;
//		#if DEBUG_TEST
//		printf("\n\r<<<<!!!!!!!!!!!----SWD-Lock----!!!!!!!!!!!>>>>\r\n");   //����ʹ��
//		#endif		
//	}
//	else
//	{
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);  
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
//		#if DEBUG_TEST
//		printf("\n\r<<<<!!!!!!!!!!!----SWD-Unlock----!!!!!!!!!!!>>>>\r\n");   //����ʹ��
//		#endif
//	}
*/
	ConfigData_Init(&DeviceConfig);         //��Flash�ж�ȡ���ò���

}


/*******************************************************************************
* Function Name  : int main(void)
* Description    : ������
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
	u8  HistoricalDataCount =0;//�ڴ��б�����ʷ��������     //��ʷ�����ϱ����
	u8  HistoricalDataSendBuff[100]={0};                    //��ʷ�����ϱ����
  u16 HistoricalDataRecevFlag=0;                          //��ʷ�����ϱ����
  u8  HistoricalDataSendCounter =0;                       //��ʷ�����ϱ����
	u8  i=0;
	u8  RouteControl =1;      //·�ɿ��Ʊ�����Ĭ��ʹ�����ݼ������������ݴ���
	u8  CollectDataEnable =0; //���ݲɼ�ʹ�ܱ�����1�������ݲɼ�����������	
	u32 CurrentTime_RTC =0;   //RTC��ǰʱ��
	u16 TimeDiff =0;          //ʱ����λ�����ӣ��������ж��Ƿ���RTC���ѣ��������òο��豸�����벻ͬ�豸֮�����ʱ������Ϊ10���ӣ�+/-����
	                          //��������20���豸��ʱ��˵���֧��10*3*2 =60̨�豸    
	u16 TimeGet =0;
		                                      
	u8  MonitorDeviceID[6] ={0x92,0x20,0x17,0x01,0x92,0x01}; //��ʼ���豸ID��
  u16 NodeAddr =0x0000;
//  struct SenserData   sPerceptionData;       //����������
  u8  OpTypeCode =0;                         //�������ͱ���

	NodeAddr =MonitorDeviceID[4]*256 +MonitorDeviceID[5];     //��ȡ�豸ID������������ֽ���Ϊ�ڵ��ַ
  PeripheralInit(MonitorDeviceID);                           //��ʼ������
  if (PowerOffReset ==1)          
  {
    printf("\r\n�������������³�ʼ�����ؼ�\r\n");                 //����ʹ��   
		Set_register_ds2780();    //�����Կ��ؼ����³�ʼ��
		Delay_ms(1000); 
	  for(g=0;g<3;g++)
		{
			ambient_temperature = get_temperature();
			printf("\n\r Temperature:%d (��)\r\n",ambient_temperature);
			Delay_ms(1000); 
		}
		residual_capacity=get_ACR_capacity(ambient_temperature);
		set_ACR(residual_capacity);
		printf("\n\r ACR_capacity:%d mAh\r\n",residual_capacity);
		DS2780_CapacityInit();    //���������д�������
		DS2780_Test();            //����󽫵������ϵ������д��Flash
		printf("\r\n �����������̽���\r\n");
  
	  SX1287_Init(NodeAddr);    //433ģ�������ʼ��                          
	  //////�豸�ϵ�ע��///////////
//		Delay_ms(2000);   
//		DeviceStartupRequest(Usart2_send_buff, DeviceID, NodeAddr);  
  }	
	
	for(i=0;i<20;i++)
	{
		 Delay_ms(200);                                //�ο�̽ͷ�ϵ�������ȶ�ʱ�䣬����Ӧ����
		 if(DMA_UART3_RECEV_FLAG==1)                   //��ѯ���ݽ������
		 {
				DMA_UART3_RecevDetect(MonitorDeviceID, NodeAddr);  
				break;
		 }
  }	
	while(1)
  {
		
		WWDOG_Feed =0x1FFF;                              //���ڿ��Ź�ι��,��ʱ4��20�룬�ڶ��ֽ�Լ1��仯һ�Σ���0x09AF�䵽0x099FԼ��ʱ1��		
		CurrentTime_RTC = RTC_GetCounter();                   //��ȡϵͳ��ǰʱ��
		Time_Display(CurrentTime_RTC,&systmtime);  
	  
	  printf(" \r\n��ǰʱ��Ϊ: %d�� %d�� %d��   %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,    
		                             systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);            /* �������ʱ�� */
	                   
		//����ʱʱ�䵽�����в�ѯ����ʱ�Ųɼ����ݲ��ϱ�����
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
   	CollectDataEnable =1;//����ʹ��
		
		if((CollectDataEnable ==1)||((DataRequestFlag &0x0F) ==0x01))
		{		
		    DeviceConfig.Time_Sec  =systmtime.tm_sec;     //��¼���ݲɼ�ʱ��
		    DeviceConfig.Time_Min  =systmtime.tm_min;
		    DeviceConfig.Time_Hour =systmtime.tm_hour;
		    DeviceConfig.Time_Mday =systmtime.tm_mday;	
		    DeviceConfig.Time_Mon  =systmtime.tm_mon;    
			  if(systmtime.tm_year >=2000)
				{
					 DeviceConfig.Time_Year =systmtime.tm_year-2000; //���ϴ����ȥ��������
				}
				else
				{
					 DeviceConfig.Time_Year =0;
        }
			  Sensor_Collect();                       //�ɼ�����
    }
    else
		{
       DS2780_Test();                           //��ӡ��ص�����Ϣ�������ֳ����
       printf("\r\n433 Module Network ID is: %d (ʮ����)\r\n",GetNetworkID ()); //��ȡ433ģ������ID���������ⶨλ 				
			 DeviceSleep(MonitorDeviceID[5]);
    }
		
		for(i=0;i<5;i++)
		{
			 Delay_ms(200);                                //�ο�̽ͷ�ϵ�������ȶ�ʱ�䣬����Ӧ����
			 if(DMA_UART3_RECEV_FLAG==1)                   //��ѯ���ݽ������
			 {
				 DMA_UART3_RecevDetect(MonitorDeviceID, NodeAddr);  
				 break;
			 }
    }	
		if((DataRequestFlag &0x0F) ==0x01)   // ��ȡ���ݲ�ѯ״̬����4λ
		{
       OpTypeCode =2;
			 RouteControl =DataRequestFlag>>4; // ��ȡ·�ɱ�־��������4λ
    }
		else
		{
       OpTypeCode =4;
    }				 
   //���Ȼ�ȡ��ʷ����
		DataRead_From_Flash(0,10,0, &HistoricalDataCount,1);             //��ȡ��ʷ���ݳ���
		if((HistoricalDataCount!=0)&&(HistoricalDataCount<=100))
		{
       #if DEBUG_TEST	 
			 printf("\r\n Historical data length:%d!\r\n",HistoricalDataCount); //����ʹ��
			 #endif
			 DataRead_From_Flash(0,11,0, HistoricalDataSendBuff,HistoricalDataCount);             //��ȡ��ʷ��������
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
				  if(HistoricalDataRecevFlag ==0x058C)     //�ɹ����յ�TrapResponse
					{
							#if DEBUG_TEST	 
							printf("\r\n Historical data upload success!\r\n"); //����ʹ��
							#endif
							break;     
				  }
			 }
    }
		
		TrapRequest(Usart2_send_buff, MonitorDeviceID, NodeAddr, OpTypeCode,RouteControl);  //������������
    		
		printf("\r\n433 Module Network ID is: %d (ʮ����)\r\n",GetNetworkID ()); //��ȡ433ģ������ID���������ⶨλ 	
		DeviceSleep(MonitorDeviceID[5]);
//		gotoSleep(60*3);   //����ʹ��

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
