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
//#define  FILTER_ORDER    6                     //¶¨ÒåÊı¾İÂË²¨Éî¶È£¬ÓÃÓÚ¶ÔÊı¾İÆ½»¬´¦Àí,²»Ó¦Ğ¡ÓÚ5

struct    rtc_time        systmtime;           //RTCÊ±ÖÓÉèÖÃ½á¹¹Ìå
struct    DeviceSet       DeviceConfig ={0x00};//ÒºÎ»¼ÆÅäÖÃĞÅÏ¢½á¹¹Ìå
struct    Config_RegPara  ConfigData   ={0x00};//¶¨ÒåµÄÏÂ·¢ÅäÖÃ²ÎÊı£¬HEX¸ñÊ½£¬·½±ãÔÚÏµÍ³½øÈëĞİÃßÄ£Ê½Ö®Ç°Ğ´ÈëBKP¼Ä´æÆ÷
struct    DeviceStatus    SensorMonitorStatus  ={0x00};  //¶¨ÒåÒºÎ»¼à²âÒÇÉè±¸×´Ì¬
u16       WWDOG_Feed =0x1FFF;                  //´°¿Ú¿´ÃÅ¹·¸´Î»ÖÜÆÚÎª£ºXX*1.8s = 7.6min
char      PowerOffReset =0;                    //µôµçÖØÆô±êÖ¾Î»
u8        DataSendFlag =0;
//u8        DataCollectBkCount =0;               //±¸·İÊı¾İ²É¼¯¼ÆÊıÆ÷
//u8        SensorDataCount = FILTER_ORDER;       //´«¸ĞÊı¾İ¼ÆÊıÆ÷£¬±êÊ¾µ±Ç°²É¼¯µ½Êı¾İÊıÁ¿
u8        DataCollectCount =1;                 //Êı¾İ²É¼¯¼ÆÊıÆ÷
char      Usart1_recev_buff[100] ={'\0'};      //USART1½ÓÊÕ»º´æ
u16       Usart1_recev_count =0;               //USART1·¢ËÍ¼ÆÊıÆ÷
u8        Uart4_rev_buff[100]={0x00};          //RS485´®¿Ú½ÓÊÕ»º´æ
u8        Uart4_rev_count=0;                   //RS485´®¿Ú½ÓÊÕ¼ÆÊıÆ÷
u8        Uart4_rev_comflag=0;                 //½ÓÊÕÍê³É×´Ì¬±êÖ¾
vu8       Uart_rev_finish=0;                   //´®¿Ú½ÓÊÕÍê³É±êÖ¾±äÁ¿
//u32       WorkingTime =0;                    //Éè±¸ÔËĞĞÊ¹ÓÃµÄÊ±¼ä
u8        DMA_UART3_RECEV_FLAG =0;             //USART3 DMA½ÓÊÕ±êÖ¾±äÁ¿
u8        Usart2_send_buff[SENDBUFF_SIZE]={'\0'};       //USART2·¢ËÍ»º´æ
u8        DMA_USART3_RecevBuff[RECEIVEBUFF_SIZE] ={0x00};
static    struct DMA_USART3_RecevConfig  DMA_USART3_RecevIndicator; 
u8        DataRequestFlag =0;                  //Êı¾İ²éÑ¯±êÖ¾±äÁ¿£¬1±íÊ¾½ÓÊÕµ½·şÎñÆ÷²éÑ¯ÃüÁî     
//u8        DatRev_OK =0 ;                       //·şÎñÆ÷³É¹¦½ÓÊÕÊı¾İ±êÖ¾±äÁ¿

extern    char   Usart3_recev_buff[RECEIVEBUFF_SIZE];
extern    u16    Usart3_recev_count;

/*
////volatile unsigned char Uart4_send_ready_flag=0;  //·¢ËÍ¾ÍĞ÷×´Ì¬±êÖ¾
////volatile unsigned char Crc_counter = 0;          //RS485Ğ£Ñé¼ÆÊıÆ÷
////volatile unsigned char *Uart4_send_pointer = Uart4_send_buff;//RS485´®¿Ú·¢ËÍÖ¸Õë
//extern  char  Usart1_recev_buff[300];
////extern  char  Usart1_send_buff[];
//extern  uint16_t  Usart1_recev_count;
////extern  uint8_t  Usart1_send_count;
////extern u8  DataCollectCache[13][4];   //ÒºÎ»Êı¾İ²É¼¯»º´æ£¬×î¶à13×éÊı¾İ£»¸¡µãÊıHEX¸ñÊ½´æ´¢£¬µÍ×Ö½ÚÔÚÇ°£¬¸ß×Ö½ÚÔÚºó
//extern u8  DataCollectCount;          //Êı¾İ²É¼¯¼ÆÊıÆ÷
////extern  float LevelData_Float[FILTER_ORDER];       //²É¼¯µ½µÄÁÙÊ±ÒºÎ»Êı¾İ£¬¸¡µãĞÍ
//uint32_t  time=0 ;                   // ms ¼ÆÊ±±äÁ¿  
//char      Usart1_send_buff[300]={'\0'};       //USART1·¢ËÍ»º´æ
//uint8_t   Usart1_send_count=0;                 //USART1·¢ËÍ¼ÆÊıÆ÷
//uint32_t  Tic_IWDG=0;                //¶ÀÁ¢¿´ÃÅ¹·Î¹¹·Ê±¼äÉèÖÃ
//extern  char  Usart3_send_buff[];
//extern  uint8_t  Usart3_send_count;                     
//extern  struct  Config_RegPara   ConfigData;  //¶¨ÒåµÄÏÂ·¢ÅäÖÃ²ÎÊı£¬HEX¸ñÊ½£¬·½±ãÔÚÏµÍ³½øÈëĞİÃßÄ£Ê½Ö®Ç°Ğ´ÈëBKP¼Ä´æÆ÷
//extern  float  Data_Liquid_Level;
*/

//extern void     Delay(uint32_t nCount);         
extern void     RecvBuffInit_USART3(void);
u16  DMA_UART3_RecevDetect(u8* pDeviceID, u16 sNodeAddress);     //USART3½ÓÊÕÊı¾İ¼à²âÓëÊı¾İ½âÎö
int  DMA_UART3_RecevDataGet(void);
extern  u8 SensorStatusUpload(u8* pSendBuff, u8* pDeviceID, u16 NodeAddress,struct DeviceStatus* pSensorStatus, struct rtc_time* pTime, u8 OpType);
void  DeviceSleep(u8 DeviceSleepDelay);
/*******************************************************************************
* Function Name  : int  DMA_UART3_RecevDataGet(void)
* Description    : ´ÓDMA½ÓÊÕ´æ´¢Æ÷ÖĞÌáÈ¡ÓĞĞ§Êı¾İ£¬·ÅÈëUsart3_recev_buff[],±ãÓÚºóĞøÊı¾İ½âÎö
* Input          : None
* Output         : None
* Return         : ½ÓÊÕÊı¾İ³¤¶È
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
//	 printf("\r\nDMA UART2 Recev Data Start Num:%d---End Num: %d\r\n",DMA_USART3_RecevIndicator.CurrentDataStartNum,DMA_USART3_RecevIndicator.CurrentDataEndNum);    //²âÊÔÊ¹ÓÃ
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
//		 DMA_Cmd(DMA1_Channel6, DISABLE);           //¹Ø±ÕDMA·ÀÖ¹´¦ÀíÆÚ¼äÓĞÊı¾İ
//		 USART_ITConfig(USART2, USART_IT_IDLE, DISABLE);   //ÏòUSART2·¢ËÍÊı¾İÇ°£¬ÏÈ´ò¿ªUSART2½ÓÊÕ¿ÕÏĞÖĞ¶Ï£¬±ãÓÚ¼à²âÊı¾İ½ÓÊÕÍê³É
		 DataLength = DMA_UART3_RecevDataGet();
//		 DMA_UART3_RECEV_FLAG =0;
//		 DMA_Cmd(DMA1_Channel6, ENABLE);            //¿ªÆôDMA 
		 if(DataLength>0)
		 {
				printf("\r\nDataLength:%d\r\n", DataLength);             //²âÊÔÊ¹ÓÃ
			  printf("\r\nUart3:%s\r\n", Usart3_recev_buff);           //²âÊÔÊ¹ÓÃ
			  Usart3_recev_count =DataLength;                          //Êı¾İ³¤¶È¸³Öµ£¬ÏÂÒ»²½Êı¾İ½âÎö»áÓÃµ½£¬È±ÉÙÔòÎŞ·¨½âÎö£¬¿¼ÂÇÈí¼şÓÅ»¯¸ÄÎª²ÎÊı´«µİ¶ø·ÇÈ«¾Ö±äÁ¿ĞÎÊ½
			  for(i=0;i<DataLength;i++)
			  {
             printf(" %.2x ",Usart3_recev_buff[i]);               //²âÊÔÊ¹ÓÃ
        }
        StateFlag =Receive_Data_Analysis(pDeviceID, sNodeAddress);	      //·şÎñÆ÷ÏÂ·¢Êı¾İ½âÎö    //
        //¶Ô½ÓÊÕÊı¾İÀàĞÍ½øĞĞÖ¸Ê¾ 		
		 }
		 else
		 {
        printf("\r\nNo data\r\n");
     }
		 DMA_Cmd(DMA1_Channel6, DISABLE);           //¹Ø±ÕDMA·ÀÖ¹´¦ÀíÆÚ¼äÓĞÊı¾İ
		 memset(DMA_USART3_RecevBuff,0x00,RECEIVEBUFF_SIZE);   //¸´Î»DMAÊı¾İ½ÓÊÕBUFF
     DMA_UART3_RECEV_FLAG =0;
		 DMA_Cmd(DMA1_Channel6, ENABLE);            //¿ªÆôDMA 
		 USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);   //ÏòUSART2·¢ËÍÊı¾İÇ°£¬ÏÈ´ò¿ªUSART2½ÓÊÕ¿ÕÏĞÖĞ¶Ï£¬±ãÓÚ¼à²âÊı¾İ½ÓÊÕÍê³É
  } 
	return StateFlag;
}


/*******************************************************************************
¹¦ÄÜ£ºÈ¥³ı×î´ó×îĞ¡Öµ£¬¶ÔÊ£ÓàÊı¾İÇóÆ½¾ù
ÊäÈë£º¸¡µãÊıÊı×é£¬´¦ÀíÊı¾İÊıÁ¿
·µ»Ø£º´¦ÀíºóµÄÆ½¾ùÖµ
±àĞ´£º
±àĞ´ÈÕÆÚ£ºXXÄêXXÔÂXXÈÕ
°æ±¾£ºv0.1
********************************************************************************/
//float GetAverageData(float* pSensorData, u8 SensorData_Count,float* pDiffer)
//{
//	float DataTemp[FILTER_ORDER] ={0.0};
//  float Temp = 0.0;
//  float AverageData = 0.0;
//  float Dvalue = 0.0;          //×î´óÖµÓë×îĞ¡ÖµÖ®²î
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
////  	printf("\r\n**%f**\r\n##", DataTemp[i]);    //²âÊÔÊ¹ÓÃ
//  }
//	
//  for(i=SensorData_Count-1;i>=1;i--)
//	{ 
//		for(j=SensorData_Count-1;j>=SensorData_Count-i;j--)    //´ÓĞ¡µ½´óË³ĞòÅÅĞò
//		{
//       if(DataTemp[j] < DataTemp[j-1])
//			 {
//          Temp = DataTemp[j-1];
//				  DataTemp[j-1] = DataTemp[j];
//				  DataTemp[j] =  Temp;
//       }
//    }
//  }

//	Temp =0;                            //¸´Î»ÀÛ¼ÓÆ÷
//	Dvalue =DataTemp[SensorData_Count-1]-DataTemp[0];
//	*pDiffer =Dvalue ;                   //·µ»Ø×î´ó×îĞ¡²îÖµ
//	
//	if((Dvalue > 0.1) &&(SensorData_Count>=4))           //µ±×î´óÖµÓë×îĞ¡ÖµÖ®²î³¬¹ı10cmÊ±£¬È¥³ı2¸öÆ«²î½Ï´óÖµ
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
//   u8     TemperatureReadCmd[8]={0x01,0x03,0x00,0x00,0x00,0x01,0x84,0x0A};           //¶ÁÎÂ¶ÈÊı¾İÖ¸Áî,×îºóÁ½Î»ÎªCRCĞ£ÑéÂë
//   u8     TemperatureReadResp[3]={0x01,0x03,0x02};   //Êı¾İÓ¦´ğÖ¸Ê¾:01,03,02,T_H,T_L,CrcL,CrcH
//   u8*    pUart4Send =  TemperatureReadCmd;
//   u8*    pDataRead  =NULL; 
//   short  TempShort=0;           //ÓÃÓÚ¸ºÖµ×ª»»
//   float  Temper =0;             //ÎÂ¶ÈÊı¾İÔİ´æ
//   float  DataTempArry[FILTER_ORDER] ={0};

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //½ÓÊÕ±êÖ¾±äÁ¿¸´Î»
//     Uart4_rev_count   =0;   //½ÓÊÕ¼ÆÊıÆ÷ÇåÁã
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //Çå¿ÕÊı¾İ½ÓÊÕBUFF
//		 
//     DIR485_Send();                           //485·¢ËÍÊ¹ÄÜ
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, TemperatureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //µÈ´ı·¢ËÍÍê±Ï 
//		 }
//		 Delay_us(450);                  //ÔÚÊı¾İ½ÓÊÕÍê³Éºó£¬ÑÓ³ÙÒ»¶ÎÊ±¼äÔÙÇĞ»»ÊÕ·¢¿ØÖÆ¿ª¹Ø£¬Ä¬ÈÏÑÓÊ±350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //²âÊÔÊ¹ÓÃ
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //¼ì²éÊÇ·ñÊÕµ½ÎÂ¶È´«¸ĞÆ÷Ó¦´ğ
//				 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
//				 {
//           TempShort =pDataRead[3]*256 +pDataRead[4];
//					 DataTempArry[j]  =TempShort;
//					 DataTempArry[j] = DataTempArry[j]/10;		 
//					 
////					 DataTempArry[j] =(pDataRead[3]*256 +pDataRead[4])/10;
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Temperature is :%.1f ¡æ\r\n",DataTempArry[j]);                 //²âÊÔÊ¹ÓÃ
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
//  Temper = GetAverageData(DataTempArry, FILTER_ORDER);  //ÂË³ı×î´ó×îĞ¡Öµ£¬È¡µÃÆ½¾
//	printf("\r\nTemperature\r\n");                 //²âÊÔÊ¹ÓÃù
//  for(i=0;i<FILTER_ORDER;i++)
//	{
//     printf("--%.1f--",DataTempArry[i]);                 //²âÊÔÊ¹ÓÃ
//  }
//	return  Temper;		
//}
*/
/*******************************************************************************
* Function Name  : XX
* Description    : ÊÊÓÃÓÚÀ¥ÂØÖĞ´óIP68ÎÂ¶È´«¸ĞÆ÷Êı¾İ²É¼¯£¬Õë¶Ô¶şÔºÇ°ÆÚ°²×°Éè±¸£¬Êı¾İ²É¼¯Ö¸ÁîÂÔÓĞ²îÒì
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//float Temperature_DataGet(u8*  SenserStatus)
//{
//   u8     i=0,j=0; 	
//   u8     TemperatureReadCmd[8]={0xC0,0x03,0x00,0x01,0x00,0x01,0xC5,0x1B};           //¶ÁÎÂ¶ÈÊı¾İÖ¸Áî,×îºóÁ½Î»ÎªCRCĞ£ÑéÂë
//   u8     TemperatureReadResp[3]={0xC0,0x03,0x02};   //Êı¾İÓ¦´ğÖ¸Ê¾:01,03,02,T_H,T_L,CrcL,CrcH
//   u8*    pUart4Send =  TemperatureReadCmd;
//   u8*    pDataRead  =NULL; 
//	 u8     RespCount  =0;
//   float  Temper =0;   //ÎÂ¶ÈÊı¾İÔİ´æ
//   float  DataTempArry[FILTER_ORDER] ={0};
//   short  TempShort =0;
//   float  MaxDifference =0;  //×î´óÖµÓë×îĞ¡Öµ²îÖµ

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //½ÓÊÕ±êÖ¾±äÁ¿¸´Î»
//     Uart4_rev_count   =0;   //½ÓÊÕ¼ÆÊıÆ÷ÇåÁã
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //Çå¿ÕÊı¾İ½ÓÊÕBUFF
//		 
//     DIR485_Send();                           //485·¢ËÍÊ¹ÄÜ
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, TemperatureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //µÈ´ı·¢ËÍÍê±Ï 
//		 }
//		 Delay_us(300);                  //ÔÚÊı¾İ½ÓÊÕÍê³Éºó£¬ÑÓ³ÙÒ»¶ÎÊ±¼äÔÙÇĞ»»ÊÕ·¢¿ØÖÆ¿ª¹Ø£¬Ä¬ÈÏÑÓÊ±350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //²âÊÔÊ¹ÓÃ
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)TemperatureReadResp, sizeof(Uart4_rev_buff), sizeof(TemperatureReadResp));  //¼ì²éÊÇ·ñÊÕµ½ÎÂ¶È´«¸ĞÆ÷Ó¦´ğ
//				 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-8)))
//				 {
//           TempShort =(pDataRead[3]*256 +pDataRead[4]);
//					 Temper =TempShort;
//					 DataTempArry[j] =Temper/10; 		 
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Temperature is :%f ¡æ\r\n", DataTempArry[j]);                 //²âÊÔÊ¹ÓÃ
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
//     *SenserStatus =0;    //ÔÊĞí´æÔÚ¶ªÊı¾İÇé¿ö
//  }
//  Temper = GetAverageData(DataTempArry, RespCount, &MaxDifference);  //ÂË³ı×î´ó×îĞ¡Öµ£¬È¡µÃÆ½¾ù
//	Temper = Temper*10;        //¾«¶È¿ØÖÆÔÚ1Î»Ğ¡Êı£¬µ¥Î»¡æ
//	Temper =(int)Temper;       //¾«¶È¿ØÖÆÔÚ1Î»Ğ¡Êı£¬µ¥Î»¡æ
//	Temper =Temper/10;         //¾«¶È¿ØÖÆÔÚ1Î»Ğ¡Êı£¬µ¥Î»¡æ
//	
//	if((Temper<-20)||(Temper>200))                 //ÎÂ¶È´«¸ĞÆ÷Êı¾İ³¬Á¿³Ì±¨Òì³£
//	{
//     *SenserStatus =2;
//  }
//	printf("\r\nTemperature\r\n");                 //²âÊÔÊ¹ÓÃ
//  for(i=0;i<FILTER_ORDER;i++)
//	{
//     printf("--%.1f--",DataTempArry[i]);         //²âÊÔÊ¹ÓÃ
//  }
//	return  Temper;		
//}

/*******************************************************************************
* Function Name  : XX
* Description    : ÊÊÓÃÓÚÀ¥ÂØÖĞ´óIP68Ñ¹Á¦´«¸ĞÆ÷Êı¾İ²É¼¯
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//float Pressure_DataGet(u8*  SenserStatus)
//{
//   u8     i=0,j=0; 	
//   u8     PressureReadCmd[8]={0x01,0x04,0x00,0x02,0x00,0x02,0xD0,0x0B};           //¶ÁÑ¹Á¦Êı¾İÖ¸Áî,×îºóÁ½Î»ÎªCRCĞ£ÑéÂë
//   u8     PressureReadResp[3]={0x01,0x04,0x04};   //Êı¾İÓ¦´ğÖ¸Ê¾:01,04,04,P_1,P_2,P_3,P_4,CrcL,CrcH    //¸¡µã±íÊ¾£¬¸ßÎ»×Ö½ÚÔÚÇ°£¬µÍÎ»×Ö½ÚÔÚºó
//   u8*    pUart4Send = PressureReadCmd;
//   u8*    pDataRead  =NULL; 
//   u8     RespCount =0;
////   short  TempShort=0;           //ÓÃÓÚ¸ºÖµ×ª»»
//   float  Pressure =0;   //Ñ¹Á¦Êı¾İÔİ´æ
//   float  DataTempArry[FILTER_ORDER] ={0};
//   union  Hfloat  PressureTemp;
//   float  MaxDifference =0;  //×î´óÖµÓë×îĞ¡Öµ²îÖµ

//   for(j=0;j<FILTER_ORDER;j++)
//	 {
//		 Uart4_rev_comflag =0;   //½ÓÊÕ±êÖ¾±äÁ¿¸´Î»
//     Uart4_rev_count   =0;   //½ÓÊÕ¼ÆÊıÆ÷ÇåÁã
//	   memset(Uart4_rev_buff,0x00,sizeof(Uart4_rev_buff));  //Çå¿ÕÊı¾İ½ÓÊÕBUFF
//		 
//     DIR485_Send();                           //485·¢ËÍÊ¹ÄÜ
//		 Delay_ms(20);                            
//	   for(i=0;i<8;i++)
//		 {  
//			 USART_SendData(UART4, PressureReadCmd[i]);
//			 pUart4Send++;
//			 while (USART_GetFlagStatus(UART4, USART_FLAG_TC) == RESET){;}   //µÈ´ı·¢ËÍÍê±Ï 
//		 }
//		 Delay_us(350);                  //ÔÚÊı¾İ½ÓÊÕÍê³Éºó£¬ÑÓ³ÙÒ»¶ÎÊ±¼äÔÙÇĞ»»ÊÕ·¢¿ØÖÆ¿ª¹Ø£¬Ä¬ÈÏÑÓÊ±350us		
//		 DIR485_Receive();
//		 Delay_ms(500);     //²âÊÔÊ¹ÓÃ
//		 for(i=0;i<10;i++) 
//		 {
//       if(Uart4_rev_comflag ==1)
//			 {
//				 pDataRead = (u8*)Find_SpecialString((char*)Uart4_rev_buff, (char*)PressureReadResp, sizeof(Uart4_rev_buff), sizeof(PressureReadResp));  //¼ì²éÊÇ·ñÊÕµ½ÎÂ¶È´«¸ĞÆ÷Ó¦´ğ
//			 	 if((pDataRead!=NULL)&&(pDataRead< (Uart4_rev_buff +sizeof(Uart4_rev_buff)-10)))
//				 {
//           for(i=0;i<4;i++)
//					 {			
//              PressureTemp.Data_Hex[3-i] =pDataRead[3+i];                     //¸ßÎ»×Ö½ÚÔÚÇ°
////						  #if DEBUG_TEST
////							printf("\r\n--%x-- \r\n", pDataRead[3+i]);  //²âÊÔÊ¹ÓÃ
////							#endif
//           }
//					 DataTempArry[j] =PressureTemp.Data_Float;
//        	 #if DEBUG_TEST
//				   printf("\r\nPercept Pressure is :%.3f kPa\r\n", DataTempArry[j]);  //²âÊÔÊ¹ÓÃ
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
//      *SenserStatus =0;   //ÔÊĞí´æÔÚ¶ªÊı¾İÇé¿ö
//   }
//   Pressure = GetAverageData(DataTempArry, RespCount, &MaxDifference);  //ÂË³ı×î´ó×îĞ¡Öµ£¬È¡µÃÆ½¾ù
//	 Pressure =(int)Pressure;                                //¾«¶È¿ØÖÆÔÚ3Î»Ğ¡Êı£¬µ¥Î»kPa
//	 Pressure =Pressure/1000;                                //¾«¶È¿ØÖÆÔÚ3Î»Ğ¡Êı£¬µ¥Î»MPa
//   printf("\r\nSoothData:%f MPa\r\n",Pressure);            //²âÊÔÊ¹ÓÃ
//	 if((Pressure<-0.1)||(Pressure>1.6))                     //Ñ¹Á¦Êı¾İ³¬Á¿³Ì±¨Òì³£
//	 {
//     *SenserStatus =2;
//   }
//	 if(Pressure<0)
//	 {
//      Pressure=0;                                      //¸ºÖµĞŞÕı£¬È¥³ıÁãÆ¯Ôì³ÉµÄÓ°Ïì
//   }
//	 printf("\r\nPressure\r\n");                           //²âÊÔÊ¹ÓÃ
//   for(i=0;i<FILTER_ORDER;i++)
//	 {
//     printf("--%.3f--",DataTempArry[i]);                 //²âÊÔÊ¹ÓÃ
//   }
//	 return  Pressure;		
//}		
		
/*******************************************************************************
* Function Name  : void SenserDataCollect1(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
* Description    : ÊÊÓÃÓÚÀÏ°å×Ó
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/*
//void SenserDataCollect1(struct SenserData* pGetData, u8* pDevID, u16 NodeAddr)
//{
//			u32       TimCount_Current =0;
//			u8        i=0;   //²âÊÔÊ¹ÓÃ

//			#if DEBUG_TEST
//			printf("DataCollectCount:%d\r\n",DataCollectCount);  //²âÊÔÊ¹ÓÃ
//			#endif
//			
////			if(DataCollectCount == DeviceConfig.CollectNum)    //µÚÒ»´Î²É¼¯Êı¾İ£¬¼ÇÂ¼²É¼¯Ê±¼äĞÅÏ¢£¬ºóĞø¿¼ÂÇÃ¿´ÎÊÇ·ñ²ÉÓÃÕæÊµÊ±¼ä
//			{
//				pGetData->DataCount =0;                            //¶ÔÒºÎ»Êı¾İ»º´æ¼ÆÊıÆ÷¸³³õÖµ                       
//				TimCount_Current = RTC_GetCounter();
//				Time_Display(TimCount_Current,&systmtime); 
//						
//			  DeviceConfig.Time_Sec  =systmtime.tm_sec;
//				DeviceConfig.Time_Min  =systmtime.tm_min;
//				DeviceConfig.Time_Hour =systmtime.tm_hour;
//			  DeviceConfig.Time_Mday =systmtime.tm_mday;		
//			  DeviceConfig.Time_Mon  =systmtime.tm_mon;
//			  DeviceConfig.Time_Year =systmtime.tm_year-2000; //¶ÔÉÏ´«Äê·İÈ¥»ùÊıĞŞÕı				
//    		#if DEBUG_TEST
//				printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //²âÊÔÊ¹ÓÃ
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //²âÊÔÊ¹ÓÃ
//    		#endif
//			}		
//			i = pGetData->DataCount;
//    	pGetData->TemperatureData[i] = Temperature_DataGet();
//			Delay_ms(800);
//			pGetData->PressureData[i]    = Pressure_DataGet();
//			if(i==0)
//			{
//        pGetData->CollectTime[i] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);       //µÚÒ»×éÊı¾İ²É¼¯Ê±¼ä
//      }
//		  else
//			{
//        pGetData->CollectTime[i] =(pGetData->CollectTime[i-1])+ DeviceConfig.CollectPeriod;    //ºóĞøÃ¿×éÊı¾İ²É¼¯Ê±¼äÎªÔÚÇ°Ò»×éÊı¾İ»ù´¡ÉÏÔö¼Ó²É¼¯¼ä¸ô
//      }
//			pGetData->DataCount =(pGetData->DataCount)+1;
//     	DataCollectCount--;	

//}
*/

/*******************************************************************************
* Function Name  : XX
* Description    : ÊÊÓÃÓÚĞÂ°å×Ó
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void Sensor_DataCollect( struct SenserData* pGetData, struct DeviceStatus* pMonitorStatus)
//{
//   	 u32       TimCount_Current =0;
//		 u8        i=0;                     //²âÊÔÊ¹ÓÃ
////      char      PressureSensorZeroAdjust[8]={0x01,0x06,0x00,0x05,0x00,0x00,0x99,0xCB};
////      char      PressureSensorAccessCode[8]={0x01,0x06,0x00,0x0A,0x38,0x79,0x7B,0xEA};
//	   char*     pRecevBuff  =NULL; 
//		 u8        PressStatus =3;        //Ñ¹Á¦´«¸ĞÆ÷×´Ì¬±êÖ¾±äÁ¿£¬ÓÃÓÚ¼ì²â´«¸ĞÆ÷ÊÇ·ñ¹¤×÷Õı³££¬0±íÊ¾¹¤×÷Õı³££¬1±íÊ¾Ê§Ğ§,3±íÊ¾Î´ÆôÓÃ
//     u8        TemperatueStatus =3;   //ÎÂ¶È´«¸ĞÆ÷×´Ì¬±êÖ¾±äÁ¿£¬ÓÃÓÚ¼ì²â´«¸ĞÆ÷ÊÇ·ñ¹¤×÷Õı³££¬0±íÊ¾¹¤×÷Õı³££¬1±íÊ¾Ê§Ğ§,3±íÊ¾Î´ÆôÓÃ
//           
//		 TimCount_Current = RTC_GetCounter();
//		 Time_Display(TimCount_Current,&systmtime); 				
//		 DeviceConfig.Time_Sec  =systmtime.tm_sec;
//		 DeviceConfig.Time_Min  =systmtime.tm_min;
//		 DeviceConfig.Time_Hour =systmtime.tm_hour;
//		 DeviceConfig.Time_Mday =systmtime.tm_mday;	
//		 DeviceConfig.Time_Mon  =systmtime.tm_mon;                                
//		 DeviceConfig.Time_Year =systmtime.tm_year-2000; //¶ÔÉÏ´«Äê·İÈ¥»ùÊıĞŞÕı
// 		                                     		
//     #if DEBUG_TEST
//		 printf("Time: %0.4d-%0.2d-%0.2d %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,     //²âÊÔÊ¹ÓÃ
//											          systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);     //²âÊÔÊ¹ÓÃ
//     #endif
//			
//		 //ÏÈ¿ªÆô¾²Ñ¹´«¸ĞÆ÷£¬²É¼¯¾²Ñ¹Ê½´«¸ĞÆ÷Êı¾İ£¬
//     PowerON_485();             //¿ªÆô485Ä£¿éµçÔ´
//		 Delay_ms(500);  	
//		 PowerON_12V();	            //¿ªÆô12V×ÜµçÔ´
//		 #if DEBUG_TEST
//		 printf("\n\r***----12V DC-DC Power ON----***\r\n");   //²âÊÔÊ¹ÓÃ
//		 #endif
//		 Delay_ms(1000);     
//		 PowerON_Pressure();        //¿ªÆô¾²Ñ¹´«¸ĞÆ÷µçÔ´
//		 #if DEBUG_TEST
//		 printf("\n\r***----Pressure Sensor Power ON----***\r\n");   //²âÊÔÊ¹ÓÃ
//		 #endif
//		 for(i=0;i<80;i++)          //Ôö¼ÓÑÓÊ±£¬Ê¹¾²Ñ¹Ì½Í·ÎÈ¶¨£¬Í¬Ê±¼à²â´®¿Ú¿ØÖÆÃüÁîÊäÈë
//		 {
//				Delay_ms(100);   

//				pRecevBuff = Find_SpecialString(Usart1_recev_buff, "BATT Recover" ,sizeof(Usart1_recev_buff), 12);  //¼ì²éÊÇ·ñÊÕµ½¾²Ñ¹´«¸ĞÆ÷ÁãµãÖØÖÃÃüÁî£¬ÓÃÓÚ´«¸ĞÆ÷ÁãµãĞ£×¼
//				if(pRecevBuff !=NULL)
//				{
//						DataRead_From_Flash(0,9,0, &i,1);                //´ÓFlashÖĞ¶Á³öµç³ØÈİÁ¿ÏµÊı±¸·İÊı¾İ
//						if(i<=113)
//						{
//							 DataWrite_To_Flash(0,1,0,  &i,1);             //½«±¸·İÇøÊı¾İ¸üĞÂµ½µç³ØÈİÁ¿ÏµÊıÊı¾İ´æ´¢Çø
//							 #if DEBUG_TEST
//							 printf("\r\n------------BATT Recover Sucess!------------\r\n");                 //²âÊÔÊ¹ÓÃ
//						   #endif
//						}
//						#if DEBUG_TEST
//						printf("\r\n------------Warning:BATT Recover Data too big!------------\r\n");      //²âÊÔÊ¹ÓÃ
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
//		 PowerOFF_Pressure();                     //¹Ø±Õ¾²Ñ¹´«¸ĞÆ÷µçÔ´
//		 #if DEBUG_TEST
//		 printf("\n\r***----Pressure Sensor Power OFF----***\r\n");   //²âÊÔÊ¹ÓÃ
//		 #endif
//		 Delay_ms(500);
//		 PowerON_TemperatureSensor();             //´ò¿ªÎÂ¶È´«¸ĞÆ÷µçÔ´
//		 #if DEBUG_TEST
//		 printf("\n\r***----Temperature Sensor Power ON----***\r\n");   //²âÊÔÊ¹ÓÃ
//		 #endif
//		 Delay_ms(3000);
//		 pGetData->TemperatureData[0] = Temperature_DataGet(&TemperatueStatus);
//		 pMonitorStatus->TemperatureSensorStatus =TemperatueStatus;
//		 Delay_ms(100);
//		 PowerOFF_TemperatureSensor();            //¹Ø±ÕÎÂ¶È´«¸ĞÆ÷µçÔ´
//		 #if DEBUG_TEST
//		 printf("\n\r***----Temperature Sensor Power OFF----***\r\n");   //²âÊÔÊ¹ÓÃ
//		 #endif
//		 PowerOFF_12V();
//		 #if DEBUG_TEST
//		 printf("\n\r***----12V DC-DC Power OFF----***\r\n");   //²âÊÔÊ¹ÓÃ
//		 #endif
//		 Delay_ms(100);
//		 PowerOFF_485();                          //µ±ÒºÎ»Êı¾İ²É¼¯Íê³ÉÊ±£¬¹Ø±ÕUart×ª485Ä£¿éµçÔ´
//		 pGetData->CollectTime[0] =(DeviceConfig.Time_Hour)*60 +(DeviceConfig.Time_Min);       //Êı¾İ²É¼¯Ê±¼ä

//	
//}


/*******************************************************************************
* Function Name  :Power_SX1278_Init()
* Description    : ³õÊ¼»¯433Ä£¿é
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
* Description    : ³õÊ¼»¯433Ä£¿é
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
	////////////////////ÅäÖÃ433Ä£¿é²ÎÊı //////////////////// 
//	SetNodeSerialPort(4, 0);                           //ÉèÖÃ433Ä£¿é´®¿Ú²¨ÌØÂÊ£º9600,Ğ£ÑéÀàĞÍ:ÎŞĞ£Ñé
	SetNodeCentralFrequency(439);                      //ÉèÖÃ433Ä£¿éÔØ²¨ÖĞĞÄÆµÂÊ£º434MHz
	SetNodeFrequencyExpandFactor(11);                  //ÉèÖÃ433Ä£¿éÀ©ÆµÒò×Ó£º2048
	SetNodeFrequencyExpandBandwidth(7);                //ÉèÖÃ433Ä£¿éÀ©Æµ´ø¿í£º125K
	SetNodeWorkMode(2);                                //ÉèÖÃ433Ä£¿é¹¤×÷Ä£Ê½£º½ÚµãÄ£Ê½
	SetNodeID (sNodeID);                               //ÉèÖÃ433Ä£¿éID£ºÊäÈë½ÚµãID
	SetNodeNetworkID (66);                              //ÉèÖÃ433Ä£¿éÍøÂçID£º19
	SetNodeSendPowerGrade (7);                         //ÉèÖÃ433Ä£¿é·¢Éä¹¦ÂÊµÈ¼¶£º20dBm
	SetNodeBreathPeriod (0);                           //ÉèÖÃ433Ä£¿éºôÎüÖÜÆÚ:2s
	SetNodeBreathTime (5);                             //ÉèÖÃ433Ä£¿éºôÎüÊ±¼ä£º64ms
	
	PowerOFF_433();                                   //ÖØÆô433Ä£¿é£¬Ê¹ÅäÖÃÉúĞ§        
	Delay_ms(5000);
	PowerON_433(); 
	////////////////////¶ÁÈ¡433Ä£¿é²ÎÊı //////////////////// 
//u8  GetNodeReceiveSignalEnergy ();                 //»ñÈ¡433Ä£¿éÉÏÒ»Ö¡Êı¾İ½ÓÊÕĞÅºÅÇ¿¶È
  UartConfig =GetNodeSerialPortConfig();              //»ñÈ¡433Ä£¿é´®¿ÚÅäÖÃ²ÎÊı
  BaudRate   =UartConfig>>8;
  VerifyType =UartConfig &0xFF;
  printf("\r\nBaud Rate:%d---Verify Type:%d\r\n",BaudRate,VerifyType);//²âÊÔÊ¹ÓÃ
  CentralFreq =GetNodeCentralFrequency ();               //»ñÈ¡433Ä£¿éÔØ²¨ÆµÂÊ²ÎÊı
  printf("\r\nCentral Frequency :%d MHz\r\n",(CentralFreq+1));   //²âÊÔÊ¹ÓÃ
  ExpFactor   =GetNodeFrequencyExpandFactor();           //»ñÈ¡433Ä£¿éÀ©ÆµÒò×Ó²ÎÊı
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
  printf("\r\nNode Frequency Expand Factor:%d \r\n",Temp);   //²âÊÔÊ¹ÓÃ
  ExpBW =GetNodeFrequencyExpandBandwidth ();                 //»ñÈ¡433Ä£¿éÀ©Æµ´ø¿í²ÎÊı
	switch( ExpBW )
  {
			case 6:  
			{
				Temp=63;       //62.5Ô¼µÈÓÚ63
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
	printf("\r\nNode Frequency Expand Bandwidth:%dKHz\r\n",Temp);   //²âÊÔÊ¹ÓÃ
  WorkMode = GetNodeWorkMode ();                                  //»ñÈ¡433Ä£¿é¹¤×÷Ä£Ê½²ÎÊı
	switch( WorkMode )
  {
			case 0:  
			{
	      printf("\r\n433 Module Work Mode is: Standard\r\n");   //²âÊÔÊ¹ÓÃ
				break;
			}
			case 1:  
			{
				printf("\r\n433 Module Work Mode is: Center\r\n");    //²âÊÔÊ¹ÓÃ
				break;
			}
			case 2:  
			{
				printf("\r\n433 Module Work Mode is: Node\r\n");    //²âÊÔÊ¹ÓÃ
				break;
			}		
			default:
			{
				printf("\r\n433 Module Work Mode is: Unknown\r\n");    //²âÊÔÊ¹ÓÃ
				break;
			}		
  }
  NodeID =GetNodeID ();                                 //»ñÈ¡433Ä£¿é½ÚµãID
	printf("\r\n433 Module Node ID is: %x\r\n",NodeID);   //²âÊÔÊ¹ÓÃ
  NetID =GetNetworkID ();                               //»ñÈ¡433Ä£¿éÍøÂçID
	printf("\r\n433 Module Network ID is: %x\r\n",NetID); //²âÊÔÊ¹ÓÃ
  PowerGrade = GetNodeSendPowerGrade ();                //»ñÈ¡433Ä£¿é·¢Éä¹¦ÂÊ
	printf("\r\n433 Module Send Power Grade is: %d\r\n",PowerGrade); //²âÊÔÊ¹ÓÃ
  BreathPeriod = GetNodeBreathPeriod ();                //»ñÈ¡433Ä£¿éºôÎüÖÜÆÚ
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
	printf("\r\nNode Breath Period:%d s\r\n",Temp );   //²âÊÔÊ¹ÓÃ
  WakeTime  =  GetNodeBreathTime ();                 //»ñÈ¡433Ä£¿éºôÎüÊ±¼ä
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
	printf("\r\nNode Wake Time:%d ms\r\n",Temp );   //²âÊÔÊ¹ÓÃ

}
/*******************************************************************************
* Function Name  : XXX
* Description    : Éè±¸½øÈëĞİÃß×´Ì¬
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void  DeviceSleep(u8 DeviceSleepDelay)
{
  u16 WakeupTime  =0;  //ÏÂÒ»´Î»½ĞÑÊ±¼ä
	u32 SleepTime   =0;  //±¾´ÎÉè±¸ĞİÃßÊ±¼ä	
	u32 CurrentTime =0;  //RTCµ±Ç°Ê±¼ä
	u32 RelativeCurrentTime =0;  //µ±Ç°Ê±¼ä(Ïà¶ÔÖµ£¬¿ÉÄÜ³¬¹ı1440)
	u8  i=0;
	
  CurrentTime = RTC_GetCounter();                   //»ñÈ¡ÏµÍ³µ±Ç°Ê±¼ä
  Time_Display(CurrentTime,&systmtime);    

  CurrentTime = RTC_GetCounter();
	Time_Display(CurrentTime,&systmtime); 
	CurrentTime =	systmtime.tm_hour*60+systmtime.tm_min;
	if((DeviceConfig.CollectStartTime + DeviceConfig.CollectPeriod* (DeviceConfig.CollectNum-1))<1440)            
	{ 
		  if(CurrentTime <DeviceConfig.CollectStartTime)
			{
        WakeupTime =DeviceConfig.CollectStartTime ;     //ÏÂÒ»´ÎÉè±¸»½ĞÑÊ±¿Ì,ÒÔ·ÖÖÓÎªµ¥Î»£¬Ã¿Ìì24Ğ¡Ê±£¬0~1440 
				SleepTime  =(WakeupTime-CurrentTime)*60;        //µ¥Î»Ãë
      }
			else if(CurrentTime >(DeviceConfig.CollectStartTime+ (DeviceConfig.CollectNum-1)*DeviceConfig.CollectPeriod))
			{
        WakeupTime =DeviceConfig.CollectStartTime ;//ÏÂÒ»´ÎÉè±¸»½ĞÑÊ±¿Ì,ÒÔ·ÖÖÓÎªµ¥Î»£¬Ã¿Ìì24Ğ¡Ê±£¬0~1440
				SleepTime  =(WakeupTime +1440 -CurrentTime)*60;  //µ¥Î»Ãë   
      }
			else
			{
        for(i=0;i<(DeviceConfig.CollectNum-1);i++)
				{
	        if((CurrentTime>=(DeviceConfig.CollectStartTime+ i*DeviceConfig.CollectPeriod))&&(CurrentTime<=(DeviceConfig.CollectStartTime+ (i+1)*DeviceConfig.CollectPeriod)))  
			    {  
				    WakeupTime =DeviceConfig.CollectStartTime +(i+1)*DeviceConfig.CollectPeriod; //ÏÂÒ»´ÎÉè±¸»½ĞÑÊ±¿Ì,ÒÔ·ÖÖÓÎªµ¥Î»£¬Ã¿Ìì24Ğ¡Ê±£¬0~1440
				    SleepTime  =(WakeupTime-CurrentTime)*60;        //µ¥Î»Ãë
						break;
          }
        }
      }
//		  printf("\r\n---1--DeviceConfig.CollectNum:%d-----CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,CurrentTime,WakeupTime);                 //²âÊÔÊ¹ÓÃ
			printf("\r\nDeviceConfig.CollectNum:%d--%d--%d------CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,DeviceConfig.CollectPeriod,DeviceConfig.CollectStartTime,CurrentTime,WakeupTime);                 //²âÊÔÊ¹ÓÃ
				
			DeviceSleepDelay =DeviceSleepDelay %DEVICE_DELAY_MAX;  //·ÀÖ¹³öÏÖÒç³ö
			SleepTime =SleepTime+ DeviceSleepDelay*20;             //¸ù¾İÉè±¸±àºÅ²»Í¬Ôö¼ÓÏàÓ¦ÑÓÊ±£¬±ÜÃâÍ¨¹ı433ÍøÂçÉÏ´«Êı¾İÊ±·¢ÉúÅö×²£¬µ¥Î»Ãë
		  gotoSleep(SleepTime);
	}
	else
	{   
			if((CurrentTime <DeviceConfig.CollectStartTime)&&(CurrentTime >((DeviceConfig.CollectStartTime + DeviceConfig.CollectPeriod* (DeviceConfig.CollectNum-1))%1440)))  
			{
        WakeupTime =DeviceConfig.CollectStartTime ;     //ÏÂÒ»´ÎÉè±¸»½ĞÑÊ±¿Ì,ÒÔ·ÖÖÓÎªµ¥Î»£¬Ã¿Ìì24Ğ¡Ê±£¬0~1440 
				SleepTime  =(WakeupTime-CurrentTime)*60;        //µ¥Î»Ãë
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
							 WakeupTime =(DeviceConfig.CollectStartTime +(i+1)*DeviceConfig.CollectPeriod); //ÏÂÒ»´ÎÉè±¸»½ĞÑÊ±¿Ì,ÒÔ·ÖÖÓÎªµ¥Î»£¬Ã¿Ìì24Ğ¡Ê±£¬0~1440
							 SleepTime  =(WakeupTime-RelativeCurrentTime)*60;        //µ¥Î»Ãë
//							 printf("\r\n---2--\r\n");
							 break;
						}
//						printf("\r\n---0--\r\n");			
        }
//				printf("\r\n---3--\r\n");
      }
		  printf("\r\nDeviceConfig.CollectNum:%d--%d--%d------CurrentTime:%d----WakeupTime:%d\r\n",DeviceConfig.CollectNum,DeviceConfig.CollectPeriod,DeviceConfig.CollectStartTime,CurrentTime,WakeupTime);                 //²âÊÔÊ¹ÓÃ
			//¸ù¾İÉè±¸±àºÅ²»Í¬Ôö¼ÓÏàÓ¦ÑÓÊ±£¬±ÜÃâÍ¨¹ı433ÍøÂçÉÏ´«Êı¾İÊ±·¢ÉúÅö×²£¬µ¥Î»Ãë
			DeviceSleepDelay =DeviceSleepDelay %DEVICE_DELAY_MAX; //·ÀÖ¹³öÏÖÒç³ö
			SleepTime =SleepTime+ DeviceSleepDelay*20;       //ÏòºóÍÆ³Ù20s*ID
	
			
//			if(DeviceSleepDelay%2 ==0)   //Å¼Êı
//			{
//        SleepTime =SleepTime+ DeviceSleepDelay*20;       //Å¼ÊıÏòºóÍÆ³Ù20s
//      }
//			else                         //ÆæÊı
//			{
//				 if(SleepTime >DeviceSleepDelay)
//				 {
//             SleepTime =SleepTime -DeviceSleepDelay*20;  //ÆæÊıÌáÇ°20s
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
* Description    : ³õÊ¼»¯¶Ë¿Ú¼°ÍâÉè
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void PeripheralInit(u8* pDevID)
{
//	u8    SWD_STATUS   =0;
//	u8    TempArray2[2]={0};
//	u16   ShortTemp2   =0;
	
	WWDG_Init(0x7F,0X5F,WWDG_Prescaler_8); 	//Ê×ÏÈ¿ªÆô´°¿Ú¿´ÃÅ¹·£¬¼ÆÊıÆ÷ÖµÎª7f,´°¿Ú¼Ä´æÆ÷Îª5f,·ÖÆµÊıÎª8	
	USART1_Config();      /* USART1 ÅäÖÃÄ£Ê½Îª 9600 8-N-1£¬  ÖĞ¶Ï½ÓÊÕ */
	USART2_Config();      /* USART2 ÅäÖÃÄ£Ê½Îª 9600 8-N-1£¬  ÖĞ¶Ï½ÓÊÕ */
	USART3_Config();      /* USART3 ÅäÖÃÄ£Ê½Îª 9600 8-N-1£¬  ÖĞ¶Ï½ÓÊÕ */
//	USART4_Config();      /* UART4  ÅäÖÃÄ£Ê½Îª 9600 8-N-1£¬  ÖĞ¶Ï½ÓÊÕ */
  UART_NVIC_Configuration();
//	USART3_DMA_Config();
  USART2_DMA_Config();
//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);  //ÅäÖÃ´®¿ÚDMA1½ÓÊÕ
//  TIM2_Configuration();       /* ¶¨Ê±Æ÷TIM2²ÎÊıÅäÖÃ */	
//	TIM2_NVIC_Configuration();  /* ÉèÖÃ¶¨Ê±Æ÷TIM2µÄÖĞ¶ÏÓÅÏÈ¼¶ */
//	TIM3_Configuration();       /* ¶¨Ê±Æ÷TIM3²ÎÊıÅäÖÃ */	
//	TIM3_NVIC_Configuration();  /* ÉèÖÃ¶¨Ê±Æ÷TIM3µÄÖĞ¶ÏÓÅÏÈ¼¶ */
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, DISABLE);     //ÔİÊ±¹Ø±Õ¶¨Ê±Æ÷TIM3
	
	RTC_NVIC_Config();                 /* ÅäÖÃRTCÃëÖĞ¶ÏÓÅÏÈ¼¶ */
	RTC_CheckAndConfig(&systmtime);
//	Time_Show(&systmtime);             /* Display time in infinite loop */
//	WorkingTime =RTC_GetCounter();     //²É¼¯Éè±¸Æô¶¯Ê±¼ä
  SysTick_Init();
  PowerON_Flash();                     //´ò¿ªFlashµçÔ´ 
  Delay_ms(100); 
//	SPI_FLASH_Init();
//	PowerON_GPRS();                    //´ò¿ªGPRSÄ£¿éµçÔ´£¬****±ØĞë´ò¿ª£¬²éÃ÷Ô­Òò;ºóĞøµçÂ·°åº¸½ÓÕıÈ·ÒÔºó´Ë²½Öè¿ÉÒÔÊ¡ÂÔ
//	Delay_ms(100);
//	PowerON_TemperatureSensor();       //´ò¿ª³¬Éù²¨Ì½Í·µçÔ´£¬ÔÚTCPÁ¬½ÓÕıÈ·½¨Á¢ÒÔºóÔÙ¿ªÆôÌ½Í·µçÔ´£¬ºóĞøÈç¹ûĞèÒª±¸·İÀúÊ·Êı¾İ£¬ÔòĞèÒª×öÏàÓ¦ĞŞ¸Ä
//	Delay_ms(100);   
//  PowerON_485();                     //´ò¿ª485µçÔ´
//	Delay_ms(100);
//Çå³ıÒºÎ»¼Æ¸ÕÆô¶¯Ê±´®¿ÚµÄÂÒÂë,¿¼ÂÇÊÇ·ñÓĞ±ØÒª //ÁÙÊ±Ê¹ÓÃ
//	memset(Uart4_rev_buff,'\0',sizeof(Uart4_rev_buff));    //ÁÙÊ±Ê¹ÓÃ
	
 	DMA_USART3_RecevIndicator.CurrentDataStartNum =0;  //³õÊ¼»¯µ±Ç°½ÓÊÕÊı¾İ¿ªÊ¼Î»ÖÃ
	DMA_USART3_RecevIndicator.CurrentDataEndNum =0;    //³õÊ¼»¯µ±Ç°½ÓÊÕÊı¾İ½áÊøÎ»ÖÃ
	DMA_USART3_RecevIndicator.NextDataStartNum =0;     //³õÊ¼»¯ÏÂÒ»´Î½ÓÊÕÊı¾İ¿ªÊ¼Î»ÖÃ
	DMA_USART3_RecevIndicator.DMA_RecevCount =0;
	///////////////////±ØĞë½«Ä£¿éÅäÖÃ³É½ÓÊÕ×´Ì¬£¬·ñÔò´®¿Ú½ÓÊÕ²»µ½Êı¾İ
	HIGH_433_SET();              //433Ä£¿éSET¹Ü½ÅÀ­¸ß£¬ÇĞ»»µ½½ÓÊÕÄ£Ê½
	Delay_ms(100);
  LOW_433_EN();                //433Ä£¿éEN¹Ü½ÅÀ­µÍ£¬ÇĞ»»µ½¸ßËÙÄ£Ê½
	Delay_ms(100);
	
	USART_GetFlagStatus(USART1,USART_FLAG_TC);       //´®¿ÚÓ²¼ş¸´Î»Ö®ºó£¬·¢ËÍÊ××Ö½ÚÖ®Ç°£¬ÏÈ¶ÁÒ»ÏÂUSART_SR,·ÀÖ¹Êı¾İ·¢ËÍÊ±Ê××Ö½Ú±»¸²¸Ç
	USART_GetFlagStatus(USART2,USART_FLAG_TC);       //´®¿ÚÓ²¼ş¸´Î»Ö®ºó£¬·¢ËÍÊ××Ö½ÚÖ®Ç°£¬ÏÈ¶ÁÒ»ÏÂUSART_SR,·ÀÖ¹Êı¾İ·¢ËÍÊ±Ê××Ö½Ú±»¸²¸Ç
//	USART_GetFlagStatus(UART4,USART_FLAG_TC);        //´®¿ÚÓ²¼ş¸´Î»Ö®ºó£¬·¢ËÍÊ××Ö½ÚÖ®Ç°£¬ÏÈ¶ÁÒ»ÏÂUSART_SR,·ÀÖ¹Êı¾İ·¢ËÍÊ±Ê××Ö½Ú±»¸²¸Ç
	Uart_rev_finish=0;
	
	#if DEBUG_TEST
	printf("\n\r****************************************************************\r\n");   //²âÊÔÊ¹ÓÃ
	printf("\n\r*-----»¶Ó­Ê¹ÓÃBIRMM-CorrE-A±£ÎÂ²ãÏÂ¸¯Ê´»·¾³¼à²âÒÇ!-------------*\r\n");   //²âÊÔÊ¹ÓÃ
	printf("\n\r*-----Ó²¼ş°æ±¾ºÅ£ºBIRMM-CorrE V1.1---------------------------------*\r\n");   //²âÊÔÊ¹ÓÃ
  printf("\n\r*-----Èí¼ş°æ±¾ºÅ£ºBIRMM-CorrE-A_V1.1-------------------------------*\r\n");   //²âÊÔÊ¹ÓÃ
	printf("\n\r*-----Éè±¸±àºÅ£º%.2x%.2x%.2x%.2x%.2x%.2x-----------------------------------*\r\n",pDevID[0],pDevID[1],pDevID[2],pDevID[3],pDevID[4],pDevID[5]);   //²âÊÔÊ¹ÓÃ
	printf("\n\r*-----°æÈ¨ËùÓĞ£ºÖĞ¹úº½Ìì¿Æ¹¤¶şÔºÖÇ»Û¹ÜÍø¼¼ÊõÑĞ¾¿Óë·¢Õ¹ÖĞĞÄ-----*\r\n");   //²âÊÔÊ¹ÓÃ
	printf("\n\r****************************************************************\r\n");   //²âÊÔÊ¹ÓÃ
	#endif
/*	
	//////////¶Á³ö¾²Ñ¹Ì½Í·ÀÛ¼Æ¶ÁÊıÊ§°Ü´ÎÊı£¬³¬Éù²¨Ì½Í·ÀÛ¼Æ¶ÁÊıÊ§°Ü´ÎÊı£¬433Ä£¿éÊı¾İ´«ÊäÀÛ¼ÆÊ§°Ü´ÎÊı
//  DataRead_From_Flash(1,4,0, TempArray2,2);            //´ÓFlashÖĞ¶Á³ö¾²Ñ¹´«¸ĞÆ÷Êı¾İ²É¼¯Ê§°Ü´ÎÊı
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****Press senser data collect fail counter:%3d -----****\r\n",ShortTemp2);   //²âÊÔÊ¹ÓÃ
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,5,0, TempArray2,2);            //´ÓFlashÖĞ¶Á³ö³¬Éù²¨´«¸ĞÆ÷Êı¾İ²É¼¯Ê§°Ü´ÎÊı
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****Ultrasonic senser data collect fail counter:%3d ****\r\n",ShortTemp2);   //²âÊÔÊ¹ÓÃ
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,6,0, TempArray2,2);            //´ÓFlashÖĞ¶Á³ö3GÄ£¿é²¦ºÅÊ§°Ü´ÎÊı
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****433 Modle Communication fail counter:%3d -------****\r\n",ShortTemp2);   //²âÊÔÊ¹ÓÃ
//	#endif
//	
//	TempArray2[0] =0;
//	TempArray2[1] =0;
//	DataRead_From_Flash(1,7,0, TempArray2,2);            //´ÓFlashÖĞ¶Á³ö3GÄ£¿éÉÏÒ»´ÎÍ¨ĞÅĞÅºÅÇ¿¶È
//	ShortTemp2 =TempArray2[1]*256 +TempArray2[0];
//	#if DEBUG_TEST
//	printf("\n\r****433 Modle latest receive signal density:%3d dBm  ***\r\n",(short)ShortTemp2);   //²âÊÔÊ¹ÓÃ
//	printf("\n\r********************************************************\r\n");                     //²âÊÔÊ¹ÓÃ
//	#endif
//	
//	DataRead_From_Flash(1,0,0, &SWD_STATUS,1);     //´ÓFlashÖĞ¶ÁÈ¡SWD¶Ë¿Ú¸´ÓÃ×´Ì¬±êÖ¾
//	if(SWD_STATUS==1)                     
//	{
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE); 
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable,ENABLE);    //½ûÖ¹SWD½Ó¿ÚÏÂÔØ¹¦ÄÜ£¬ÓÃ×÷GPIO£¬¿ÉÍ¨¹ı³¤°´¸´Î»ÏÂÔØ³ÌĞò»òÕßÍ¨¹ı´®¿ÚÏÂ·¢Ö¸¶¨½â³ıÕ¼ÓÃ;
//		#if DEBUG_TEST
//		printf("\n\r<<<<!!!!!!!!!!!----SWD-Lock----!!!!!!!!!!!>>>>\r\n");   //²âÊÔÊ¹ÓÃ
//		#endif		
//	}
//	else
//	{
//		RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO, ENABLE);  
//		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
//		#if DEBUG_TEST
//		printf("\n\r<<<<!!!!!!!!!!!----SWD-Unlock----!!!!!!!!!!!>>>>\r\n");   //²âÊÔÊ¹ÓÃ
//		#endif
//	}
*/
	ConfigData_Init(&DeviceConfig);         //´ÓFlashÖĞ¶ÁÈ¡ÅäÖÃ²ÎÊı

}


/*******************************************************************************
* Function Name  : int main(void)
* Description    : Ö÷º¯Êı
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
	u8  HistoricalDataCount =0;//ÄÚ´æÖĞ±£´æÀúÊ·Êı¾İÊıÁ¿     //ÀúÊ·Êı¾İÉÏ±¨Ïà¹Ø
	u8  HistoricalDataSendBuff[100]={0};                    //ÀúÊ·Êı¾İÉÏ±¨Ïà¹Ø
  u16 HistoricalDataRecevFlag=0;                          //ÀúÊ·Êı¾İÉÏ±¨Ïà¹Ø
  u8  HistoricalDataSendCounter =0;                       //ÀúÊ·Êı¾İÉÏ±¨Ïà¹Ø
	u8  i=0;
	u8  RouteControl =1;      //Â·ÓÉ¿ØÖÆ±äÁ¿£¬Ä¬ÈÏÊ¹ÓÃÊı¾İ¼¯ÖĞÆ÷½øĞĞÊı¾İ´«Êä
	u8  CollectDataEnable =0; //Êı¾İ²É¼¯Ê¹ÄÜ±äÁ¿£¬1ÔÊĞíÊı¾İ²É¼¯£¬·ñÔò²»ÔÊĞí	
	u32 CurrentTime_RTC =0;   //RTCµ±Ç°Ê±¼ä
	u16 TimeDiff =0;          //Ê±¼ä²î£¨µ¥Î»£º·ÖÖÓ£©£¬ÓÃÓÚÅĞ¶ÏÊÇ·ñ·¢ÉúRTC»½ĞÑ£¬¾ßÌåÉèÖÃ²Î¿¼Éè±¸Êı¾İÓë²»Í¬Éè±¸Ö®¼äµÄÑÓÊ±£¬ÈçÉèÎª10·ÖÖÓ£¨+/-£©£¬
	                          //¼´£¬¶ÔÓÚ20ÃëÉè±¸ÑÓÊ±À´Ëµ×î¶àÖ§³Ö10*3*2 =60Ì¨Éè±¸    
	u16 TimeGet =0;
		                                      
	u8  MonitorDeviceID[6] ={0x92,0x20,0x17,0x01,0x92,0x01}; //³õÊ¼»¯Éè±¸IDºÅ
  u16 NodeAddr =0x0000;
//  struct SenserData   sPerceptionData;       //´«¸ĞÆ÷Êı¾İ
  u8  OpTypeCode =0;                         //²Ù×÷ÀàĞÍ±àÂë

	NodeAddr =MonitorDeviceID[4]*256 +MonitorDeviceID[5];     //ÌáÈ¡Éè±¸IDºÅ×îºóÃæÁ½¸ö×Ö½Ú×÷Îª½ÚµãµØÖ·
  PeripheralInit(MonitorDeviceID);                           //³õÊ¼»¯ÍâÉè
  if (PowerOffReset ==1)          
  {
    printf("\r\nµôµçÖØÆô£¬ÖØĞÂ³õÊ¼»¯¿âÂØ¼Æ\r\n");                 //²âÊÔÊ¹ÓÃ   
		Set_register_ds2780();    //µôµçºó¶Ô¿âÂØ¼ÆÖØĞÂ³õÊ¼»¯
		Delay_ms(1000); 
	  for(g=0;g<3;g++)
		{
			ambient_temperature = get_temperature();
			printf("\n\r Temperature:%d (¡æ)\r\n",ambient_temperature);
			Delay_ms(1000); 
		}
		residual_capacity=get_ACR_capacity(ambient_temperature);
		set_ACR(residual_capacity);
		printf("\n\r ACR_capacity:%d mAh\r\n",residual_capacity);
		DS2780_CapacityInit();    //µôµçºóÖØĞÂĞ´µç³ØÈİÁ¿
		DS2780_Test();            //µôµçºó½«µç³ØÈİÁ¿ÏµÊıÖØĞÂĞ´ÈëFlash
		printf("\r\n µôµçÖØÆô¹ı³Ì½áÊø\r\n");
  
	  SX1287_Init(NodeAddr);    //433Ä£¿é²ÎÊı³õÊ¼»¯                          
	  //////Éè±¸ÉÏµç×¢²á///////////
//		Delay_ms(2000);   
//		DeviceStartupRequest(Usart2_send_buff, DeviceID, NodeAddr);  
  }	
	
	for(i=0;i<20;i++)
	{
		 Delay_ms(200);                                //²Î¿¼Ì½Í·ÉÏµçºóÊı¾İÎÈ¶¨Ê±¼ä£¬×÷ÏàÓ¦µ÷Õû
		 if(DMA_UART3_RECEV_FLAG==1)                   //²éÑ¯Êı¾İ½ÓÊÕÇé¿ö
		 {
				DMA_UART3_RecevDetect(MonitorDeviceID, NodeAddr);  
				break;
		 }
  }	
	while(1)
  {
		
		WWDOG_Feed =0x1FFF;                              //´°¿Ú¿´ÃÅ¹·Î¹¹·,¶¨Ê±4·Ö20Ãë£¬µÚ¶ş×Ö½ÚÔ¼1Ãë±ä»¯Ò»´Î£¬¼´0x09AF±äµ½0x099FÔ¼ºÄÊ±1Ãë		
		CurrentTime_RTC = RTC_GetCounter();                   //»ñÈ¡ÏµÍ³µ±Ç°Ê±¼ä
		Time_Display(CurrentTime_RTC,&systmtime);  
	  
	  printf(" \r\nµ±Ç°Ê±¼äÎª: %dÄê %dÔÂ %dÈÕ   %0.2d:%0.2d:%0.2d\r\n",systmtime.tm_year,systmtime.tm_mon,    
		                             systmtime.tm_mday,systmtime.tm_hour,systmtime.tm_min,systmtime.tm_sec);            /* Êä³ö¹«ÀúÊ±¼ä */
	                   
		//µ±¶¨Ê±Ê±¼äµ½»òÕßÓĞ²éÑ¯Êı¾İÊ±²Å²É¼¯Êı¾İ²¢ÉÏ±¨Êı¾İ
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
   	CollectDataEnable =1;//²âÊÔÊ¹ÓÃ
		
		if((CollectDataEnable ==1)||((DataRequestFlag &0x0F) ==0x01))
		{		
		    DeviceConfig.Time_Sec  =systmtime.tm_sec;     //¼ÇÂ¼Êı¾İ²É¼¯Ê±¼ä
		    DeviceConfig.Time_Min  =systmtime.tm_min;
		    DeviceConfig.Time_Hour =systmtime.tm_hour;
		    DeviceConfig.Time_Mday =systmtime.tm_mday;	
		    DeviceConfig.Time_Mon  =systmtime.tm_mon;    
			  if(systmtime.tm_year >=2000)
				{
					 DeviceConfig.Time_Year =systmtime.tm_year-2000; //¶ÔÉÏ´«Äê·İÈ¥»ùÊıĞŞÕı
				}
				else
				{
					 DeviceConfig.Time_Year =0;
        }
			  Sensor_Collect();                       //²É¼¯Êı¾İ
    }
    else
		{
       DS2780_Test();                           //´òÓ¡µç³ØµçÁ¿ĞÅÏ¢£¬±ãÓÚÏÖ³¡Õï¶Ï
       printf("\r\n433 Module Network ID is: %d (Ê®½øÖÆ)\r\n",GetNetworkID ()); //»ñÈ¡433Ä£¿éÍøÂçID£¬°ïÖúÎÊÌâ¶¨Î» 				
			 DeviceSleep(MonitorDeviceID[5]);
    }
		
		for(i=0;i<5;i++)
		{
			 Delay_ms(200);                                //²Î¿¼Ì½Í·ÉÏµçºóÊı¾İÎÈ¶¨Ê±¼ä£¬×÷ÏàÓ¦µ÷Õû
			 if(DMA_UART3_RECEV_FLAG==1)                   //²éÑ¯Êı¾İ½ÓÊÕÇé¿ö
			 {
				 DMA_UART3_RecevDetect(MonitorDeviceID, NodeAddr);  
				 break;
			 }
    }	
		if((DataRequestFlag &0x0F) ==0x01)   // »ñÈ¡Êı¾İ²éÑ¯×´Ì¬£¬µÍ4Î»
		{
       OpTypeCode =2;
			 RouteControl =DataRequestFlag>>4; // »ñÈ¡Â·ÓÉ±êÖ¾±äÁ¿£¬¸ß4Î»
    }
		else
		{
       OpTypeCode =4;
    }				 
   //Ê×ÏÈ»ñÈ¡ÀúÊ·Êı¾İ
		DataRead_From_Flash(0,10,0, &HistoricalDataCount,1);             //»ñÈ¡ÀúÊ·Êı¾İ³¤¶È
		if((HistoricalDataCount!=0)&&(HistoricalDataCount<=100))
		{
       #if DEBUG_TEST	 
			 printf("\r\n Historical data length:%d!\r\n",HistoricalDataCount); //²âÊÔÊ¹ÓÃ
			 #endif
			 DataRead_From_Flash(0,11,0, HistoricalDataSendBuff,HistoricalDataCount);             //»ñÈ¡ÀúÊ·Êı¾İÊıÁ¿
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
				  if(HistoricalDataRecevFlag ==0x058C)     //³É¹¦½ÓÊÕµ½TrapResponse
					{
							#if DEBUG_TEST	 
							printf("\r\n Historical data upload success!\r\n"); //²âÊÔÊ¹ÓÃ
							#endif
							break;     
				  }
			 }
    }
		
		TrapRequest(Usart2_send_buff, MonitorDeviceID, NodeAddr, OpTypeCode,RouteControl);  //·¢ËÍ×îĞÂÊı¾İ
    		
		printf("\r\n433 Module Network ID is: %d (Ê®½øÖÆ)\r\n",GetNetworkID ()); //»ñÈ¡433Ä£¿éÍøÂçID£¬°ïÖúÎÊÌâ¶¨Î» 	
		DeviceSleep(MonitorDeviceID[5]);
//		gotoSleep(60*3);   //²âÊÔÊ¹ÓÃ

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
