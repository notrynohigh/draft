#include "sys.h"
#include "dcmi.h" 
#include "ov5640.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F746开发板
//DCMI驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

u8 ov_frame=0;  						//帧率
extern void jpeg_data_process(void);	//JPEG数据处理函数

//DCMI DMA配置
//mem0addr:存储器地址0  将要存储摄像头数据的内存地址(也可以是外设地址)
//mem1addr:存储器地址1  当只使用mem0addr的时候,该值必须为0
//memblen:存储器位宽,可以为:DMA_MDATAALIGN_BYTE/DMA_MDATAALIGN_HALFWORD/DMA_MDATAALIGN_WORD
//meminc:存储器增长方式,可以为:DMA_MINC_ENABLE/DMA_MINC_DISABLE
void DCMI_DMA_Init(u32 mem0addr,u32 mem1addr,u16 memsize,u32 memblen,u32 meminc)
{ 
    HAL_DMAEx_MultiBufferStart(&hdma_dcmi,(u32)&DCMI->DR,mem0addr,mem1addr,memsize);//开启双缓冲
    __HAL_DMA_ENABLE_IT(&hdma_dcmi,DMA_IT_TC);    //开启传输完成中断
    //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, mem0addr, memsize * 2);
}

 
//DCMI,启动传输
void DCMI_Start(void)
{  
    __HAL_DMA_ENABLE(&hdma_dcmi); //使能DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI捕获使能
}

//DCMI,关闭传输
void DCMI_Stop(void)
{ 
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //关闭捕获
    while(DCMI->CR&0X01);               //等待传输完成
    __HAL_DMA_DISABLE(&hdma_dcmi);//关闭DMA
} 


//捕获到一帧图像处理函数
//hdcmi:DCMI句柄
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *thdcmi)
{
	jpeg_data_process();//jpeg数据处理
	ov_frame++; 
    //重新使能帧中断,因为HAL_DCMI_IRQHandler()函数会关闭帧中断
    __HAL_DCMI_ENABLE_IT(&hdcmi,DCMI_IT_FRAME);
}

void (*dcmi_rx_callback)(void);//DCMI DMA接收回调函数
//DMA2数据流1中断服务函数
void DMA2_Stream1_IRQHandler_dd(void)
{
    if(__HAL_DMA_GET_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5)!=RESET)//DMA传输完成
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5);//清除DMA传输完成中断标志位
        dcmi_rx_callback();	//执行摄像头接收回调函数,读取数据等操作在这里面处理
    } 
}
  



