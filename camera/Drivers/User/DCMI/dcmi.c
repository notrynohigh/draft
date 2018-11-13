#include "sys.h"
#include "dcmi.h" 
#include "ov5640.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F746������
//DCMI��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

extern DCMI_HandleTypeDef hdcmi;
extern DMA_HandleTypeDef hdma_dcmi;

u8 ov_frame=0;  						//֡��
extern void jpeg_data_process(void);	//JPEG���ݴ�����

//DCMI DMA����
//mem0addr:�洢����ַ0  ��Ҫ�洢����ͷ���ݵ��ڴ��ַ(Ҳ�����������ַ)
//mem1addr:�洢����ַ1  ��ֻʹ��mem0addr��ʱ��,��ֵ����Ϊ0
//memblen:�洢��λ��,����Ϊ:DMA_MDATAALIGN_BYTE/DMA_MDATAALIGN_HALFWORD/DMA_MDATAALIGN_WORD
//meminc:�洢��������ʽ,����Ϊ:DMA_MINC_ENABLE/DMA_MINC_DISABLE
void DCMI_DMA_Init(u32 mem0addr,u32 mem1addr,u16 memsize,u32 memblen,u32 meminc)
{ 
    HAL_DMAEx_MultiBufferStart(&hdma_dcmi,(u32)&DCMI->DR,mem0addr,mem1addr,memsize);//����˫����
    __HAL_DMA_ENABLE_IT(&hdma_dcmi,DMA_IT_TC);    //������������ж�
    //HAL_DCMI_Start_DMA(&hdcmi, DCMI_MODE_CONTINUOUS, mem0addr, memsize * 2);
}

 
//DCMI,��������
void DCMI_Start(void)
{  
    __HAL_DMA_ENABLE(&hdma_dcmi); //ʹ��DMA
    DCMI->CR|=DCMI_CR_CAPTURE;          //DCMI����ʹ��
}

//DCMI,�رմ���
void DCMI_Stop(void)
{ 
    DCMI->CR&=~(DCMI_CR_CAPTURE);       //�رղ���
    while(DCMI->CR&0X01);               //�ȴ��������
    __HAL_DMA_DISABLE(&hdma_dcmi);//�ر�DMA
} 


//����һ֡ͼ������
//hdcmi:DCMI���
void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *thdcmi)
{
	jpeg_data_process();//jpeg���ݴ���
	ov_frame++; 
    //����ʹ��֡�ж�,��ΪHAL_DCMI_IRQHandler()������ر�֡�ж�
    __HAL_DCMI_ENABLE_IT(&hdcmi,DCMI_IT_FRAME);
}

void (*dcmi_rx_callback)(void);//DCMI DMA���ջص�����
//DMA2������1�жϷ�����
void DMA2_Stream1_IRQHandler_dd(void)
{
    if(__HAL_DMA_GET_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5)!=RESET)//DMA�������
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_dcmi,DMA_FLAG_TCIF1_5);//���DMA��������жϱ�־λ
        dcmi_rx_callback();	//ִ������ͷ���ջص�����,��ȡ���ݵȲ����������洦��
    } 
}
  



