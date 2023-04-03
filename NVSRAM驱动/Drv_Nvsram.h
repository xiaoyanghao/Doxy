/**
  ******************************************************************************
  * @file    Drv_Nvsram.h
  * @author  YZH
  * @brief   ����ʧ��RAM����.
  */

#ifndef _DRV_NVSRAM_H
#define _DRV_NVSRAM_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "Data_Type.h" 


/** @addtogroup �豸����
  * @{
  */
/** @defgroup NVSRAM����
  * @brief   CY14B101Q2A-SXI.
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup NVSRAM�ⲿö����ṹ������
  * @{
  */ 

  /**
  * @brief �ڴ�д�����ȼ�
  */ 
 typedef enum
 {
   
   NVSRAM_BPL0 = 0x00,      /*!<  �ڴ治���� */ 
   NVSRAM_BPL1,             /*!<  �ڴ汣��  0x18000�C0x1FFFF */ 
   NVSRAM_BPL2,             /*!<  �ڴ汣��  0x10000�C0x1FFFF */ 
   NVSRAM_BPL3,             /*!<  �ڴ汣��  0x00000�C0x1FFFF*/ 
   
 }NVSRAM_BPL;
 
  /**
  * @brief NVSRAM����ֵ˵��
  */ 
 typedef enum
 {
   
   NVSRAM_SUCCEED,          /*!<  �ɹ� */ 
   NVSRAM_PRAM_ERRO,        /*!<  ����������� */ 
   NVSRAM_Busy,             /*!<  �豸æ */ 
   
 }NVSRAM_RET;

/**
  * @brief NVSRAM�ṹ��
*/ 
 typedef struct 
{
  uint8_t error;                   /*!<  �豸������*/ 
  uint8_t enabled;                 /*!<  �豸ʹ��*/ 
  uint32_t id;                     /*!<  �豸ID*/ 
  uint8_t snw;                     /*!<  �Ƿ�д���豸���к�*/ 
} Nvsram_Struct_t;  

/**
* @}
*/  

 /** @defgroup NVSRAM�ⲿ��
  * @{
  */ 
#define Drv_Nvsram_TSET   /*!<  �豸���� ���� */ 
 
/**
* @}
*/  
 
 /* Exported function ------------------------------------------------------------*/
/** @defgroup NVSRAM�ⲿ���� NVSRAM�ⲿ���� 
  * @{
  */ 
uint8_t Nvsram_Init(void);
uint8_t Nvsram_Set_Bp(uint8_t Level);
uint8_t Nvsram_Get_Bp(void);
uint32_t Nvsram_Burst_Write_Buff(uint32_t nvsram_addr, uint8_t *w_buf, uint32_t len);
uint32_t Nvsram_Burst_Read_Buff(uint32_t nvsram_addr, uint8_t *r_buf, uint32_t len);
uint8_t Nvsram_Soft_Store(void);
uint8_t Nvsram_Soft_Recall(void);
uint8_t Nvsram_Set_SN(uint8_t *sn_buff);
void Nvsram_Get_SN(uint8_t *sn_buff);
void Nvsram_Get_Struct(Nvsram_Struct_t *nvsram_st);

#ifdef Drv_Nvsram_TSET
  void Nvsram_Test(void);
#endif
/**
  * @}
  */

 
/**
  * @}
  */

/**
  * @}
  */
 

#ifdef __cplusplus
}
#endif

#endif

/*****************************END OF FILE**************************************/




















