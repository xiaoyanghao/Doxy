/**
  ******************************************************************************
  * @file    Drv_Nvsram.h
  * @author  YZH
  * @brief   非易失性RAM驱动.
  */

#ifndef _DRV_NVSRAM_H
#define _DRV_NVSRAM_H

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "Data_Type.h" 


/** @addtogroup 设备驱动
  * @{
  */
/** @defgroup NVSRAM驱动
  * @brief   CY14B101Q2A-SXI.
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup NVSRAM外部枚举与结构体类型
  * @{
  */ 

  /**
  * @brief 内存写保护等级
  */ 
 typedef enum
 {
   
   NVSRAM_BPL0 = 0x00,      /*!<  内存不保护 */ 
   NVSRAM_BPL1,             /*!<  内存保护  0x18000–0x1FFFF */ 
   NVSRAM_BPL2,             /*!<  内存保护  0x10000–0x1FFFF */ 
   NVSRAM_BPL3,             /*!<  内存保护  0x00000–0x1FFFF*/ 
   
 }NVSRAM_BPL;
 
  /**
  * @brief NVSRAM返回值说明
  */ 
 typedef enum
 {
   
   NVSRAM_SUCCEED,          /*!<  成功 */ 
   NVSRAM_PRAM_ERRO,        /*!<  传入参数错误 */ 
   NVSRAM_Busy,             /*!<  设备忙 */ 
   
 }NVSRAM_RET;

/**
  * @brief NVSRAM结构体
*/ 
 typedef struct 
{
  uint8_t error;                   /*!<  设备不正常*/ 
  uint8_t enabled;                 /*!<  设备使能*/ 
  uint32_t id;                     /*!<  设备ID*/ 
  uint8_t snw;                     /*!<  是否写入设备序列号*/ 
} Nvsram_Struct_t;  

/**
* @}
*/  

 /** @defgroup NVSRAM外部宏
  * @{
  */ 
#define Drv_Nvsram_TSET   /*!<  设备测试 开关 */ 
 
/**
* @}
*/  
 
 /* Exported function ------------------------------------------------------------*/
/** @defgroup NVSRAM外部函数 NVSRAM外部函数 
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




















