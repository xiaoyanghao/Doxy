/**
  ******************************************************************************
  * @file    Drv_Nvsram.c
  * @author  YZH
  * @brief   ����ʧ��RAM����.
  *
  @verbatim
  ==============================================================================
                     ##### оƬ���� #####
  ==============================================================================
[..]
   (+) оƬ�ͺ�   �� CY14B101Q2A-SXI
   (+) �ڴ�       �� 128K * 8 bit
   (+) ��ַ       �� 0x00000-0x1FFFF
   (+) ������ѹ   �� 3V
   (+) �����¶�   ��-40~85��
   (+) �豸ID     :  0x06818820
   (+) ���ݱ���ʱ�䣨�����¶�85�棩 �� 20��
   (+) ͨ�ŷ�ʽ  �� SPI ģʽ0��ģʽ3
   (+) ʱ��Ƶ��  �� ������д ���40MHZ  �����ʽ��� 104MHZ
   (+) д����    �� ֧�� 1/4 1/2 1/1 0 �ڴ汣��
   (+) ������ʽ  �� ���ڲ�������ģ����� SRAM �����綪ʧ���� �������壨���籣�棩
                   ������д��SRAM�У�����Ĺ����У��ⲿ���ӵ���ʣ��ĵ�������SRAM
                   �е����ݱ��浽�������嵥Ԫ��ʵ�ֵ������ݱ������ϵ�֮�󣬻��Զ�
                   ���������嵥Ԫ�����ݰ��˵�SRAM�У�ʵ���ϵ���أ�ͬʱҲ��������
                   �������ʵ�����ݱ��档  
  @endverbatim
  ******************************************************************************
  * @attention
   <pre>
         (+)  ���ݽ���ͻ��д����ʱ����ͻ��д����һ���ܱ����Ŀ��ַʱ��������ַ��
              ���������ܱ����ռ䣬���ǲ����ܱ����ڴ�д���κ����ݡ������ַ��ת��
              ͻ��д��������δ�ܱ����Ŀռ䣬Ȼ��ָ�д�롣
         (+)  ���ݽ���ͻ��д�������߶�����ʱ�������ַ����0x1FFFF����ַ���Զ�
              ��0x00000����������д����߶�ȡ��
         (+)  �ڽ���д�ڴ棬д�Ĵ��������߱��棬���ء��Զ�����ʹ�ܣ��Զ����治ʹ
               �ܲ���֮��дʹ�ܽ�����ֹ��ͬʱ״̬�Ĵ���WEN bitΪ0����һ�ν���д
               ����ʱ���뽫����дʹ�ܡ�
         (+)  оƬ�ڲ���8���ֽڵ����кſռ䣬��ֻ�ܱ�дһ�Σ����浽��������֮�󣩣�
              дһ��֮��״̬�Ĵ�����SNL bit Ϊ1��������Զ���ܲ����ñ�־λ�����к�
              ��Զ�����ٱ����ġ�
         (+)  ����豸������д�������������� �����SPIͨ������
                                               ����NVSRAM_BUF_SIZE������
    </pre>
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Drv_Nvsram.h"
#include "Drv_Spi.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup �豸����
  * @{
  */
/** @defgroup NVSRAM����
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private function macro ---------------------------------------------------------*/

/** @addtogroup NVSRAM��̬������
  * @{
  */
/**
  * @brief   NVSRAM ������ʱ.
  * @param[in]  MS: ��ʱ N ms
  */
#define Nvsram_Delay_Ms(NMS)                               HAL_Delay(NMS)
    
/**
  * @brief   NVSRAM ��ȡϵͳʱ���.
  * @retval  ϵͳʱ��� ����
  */
#define Nvsram_Get_tick()                                  HAL_GetTick()                                                                  
    
/**
  * @brief  NVSRAM SPIͨ�Žӿ�
  * @param[in]   txbuf: ���ͻ�������ַ
  * @param[out]  rxbuf: ���ջ�������ַ
  * @param[in]   len: �����ֽ���
  * @retval      RET_OK���ɹ� ; 
    @retval      RET_ERR:ʧ�� 
  */
#define Nvsram_Spi_TR(txbuf,rxbuf,len)                    SPI_TransmitReceive(SPI_CH1, txbuf, rxbuf, len)

/**
  * @}
  */ 
/** @addtogroup NVSRAM�豸����
  * @{
  */
#define NVSRAM_STATUS_RDSR                                        0x05   /*!<  ��״̬�Ĵ��� */ 
#define NVSRAM_STATUS_FRDSR                                       0x0A   /*!<  ���״̬�Ĵ��� */ 
#define NVSRAM_STATUS_WRSR                                        0x01   /*!<  дת̬�Ĵ���*/ 
#define NVSRAM_STATUS_WREN                                        0x06   /*!<  дʹ��*/ 
#define NVSRAM_STATUS_WRDI                                        0x04   /*!<  ��λдʹ�� */ 

#define NVSRAM_SRAM_READ                                          0x03  /*!<  ���ڴ� */ 
#define NVSRAM_SRAM_FREAD                                         0x0B  /*!<  ����ڴ�*/ 
#define NVSRAM_SRAM_WRITE                                         0x02  /*!<  д�ڴ� */ 

#define NVSRAM_SPEC_NV_STORE                                      0x3C  /*!<  �ڴ����ݴ洢 */ 
#define NVSRAM_SPEC_NV_RECALL                                     0x60  /*!<  �ڴ����ݼ��� */ 
#define NVSRAM_SPEC_NV_ASENB                                      0x59  /*!<  �Զ�����ʹ�� */ 
#define NVSRAM_SPEC_NV_ASDISB                                     0x19  /*!<  �Զ������ֹ */ 

#define NVSRAM_SPEC_SLEEP                                         0xB9  /*!<  ˯��ģʽʹ�� */ 
#define NVSRAM_SPEC_WRSN                                          0xC2  /*!<  д���к� */ 
#define NVSRAM_SPEC_RDSN                                          0xC3  /*!<  �����к� */ 
#define NVSRAM_SPEC_FRDSN                                         0xC9  /*!<  ������к� */ 
#define NVSRAM_SPEC_RDID                                          0x9F  /*!<  ���豸ID */ 
#define NVSRAM_SPEC_FRDID                                         0x99  /*!<  ����豸ID */ 

#define NVSRAM_STATUS_RDY                                         0x01 /*!<  ״̬�Ĵ��� ����λ*/ 
#define NVSRAM_STATUS_WEN                                         0x02 /*!<  ״̬�Ĵ��� дʹ��λ*/ 
#define NVSRAM_STATUS_BP0                                         0x04 /*!<  ״̬�Ĵ��� �鱣�� 0λ*/  
#define NVSRAM_STATUS_BP1                                         0x08 /*!<  ״̬�Ĵ��� �鱣�� 1λ*/ 
#define NVSRAM_STATUS_SNL                                         0x40 /*!<  ״̬�Ĵ��� ���к�����λ*/ 
#define NVSRAM_STATUS_WPEN                                        0x80 /*!<  ״̬�Ĵ��� д��������ʹ��λ*/ 

#define NVSRAM_DEV_ID                                             0x06818820 /*!<  �豸ID*/ 
#define NVSRAM_MAX_ADDR                                           0x1FFFF    /*!<  �豸����ַ*/
#define NVSRAM_BUF_SIZE                                           256        /*!<  �豸һ�η�������ֽ���*/

/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
static Nvsram_Struct_t _nvsram = {0};
static uint8_t _nvsram_txbuf[NVSRAM_BUF_SIZE + 4] = {0}; /*!<  ǰ4���ֽ� 1���ֽ�������+3���ֽڵ�ַ */ 
static uint8_t _nvsram_rxbuf[NVSRAM_BUF_SIZE + 4] = {0}; /*!<  ǰ4���ֽ� 1���ֽ�������+3���ֽڵ�ַ */ 

/* Private function ---------------------------------------------------------*/
/** @addtogroup NVSRAM��̬����
  * @{
  */

static int8_t  nvsram_check_id(void);
static int8_t  nvsram_get_ready(void);
static int8_t  nvsram_get_snw(void);

/**
  * @brief   NVSRAM ����豸ID
  * @retval  0:�ɹ� 
    @retval  -1:ʧ�� 
  */
static int8_t  nvsram_check_id(void)
{
  int8_t ret = 0;
  uint8_t dec = 10;
  uint8_t txbuf[5] = {0};
  uint8_t rxbuf[5] = {0};
  txbuf[0] = NVSRAM_SPEC_RDID;
  do{  
      Nvsram_Spi_TR(txbuf, rxbuf, 5);
      dec--;
      _nvsram.id = ((rxbuf[1]<<24) | (rxbuf[2]<<16) | (rxbuf[3]<<8) | rxbuf[4]);
  }while(_nvsram.id != NVSRAM_DEV_ID && (dec!=0));
  if(dec == 0){
    ret = -1;
  }
  return ret;
}

/**
  * @brief   NVSRAM ��ȡ����״̬���ϵ�ȴ���
  * @retval  0:�ɹ� 
    @retval -1:ʧ�� 
  */
static int8_t  nvsram_get_ready(void)
{
  int8_t ret = 0;
  uint16_t dec = 0xffff;
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
  txbuf[0] = NVSRAM_STATUS_RDSR;
  do{  
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      dec--;
  }while((rxbuf[1] & NVSRAM_STATUS_RDY) != 0x00 && (dec!=0));
  if(dec == 0){
    ret = -1;
  }
  return ret;  
}

/**
  * @brief   NVSRAM ���к��Ƿ�д��(�ϵ���)
  * @retval  0:�ɹ� 
    @retval -1:ʧ�� 
  */
static int8_t nvsram_get_snw(void)
{
  int8_t ret = 0;
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
  txbuf[0] = NVSRAM_STATUS_RDSR;
  Nvsram_Spi_TR(txbuf, rxbuf, 2);
  if((rxbuf[1] & NVSRAM_STATUS_SNL) != NVSRAM_STATUS_SNL){
    ret = -1;
  }
  return ret;  
}

/**
  * @brief   NVSRAM ����дʹ��
  * @note   ÿ��д����֮ǰ��Ҫ���� 
  * @note   ״̬�Ĵ������������ж� : �豸����λΪ0  дʹ��λΪ1
  * @retval  0:�ɹ� 
    @retval -1:ʧ�� 
  */
static int8_t  nvsram_verify_set_wen(void)
{
  int8_t ret = 0;
  uint8_t dec = 5;
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
  txbuf[0] = NVSRAM_STATUS_WREN;
  do{  
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      txbuf[0] = NVSRAM_STATUS_RDSR;
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      dec--;
  }while((rxbuf[1] & (NVSRAM_STATUS_RDY | NVSRAM_STATUS_WEN )) != 0x02 && (dec!=0));
  if(dec == 0){
    ret = -1;
  }
  return ret;   
}
/**
  * @}
  */


/** @addtogroup NVSRAM�ⲿ����
  * @{
  */
/**
  * @brief   NVSRAM ����д����.
  * @param[in]  Level: ����д�����ȼ� 
  *          @arg @ref NVSRAM_BPL0  0   �ڴ治���� 
  *          @arg @ref NVSRAM_BPL1  1/4 �ڴ汣�� 0x18000�C0x1FFFF
  *          @arg @ref NVSRAM_BPL2  1/2 �ڴ汣�� 0x10000�C0x1FFFF
  *          @arg @ref NVSRAM_BPL3  ALL �ڴ汣�� 0x00000�C0x1FFFF
  *  @retval NVSRAM_SUCCEED:�ɹ�  
  *  @retval NVSRAM_PRAM_ERRO:����������� 
  *  @retval NVSRAM_Busy:�豸æ 
  */
uint8_t Nvsram_Set_Bp(uint8_t Level)
{
  uint8_t ret = NVSRAM_SUCCEED;
  uint8_t dec = 5;
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
 
  if(Level == NVSRAM_BPL0){
    txbuf[1] = 0x00;
  }
  else if(Level == NVSRAM_BPL1){
    txbuf[1] = 0x04;
  }
  else if(Level == NVSRAM_BPL2){
    txbuf[1] = 0x08;
  }
  else if(Level == NVSRAM_BPL3){
    txbuf[1] = 0x0C;
  }
  else{
     ret = NVSRAM_PRAM_ERRO;
     goto nvram_bp_erro;
  }
  do{  
      if(nvsram_verify_set_wen() == -1){
       ret = NVSRAM_Busy;
       goto nvram_bp_erro;
      }
      txbuf[0] = NVSRAM_STATUS_WRSR;
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      txbuf[0] = NVSRAM_STATUS_RDSR;
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      dec--;
  }while((rxbuf[1] & (NVSRAM_STATUS_BP1 | NVSRAM_STATUS_BP0 )) != txbuf[1] && (dec!=0));
  if(dec == 0){
    ret = NVSRAM_Busy;
    goto nvram_bp_erro;
  }
nvram_bp_erro:
    return  ret;
}

/**
  * @brief   NVSRAM ��ȡд������.
  * @retval  NVSRAM_BPL0  0   �ڴ治���� 
  * @retval  NVSRAM_BPL1  1/4 �ڴ汣�� 0x18000�C0x1FFFF
  * @retval  NVSRAM_BPL2  1/2 �ڴ汣�� 0x10000�C0x1FFFF
  * @retval  NVSRAM_BPL3  ALL �ڴ汣�� 0x00000�C0x1FFFF
  */
uint8_t Nvsram_Get_Bp(void)
{
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
  uint8_t rxbp = 0;
  uint8_t ret = 0;
  txbuf[0] = NVSRAM_STATUS_RDSR;
  Nvsram_Spi_TR(txbuf, rxbuf, 2);
  rxbp = (rxbuf[1] & (NVSRAM_STATUS_BP1 | NVSRAM_STATUS_BP0 ));
  if(rxbp == 0x00){
    ret = NVSRAM_BPL0;
  }
  else if(rxbp == 0x04){
    ret = NVSRAM_BPL1;
  }
  else if(rxbp == 0x08){
    ret = NVSRAM_BPL2;
  }
  else if(rxbp == 0x0C){
    ret = NVSRAM_BPL3;
  }
  else{
  }
  return ret;
}

/**
  * @brief   NVSRAM д�ڴ溯��
  * @note    1) ���ݽ���ͻ��д����ʱ����ͻ��д����һ���ܱ����Ŀ��ַʱ��������ַ��
  * @note       ���������ܱ����ռ䣬���ǲ����ܱ����ڴ�д���κ����ݡ������ַ���ӵ�
  * @note       ͻ��д��������δ�ܱ����Ŀռ䣬Ȼ��ָ�д��
  * @note    2) ���ݽ���ͻ��д����ʱ�������ַ����0x1FFFF����ַ���Զ���0x00000����������д��  
  * @note    3) ��������СΪ 256Byte, SPIͨ������Ϊ 12.5MHZ д����Ϊ 262.83KByte/S     
  * @param[in]  nvsram_addr:NVSRAM�ڴ��ַ 0x00000�C0x1FFFF
  * @param[in]  w_buf:д��������ַ
  * @param[in]  len:д���ֽڳ���
  * @retval  д���ֽ�����
  */
uint32_t Nvsram_Burst_Write_Buff(uint32_t nvsram_addr, uint8_t *w_buf, uint32_t len)
{
  uint32_t ret = 0;
  uint32_t txlen = len;
  uint32_t addr = nvsram_addr;
  if(addr > NVSRAM_MAX_ADDR || len == 0){
    goto nvram_w_erro; 
  }
  do{
    if(nvsram_verify_set_wen() == -1){
      goto nvram_w_erro;       
    }
    _nvsram_txbuf[0] = NVSRAM_SRAM_WRITE;
    _nvsram_txbuf[1] = (addr&0x0f0000)>>16;
    _nvsram_txbuf[2] = (addr&0x00ff00)>>8;
    _nvsram_txbuf[3] = (addr&0x0000ff);
    if(txlen/NVSRAM_BUF_SIZE > 0){
      memcpy(&_nvsram_txbuf[4], w_buf, NVSRAM_BUF_SIZE);
      Nvsram_Spi_TR(_nvsram_txbuf, _nvsram_rxbuf, NVSRAM_BUF_SIZE+4);
      w_buf += NVSRAM_BUF_SIZE;
      txlen -= NVSRAM_BUF_SIZE;
      ret += NVSRAM_BUF_SIZE;
      addr += NVSRAM_BUF_SIZE;
    }
    else{
      memcpy(&_nvsram_txbuf[4], w_buf, txlen);
      Nvsram_Spi_TR(_nvsram_txbuf, _nvsram_rxbuf, txlen+4);
      ret += txlen;
      txlen = 0;
    }
  }while(txlen>0);
nvram_w_erro:
    return  ret;
}

/**
  * @brief   NVSRAM ���ڴ溯��
  * @note    ���ݽ��ж�����ʱ,�����ַ����0x1FFFF,��ַ���Զ���0x00000���������Ӷ�ȡ
  * @note    ��������СΪ 256Byte, SPIͨ������Ϊ 12.5MHZ ������Ϊ 262.83KByte/S   
  * @param[in]  nvsram_addr: NVSRAM�ڴ��ַ 0x00000�C0x1FFFF
  * @param[out]  r_buf:����������ַ
  * @param[in]   len:���ֽڳ���
  * @retval  �����ֽ�����
  */
uint32_t Nvsram_Burst_Read_Buff(uint32_t nvsram_addr, uint8_t *r_buf, uint32_t len)
{
  uint32_t ret = 0;
  uint32_t rxlen = len;
  uint32_t addr = nvsram_addr;
  if(addr > NVSRAM_MAX_ADDR || len == 0){
    goto nvram_r_erro; 
  }
  do{
    _nvsram_txbuf[0] = NVSRAM_SRAM_READ;
    _nvsram_txbuf[1] = (addr&0x0f0000)>>16;
    _nvsram_txbuf[2] = (addr&0x00ff00)>>8;
    _nvsram_txbuf[3] = (addr&0x0000ff);
    if(rxlen/NVSRAM_BUF_SIZE > 0){
      Nvsram_Spi_TR(_nvsram_txbuf, _nvsram_rxbuf, NVSRAM_BUF_SIZE+4);
      memcpy(r_buf, &_nvsram_rxbuf[4], NVSRAM_BUF_SIZE);
      r_buf += NVSRAM_BUF_SIZE;
      rxlen -= NVSRAM_BUF_SIZE;
      ret += NVSRAM_BUF_SIZE;
      addr += NVSRAM_BUF_SIZE;
    }
    else{  
      Nvsram_Spi_TR(_nvsram_txbuf, _nvsram_rxbuf, rxlen + 4);
      memcpy(r_buf, &_nvsram_rxbuf[4], rxlen);
      ret += rxlen;
      rxlen = 0;
    }
  }while(rxlen>0);
nvram_r_erro:
    return  ret;  
}

/**
  * @brief   NVSRAM ����洢
  * @note    ���ڲ�SDRAM���ݴ洢������������,���ڴ洢��Ҫһ����ʱ��,����豸���Ե��籣��,
  * @note    ������ʹ�øú���
  * @retval  NVSRAM_SUCCEED:�ɹ�  
  * @retval  NVSRAM_PRAM_ERRO:����������� 
  * @retval  NVSRAM_Busy:�豸æ 
  */
uint8_t Nvsram_Soft_Store(void)
{
  uint8_t ret = NVSRAM_SUCCEED;
  uint16_t dec = 0xffff;
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
  if(nvsram_verify_set_wen() == -1){
      ret = NVSRAM_Busy;
      goto nvram_store_erro;       
  }
  txbuf[0] = NVSRAM_SPEC_NV_STORE;
  Nvsram_Spi_TR(txbuf, rxbuf, 2);
  do{  
      txbuf[0] = NVSRAM_STATUS_RDSR;
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      dec--;
  }while((rxbuf[1] & NVSRAM_STATUS_RDY) != 0x00 && (dec!=0));
  if(dec == 0){
    ret = NVSRAM_Busy;
  }
nvram_store_erro:
  return ret;
}

/**
  * @brief   NVSRAM �����������
  * @note    ���������������ݼ��ص��ڲ�SDRAM������,���ڴ洢��Ҫһ����ʱ��,����豸�����ϵ����,
  * @note    ������ʹ�øú���
  * @retval  NVSRAM_SUCCEED:�ɹ�  
  * @retval  NVSRAM_PRAM_ERRO:����������� 
  * @retval  NVSRAM_Busy:�豸æ 
  */

uint8_t Nvsram_Soft_Recall(void)
{
  uint8_t ret = NVSRAM_SUCCEED;
  uint16_t dec = 0xffff;
  uint8_t txbuf[2] = {0};
  uint8_t rxbuf[2] = {0};
  if(nvsram_verify_set_wen() == -1){
      ret = NVSRAM_Busy;
      goto nvram_recall_erro;       
  }
  txbuf[0] = NVSRAM_SPEC_NV_RECALL;
  Nvsram_Spi_TR(txbuf, rxbuf, 2);
  do{  
      txbuf[0] = NVSRAM_STATUS_RDSR;
      Nvsram_Spi_TR(txbuf, rxbuf, 2);
      dec--;
  }while((rxbuf[1] & NVSRAM_STATUS_RDY) != 0x00 && (dec!=0));
  if(dec == 0){
    ret = NVSRAM_Busy;
  }
nvram_recall_erro:
  return ret;
}

/**
  * @brief   NVSRAM �������к�
  * @note   �ڲ���8���ֽڵ����кſռ䣬��ֻ�ܱ�дһ��(���浽��������֮��),
  * @note   дһ��֮��,״̬�Ĵ�����SNL bit Ϊ1,������Զ���ܲ����ñ�־λ,���к�
  * @note   ��Զ�����ٱ�����,������Ҫ�洢����,�����ʱ6ms����,�����齫�˺��������߳���ʹ��
  * @param[in]  sn_buff: ���кŻ����������ַ �û�����Ҫ���ڵ���8���ֽ� 
  * @retval  NVSRAM_SUCCEED:�ɹ�  
  * @retval  NVSRAM_PRAM_ERRO:����������� 
  * @retval  NVSRAM_Busy:�豸æ 
  */
uint8_t Nvsram_Set_SN(uint8_t *sn_buff)
{
  uint8_t ret = NVSRAM_SUCCEED;
  uint8_t txbuf[9] = {0};
  uint8_t rxbuf[9] = {0};
  if(_nvsram.snw == 1){
    ret = NVSRAM_PRAM_ERRO;
    goto nvram_wrsn_erro;  
  }
  if(nvsram_verify_set_wen() == -1){
      ret = NVSRAM_Busy;
      goto nvram_wrsn_erro;       
  }
  txbuf[0] = NVSRAM_SPEC_WRSN;
  memcpy(&txbuf[1], sn_buff, 8);
  Nvsram_Spi_TR(txbuf, rxbuf, 9);
  if(Nvsram_Soft_Store() != NVSRAM_SUCCEED){
    ret = NVSRAM_Busy;
    goto nvram_wrsn_erro; 
  }
  txbuf[0] = NVSRAM_STATUS_RDSR;
  Nvsram_Spi_TR(txbuf, rxbuf, 2);
  if((rxbuf[1] & NVSRAM_STATUS_SNL) != NVSRAM_STATUS_SNL){
    ret = NVSRAM_Busy;
  }
  else{
    _nvsram.snw = 1;
  }
nvram_wrsn_erro:
  return ret;
}

/**
  * @brief   NVSRAM ��ȡ���к�
  * @param[out]  sn_buff: ���кŻ����������ַ �û�����Ҫ���ڵ���8���ֽ� 
  */
void Nvsram_Get_SN(uint8_t *sn_buff)
{
  uint8_t txbuf[9] = {0};
  uint8_t rxbuf[9] = {0};
  txbuf[0] = NVSRAM_SPEC_RDSN;
  Nvsram_Spi_TR(txbuf, rxbuf, 9);
  memcpy(sn_buff,&rxbuf[1],8);
}

/**
  * @brief   NVSRAM �豸��ʼ��
  * @note    Ӳ����ʼ�� �ȴ��豸����,����ID��,�������к�д���־
  * @retval  NVSRAM_SUCCEED:�ɹ�  
  * @retval  NVSRAM_PRAM_ERRO:����������� 
  * @retval  NVSRAM_Busy:�豸æ 
 */
uint8_t Nvsram_Init(void)
{
   uint8_t ret = NVSRAM_SUCCEED;
   SPI_Init(SPI_CH1, SPI_BAUDRATE_12_5M);
   //�ȴ����ݼ���
   Nvsram_Delay_Ms(10); 
   if(nvsram_get_ready() != 0){
     _nvsram.error = 1;
     ret = NVSRAM_Busy;
     goto nvsram_init_erro;
   }
   _nvsram.enabled = 1;
   if(nvsram_check_id() != 0){
     _nvsram.error = 1;
     ret = NVSRAM_Busy;
     goto nvsram_init_erro;
   }
   if(nvsram_get_snw() == 0){
     _nvsram.snw = 1;
   }
nvsram_init_erro:
   return ret;
}

/**
  * @brief   NVSRAM ��ȡ�豸״̬
  * @param[out]  nvsram_st �豸״̬�ṹ���ַ
 */
void Nvsram_Get_Struct(Nvsram_Struct_t *nvsram_st)
{
  memcpy(&_nvsram, nvsram_st, sizeof(Nvsram_Struct_t));
}

#ifdef Drv_Nvsram_TSET
uint8_t test_tx[NVSRAM_BUF_SIZE] = {0};
uint8_t test_rx[NVSRAM_BUF_SIZE] = {0};
/**
  * @brief   NVSRAM ��������
  * @note    1)�豸��ʼ������,��������ʱ��,�豸ID,�豸���к��Ƿ�д��
  * @note    2)�豸�������ָ���� ��������ʱ��
  * @note    3)�豸������ݼ���ָ���� ��������ʱ�� 
  * @note    4)�豸д�������ԣ�һ��дһ����������
  * @note    5)�豸���������ԣ�һ�ζ�һ����������
  * @note    6)�豸д�������ԣ�128K�ڴ棩����д������ʱ��
  * @note    7)�豸���������ԣ�128K�ڴ棩����д��������ݵ���ȷ��  
  * @note    8)�豸���������ԣ�128K�ڴ棩���Զ�������ʱ�� 
  * @note    9)�豸�������ݱ������ ����״̬���ڴ���д������ 0x55
  * @note      �ϵ�����,�豸��ʼ��֮��,�������ݣ��ж�����ȫΪ0x55      
  * @note    10)�豸�鱣������  NVSRAM_BPL0 ��128K�ڴ棩д��0x11  ���������ж��Ƿ�һ��
  * @note                       NVSRAM_BPL1 ��128K�ڴ棩д��0x22  ���������ж��Ƿ�һ��
  * @note                       NVSRAM_BPL2 ��128K�ڴ棩д��0x33  ���������ж��Ƿ�һ��
  * @note                       NVSRAM_BPL3 ��128K�ڴ棩д��0x44  ���������ж��Ƿ�һ�� 
  * @note                       ����鱣��  ��128K�ڴ棩д��0x55  ���������ж��Ƿ�һ�� 
 */
void Nvsram_Test(void)
{
   uint32_t start = 0,end = 0,txlen = 0,rxlen = 0,bp_start = 0, bp_len= 0;
   uint8_t ret = 0;
   uint32_t idx = 0,j = 0;
   
   for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
     test_tx[idx] = (uint8_t) idx;
   }
   
   printf("\r\n1)NVSRAM ��ʼ������ ����\n");
   start = Nvsram_Get_tick();
   ret = Nvsram_Init();
   end = Nvsram_Get_tick();
   if(ret == NVSRAM_SUCCEED){
     printf("\rNVSRAM ��ʼ���ɹ�, ������ʱ %d MS\n",end - start); 
     printf("\rNVSRAM �豸ID 0x%x\n",_nvsram.id);
     if(_nvsram.snw == 1){
       printf("\rNVSRAM �豸���к���д��\n");
     }
     else{
       printf("\rNVSRAM �豸���к�δд��\n"); 
     }
   }
   
   Nvsram_Burst_Read_Buff(0x00000, test_rx, 1);
//   if(test_rx[0] == 0x55){
//     printf("\r\n9)NVSRAM ���籣�������(�ڴ�128K ��ַ 0X00000~0X1FFFF) ��֤�����Ƿ���ȷ����\n");
//     rxlen = 0;
//     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
//       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
//       for(j=0; j<NVSRAM_BUF_SIZE; j++){
//         if(test_rx[j] != 0x55){
//           printf("\rNVSRAM �������� ��ַ 0x%x\n",idx * NVSRAM_BUF_SIZE  + j);
//         }   
//       }
//     }
//     printf("\rNVSRAM �����ֽ���%d\n",rxlen);
//   }
   
   printf("\r\n2)NVSRAM ���ݱ��溯�� ����\n");
   start = Nvsram_Get_tick();
   ret = Nvsram_Soft_Store();
   end = Nvsram_Get_tick();
   if(ret == NVSRAM_SUCCEED){
     printf("\rNVSRAM ���ݱ���ɹ�, ������ʱ %d MS\n",end - start); 
   }
   printf("\r\n3)NVSRAM ���ݼ��غ��� ����\n");
   start = Nvsram_Get_tick();
   ret = Nvsram_Soft_Recall();
   end = Nvsram_Get_tick();
   if(ret == NVSRAM_SUCCEED){
     printf("\rNVSRAM ���ݼ��سɹ�, ������ʱ %d MS\n",end - start); 
   }   
   
   printf("\r\n4)NVSRAM д����(һ��д��%d���ֽ�) ����\n", NVSRAM_BUF_SIZE);
   start = Nvsram_Get_tick();
   txlen = Nvsram_Burst_Write_Buff(0x00000, test_tx, NVSRAM_BUF_SIZE);
   end = Nvsram_Get_tick();
   printf("\rNVSRAM д���ֽ���%d, ������ʱ %d MS\n",txlen, end - start);
   
   printf("\r\n5)NVSRAM ������(һ�ζ���%d���ֽ�) ����\n", NVSRAM_BUF_SIZE);
   start = Nvsram_Get_tick();
   rxlen = Nvsram_Burst_Read_Buff(0x00000, test_rx, NVSRAM_BUF_SIZE);
   end = Nvsram_Get_tick();
   printf("\rNVSRAM �����ֽ���%d, ������ʱ %d MS\n", rxlen, end - start);
   for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
     if(test_rx[idx] != test_tx[idx]){
       printf("\rNVSRAM �������� ��ַ %d\n",idx);
     }   
   }
   
   printf("\r\n6)NVSRAM д����(�ڴ�128K ��ַ 0X00000~0X1FFFF) ����\n");
   txlen = 0;
   start = Nvsram_Get_tick();
   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
      txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
   }
   end = Nvsram_Get_tick();
   printf("\rNVSRAM д���ֽ���%d, ��ʱ %d MS, %f KByte/s\n",txlen, end - start, (NVSRAM_MAX_ADDR+1)/1024*1000/((end-start) * 1.0));
 
   printf("\r\n7)NVSRAM ������(�ڴ�128K ��ַ 0X00000~0X1FFFF) ��֤�����Ƿ���ȷ����\n");
   rxlen = 0;
   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
     rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
     for(j=0; j<NVSRAM_BUF_SIZE; j++){
       if(test_rx[j] != test_tx[j]){
         printf("\rNVSRAM �������� ��ַ 0x%x\n",idx * NVSRAM_BUF_SIZE  + j);
       }   
     }
   }
   printf("\rNVSRAM �����ֽ���%d\n",rxlen);
 
   printf("\r\n8)NVSRAM ������(�ڴ�128K ��ַ 0X00000~0X1FFFF) ���ʲ���\n");
   rxlen = 0;
   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
     rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
   }
   printf("\rNVSRAM �����ֽ���%d, ��ʱ %d MS, %f KByte/s\n",rxlen, end - start, (NVSRAM_MAX_ADDR+1)/1024*1000/((end-start) * 1.0));
   
//   printf("\r\n9)NVSRAM ���籣��(�ڴ�128K ��ַ 0X00000~0X1FFFF) ���ԣ�д�룩\n");
//   for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
//     test_tx[idx] = (uint8_t) 0x55;
//   }
//   txlen = 0;
//   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
//      txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
//   }
//   printf("\rNVSRAM д���ֽ���%d\n",txlen);
   
   printf("\r\n10-1)NVSRAM �鱣�� NVSRAM_BPL0 (�ڴ治����) ����\n");
   bp_start = 0;
   bp_len = 0;
   ret = Nvsram_Set_Bp(NVSRAM_BPL0);
   if(ret == NVSRAM_SUCCEED){
     for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
       test_tx[idx] = (uint8_t) 0x11;
     }
     
     txlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
     }
     printf("\rNVSRAM д���ֽ���%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
              bp_start = idx * NVSRAM_BUF_SIZE + j;
              printf("\rNVSRAM �ڴ汣����ʼ��ַ 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM �����ֽ���%d, �ڴ汣����С%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL0){
       printf("\rNVSRAM �鱣�� NVSRAM_BPL0 ���óɹ�");
     }
   }
   
   printf("\r\n10-2)NVSRAM �鱣�� NVSRAM_BPL1 (0x18000�C0x1FFFF) ����\n");
   bp_start = 0;
   bp_len = 0;
   ret = Nvsram_Set_Bp(NVSRAM_BPL1);
   if(ret == NVSRAM_SUCCEED){
     for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
       test_tx[idx] = (uint8_t) 0x22;
     }
     
     txlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
     }
     printf("\rNVSRAM д���ֽ���%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
             bp_start = idx * NVSRAM_BUF_SIZE + j;
             printf("\rNVSRAM �ڴ汣����ʼ��ַ 0x%x\n",bp_start);
           }
           bp_len++;
         }  
         
       }
     }
     printf("\rNVSRAM �����ֽ���%d, �ڴ汣����С%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL1){
       printf("\rNVSRAM �鱣�� NVSRAM_BPL1 ���óɹ�\n");
     }
   }
   
   printf("\r\n10-3)NVSRAM �鱣�� NVSRAM_BPL2 (0x10000�C0x1FFFF) ����\n");
   bp_start = 0;
   bp_len = 0;
   ret = Nvsram_Set_Bp(NVSRAM_BPL2);
   if(ret == NVSRAM_SUCCEED){
     for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
       test_tx[idx] = (uint8_t) 0x33;
     }
     
     txlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
     }
     printf("\rNVSRAM д���ֽ���%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
             bp_start = idx * NVSRAM_BUF_SIZE + j;
             printf("\rNVSRAM �ڴ汣����ʼ��ַ 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM �����ֽ���%d, �ڴ汣����С%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL2){
       printf("\rNVSRAM �鱣�� NVSRAM_BPL2 ���óɹ�\n");
     }
   }  
   
   printf("\r\n10-4)NVSRAM �鱣�� NVSRAM_BPL3 (0x00000�C0x1FFFF) ����\n");
   bp_start = 0;
   bp_len = 0;
   ret = Nvsram_Set_Bp(NVSRAM_BPL3);
   if(ret == NVSRAM_SUCCEED){
     for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
       test_tx[idx] = (uint8_t) 0x44;
     }
     
     txlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
     }
     printf("\rNVSRAM д���ֽ���%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0 && bp_len == 0){
             bp_start = idx * NVSRAM_BUF_SIZE + j;
             printf("\rNVSRAM �ڴ汣����ʼ��ַ 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM �����ֽ���%d, �ڴ汣����С%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL3){
       printf("\rNVSRAM �鱣�� NVSRAM_BPL3 ���óɹ�\n");
     }
   } 
   
   printf("\r\n10-5)NVSRAM �鱣�� ���(�ڴ治����) ����\n");
   bp_start = 0;
   bp_len = 0;
   ret = Nvsram_Set_Bp(NVSRAM_BPL0);
   if(ret == NVSRAM_SUCCEED){
     for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
       test_tx[idx] = (uint8_t) 0x55;
     }
     
     txlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
     }
     printf("\rNVSRAM д���ֽ���%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
              bp_start = idx * NVSRAM_BUF_SIZE + j;
              printf("\rNVSRAM �ڴ汣����ʼ��ַ 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM �����ֽ���%d, �ڴ汣����С%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL0){
       printf("\rNVSRAM �鱣�� ����ɹ�\n");
     }
   }  
   
   while(1);

}
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
/*****************************END OF FILE**************************************/










































