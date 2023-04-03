/**
  ******************************************************************************
  * @file    Drv_Nvsram.c
  * @author  YZH
  * @brief   非易失性RAM驱动.
  *
  @verbatim
  ==============================================================================
                     ##### 芯片介绍 #####
  ==============================================================================
[..]
   (+) 芯片型号   ： CY14B101Q2A-SXI
   (+) 内存       ： 128K * 8 bit
   (+) 地址       ： 0x00000-0x1FFFF
   (+) 工作电压   ： 3V
   (+) 工作温度   ：-40~85℃
   (+) 设备ID     :  0x06818820
   (+) 数据保存时间（环境温度85℃） ： 20年
   (+) 通信方式  ： SPI 模式0或模式3
   (+) 时钟频率  ： 正常读写 最高40MHZ  快读方式最高 104MHZ
   (+) 写保护    ： 支持 1/4 1/2 1/1 0 内存保护
   (+) 工作方式  ： 其内部有两个模块组成 SRAM （掉电丢失）和 量子陷阱（掉电保存）
                   正常读写在SRAM中，掉电的过程中，外部连接电容剩余的电量，将SRAM
                   中的数据保存到量子陷阱单元，实现掉电数据保护，上电之后，会自动
                   将量子陷阱单元的数据搬运到SRAM中，实现上电加载，同时也可以利用
                   软件命令实现数据保存。  
  @endverbatim
  ******************************************************************************
  * @attention
   <pre>
         (+)  数据进行突发写操作时，当突发写到达一个受保护的块地址时，它将地址增
              量继续到受保护空间，但是不向受保护内存写入任何数据。如果地址滚转将
              突发写操作带到未受保护的空间，然后恢复写入。
         (+)  数据进行突发写操作或者读操作时，如果地址大于0x1FFFF，地址将自动
              在0x00000处依次增加写入或者读取；
         (+)  在进行写内存，写寄存器，或者保存，加载、自动保存使能，自动保存不使
               能操作之后，写使能将被禁止，同时状态寄存中WEN bit为0，下一次进行写
               操作时必须将开启写使能。
         (+)  芯片内部有8个字节的序列号空间，但只能被写一次（保存到量子陷阱之后），
              写一次之后，状态寄存器中SNL bit 为1，并且永远不能擦除该标志位，序列号
              永远不能再被更改。
         (+)  提高设备驱动读写速率有两个方法 ：提高SPI通信速率
                                               增大NVSRAM_BUF_SIZE缓存区
    </pre>
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "Drv_Nvsram.h"
#include "Drv_Spi.h"
#include <stdio.h>
#include <string.h>

/** @addtogroup 设备驱动
  * @{
  */
/** @defgroup NVSRAM驱动
  * @{
  */

/* Private types -------------------------------------------------------------*/
/* Private function macro ---------------------------------------------------------*/

/** @addtogroup NVSRAM静态函数宏
  * @{
  */
/**
  * @brief   NVSRAM 毫秒延时.
  * @param[in]  MS: 延时 N ms
  */
#define Nvsram_Delay_Ms(NMS)                               HAL_Delay(NMS)
    
/**
  * @brief   NVSRAM 获取系统时间戳.
  * @retval  系统时间戳 毫秒
  */
#define Nvsram_Get_tick()                                  HAL_GetTick()                                                                  
    
/**
  * @brief  NVSRAM SPI通信接口
  * @param[in]   txbuf: 发送缓存区地址
  * @param[out]  rxbuf: 接收缓存区地址
  * @param[in]   len: 发送字节数
  * @retval      RET_OK：成功 ; 
    @retval      RET_ERR:失败 
  */
#define Nvsram_Spi_TR(txbuf,rxbuf,len)                    SPI_TransmitReceive(SPI_CH1, txbuf, rxbuf, len)

/**
  * @}
  */ 
/** @addtogroup NVSRAM设备参数
  * @{
  */
#define NVSRAM_STATUS_RDSR                                        0x05   /*!<  读状态寄存器 */ 
#define NVSRAM_STATUS_FRDSR                                       0x0A   /*!<  快读状态寄存器 */ 
#define NVSRAM_STATUS_WRSR                                        0x01   /*!<  写转态寄存器*/ 
#define NVSRAM_STATUS_WREN                                        0x06   /*!<  写使能*/ 
#define NVSRAM_STATUS_WRDI                                        0x04   /*!<  复位写使能 */ 

#define NVSRAM_SRAM_READ                                          0x03  /*!<  读内存 */ 
#define NVSRAM_SRAM_FREAD                                         0x0B  /*!<  快读内存*/ 
#define NVSRAM_SRAM_WRITE                                         0x02  /*!<  写内存 */ 

#define NVSRAM_SPEC_NV_STORE                                      0x3C  /*!<  内存数据存储 */ 
#define NVSRAM_SPEC_NV_RECALL                                     0x60  /*!<  内存数据加载 */ 
#define NVSRAM_SPEC_NV_ASENB                                      0x59  /*!<  自动储存使能 */ 
#define NVSRAM_SPEC_NV_ASDISB                                     0x19  /*!<  自动储存禁止 */ 

#define NVSRAM_SPEC_SLEEP                                         0xB9  /*!<  睡眠模式使能 */ 
#define NVSRAM_SPEC_WRSN                                          0xC2  /*!<  写序列号 */ 
#define NVSRAM_SPEC_RDSN                                          0xC3  /*!<  读序列号 */ 
#define NVSRAM_SPEC_FRDSN                                         0xC9  /*!<  快读序列号 */ 
#define NVSRAM_SPEC_RDID                                          0x9F  /*!<  读设备ID */ 
#define NVSRAM_SPEC_FRDID                                         0x99  /*!<  快读设备ID */ 

#define NVSRAM_STATUS_RDY                                         0x01 /*!<  状态寄存器 就绪位*/ 
#define NVSRAM_STATUS_WEN                                         0x02 /*!<  状态寄存器 写使能位*/ 
#define NVSRAM_STATUS_BP0                                         0x04 /*!<  状态寄存器 块保护 0位*/  
#define NVSRAM_STATUS_BP1                                         0x08 /*!<  状态寄存器 块保护 1位*/ 
#define NVSRAM_STATUS_SNL                                         0x40 /*!<  状态寄存器 序列号锁存位*/ 
#define NVSRAM_STATUS_WPEN                                        0x80 /*!<  状态寄存器 写保护引脚使能位*/ 

#define NVSRAM_DEV_ID                                             0x06818820 /*!<  设备ID*/ 
#define NVSRAM_MAX_ADDR                                           0x1FFFF    /*!<  设备最大地址*/
#define NVSRAM_BUF_SIZE                                           256        /*!<  设备一次发送最大字节数*/

/**
  * @}
  */
/* Private variables ---------------------------------------------------------*/
static Nvsram_Struct_t _nvsram = {0};
static uint8_t _nvsram_txbuf[NVSRAM_BUF_SIZE + 4] = {0}; /*!<  前4个字节 1个字节命令码+3个字节地址 */ 
static uint8_t _nvsram_rxbuf[NVSRAM_BUF_SIZE + 4] = {0}; /*!<  前4个字节 1个字节命令码+3个字节地址 */ 

/* Private function ---------------------------------------------------------*/
/** @addtogroup NVSRAM静态函数
  * @{
  */

static int8_t  nvsram_check_id(void);
static int8_t  nvsram_get_ready(void);
static int8_t  nvsram_get_snw(void);

/**
  * @brief   NVSRAM 检测设备ID
  * @retval  0:成功 
    @retval  -1:失败 
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
  * @brief   NVSRAM 获取就绪状态（上电等待）
  * @retval  0:成功 
    @retval -1:失败 
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
  * @brief   NVSRAM 序列号是否写入(上电检测)
  * @retval  0:成功 
    @retval -1:失败 
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
  * @brief   NVSRAM 设置写使能
  * @note   每次写操作之前都要调用 
  * @note   状态寄存器两个方面判断 : 设备就绪位为0  写使能位为1
  * @retval  0:成功 
    @retval -1:失败 
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


/** @addtogroup NVSRAM外部函数
  * @{
  */
/**
  * @brief   NVSRAM 设置写保护.
  * @param[in]  Level: 设置写保护等级 
  *          @arg @ref NVSRAM_BPL0  0   内存不保护 
  *          @arg @ref NVSRAM_BPL1  1/4 内存保护 0x18000–0x1FFFF
  *          @arg @ref NVSRAM_BPL2  1/2 内存保护 0x10000–0x1FFFF
  *          @arg @ref NVSRAM_BPL3  ALL 内存保护 0x00000–0x1FFFF
  *  @retval NVSRAM_SUCCEED:成功  
  *  @retval NVSRAM_PRAM_ERRO:传入参数错误 
  *  @retval NVSRAM_Busy:设备忙 
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
  * @brief   NVSRAM 获取写保护块.
  * @retval  NVSRAM_BPL0  0   内存不保护 
  * @retval  NVSRAM_BPL1  1/4 内存保护 0x18000–0x1FFFF
  * @retval  NVSRAM_BPL2  1/2 内存保护 0x10000–0x1FFFF
  * @retval  NVSRAM_BPL3  ALL 内存保护 0x00000–0x1FFFF
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
  * @brief   NVSRAM 写内存函数
  * @note    1) 数据进行突发写操作时，当突发写到达一个受保护的块地址时，它将地址增
  * @note       量继续到受保护空间，但是不向受保护内存写入任何数据。如果地址增加到
  * @note       突发写操作带到未受保护的空间，然后恢复写入
  * @note    2) 数据进行突发写操作时，如果地址大于0x1FFFF，地址将自动在0x00000处依次增加写入  
  * @note    3) 缓存区大小为 256Byte, SPI通信速率为 12.5MHZ 写速率为 262.83KByte/S     
  * @param[in]  nvsram_addr:NVSRAM内存地址 0x00000–0x1FFFF
  * @param[in]  w_buf:写缓存区地址
  * @param[in]  len:写入字节长度
  * @retval  写入字节数量
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
  * @brief   NVSRAM 读内存函数
  * @note    数据进行读操作时,如果地址大于0x1FFFF,地址将自动在0x00000中依次增加读取
  * @note    缓存区大小为 256Byte, SPI通信速率为 12.5MHZ 读速率为 262.83KByte/S   
  * @param[in]  nvsram_addr: NVSRAM内存地址 0x00000–0x1FFFF
  * @param[out]  r_buf:读缓存区地址
  * @param[in]   len:读字节长度
  * @retval  读出字节数量
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
  * @brief   NVSRAM 软件存储
  * @note    将内部SDRAM数据存储到量子陷阱中,由于存储需要一定的时间,如果设备可以掉电保存,
  * @note    不建议使用该函数
  * @retval  NVSRAM_SUCCEED:成功  
  * @retval  NVSRAM_PRAM_ERRO:传入参数错误 
  * @retval  NVSRAM_Busy:设备忙 
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
  * @brief   NVSRAM 软件加载数据
  * @note    将量子陷阱中数据加载到内部SDRAM数据中,由于存储需要一定的时间,如果设备可以上电加载,
  * @note    不建议使用该函数
  * @retval  NVSRAM_SUCCEED:成功  
  * @retval  NVSRAM_PRAM_ERRO:传入参数错误 
  * @retval  NVSRAM_Busy:设备忙 
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
  * @brief   NVSRAM 设置序列号
  * @note   内部有8个字节的序列号空间，但只能被写一次(保存到量子陷阱之后),
  * @note   写一次之后,状态寄存器中SNL bit 为1,并且永远不能擦除该标志位,序列号
  * @note   永远不能再被更改,由于需要存储功能,将会耗时6ms左右,不建议将此函数放入线程中使用
  * @param[in]  sn_buff: 序列号缓存区保存地址 该缓存区要大于等于8个字节 
  * @retval  NVSRAM_SUCCEED:成功  
  * @retval  NVSRAM_PRAM_ERRO:传入参数错误 
  * @retval  NVSRAM_Busy:设备忙 
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
  * @brief   NVSRAM 获取序列号
  * @param[out]  sn_buff: 序列号缓存区保存地址 该缓存区要大于等于8个字节 
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
  * @brief   NVSRAM 设备初始化
  * @note    硬件初始化 等待设备就绪,加载ID号,加载序列号写入标志
  * @retval  NVSRAM_SUCCEED:成功  
  * @retval  NVSRAM_PRAM_ERRO:传入参数错误 
  * @retval  NVSRAM_Busy:设备忙 
 */
uint8_t Nvsram_Init(void)
{
   uint8_t ret = NVSRAM_SUCCEED;
   SPI_Init(SPI_CH1, SPI_BAUDRATE_12_5M);
   //等待数据加载
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
  * @brief   NVSRAM 获取设备状态
  * @param[out]  nvsram_st 设备状态结构体地址
 */
void Nvsram_Get_Struct(Nvsram_Struct_t *nvsram_st)
{
  memcpy(&_nvsram, nvsram_st, sizeof(Nvsram_Struct_t));
}

#ifdef Drv_Nvsram_TSET
uint8_t test_tx[NVSRAM_BUF_SIZE] = {0};
uint8_t test_rx[NVSRAM_BUF_SIZE] = {0};
/**
  * @brief   NVSRAM 驱动测试
  * @note    1)设备初始化测试,函数运行时间,设备ID,设备序列号是否写入
  * @note    2)设备软件保存指令检测 测试运行时间
  * @note    3)设备软件数据加载指令检测 测试运行时间 
  * @note    4)设备写函数测试（一次写一个缓存区）
  * @note    5)设备读函数测试（一次读一个缓存区）
  * @note    6)设备写函数测试（128K内存）测试写入运行时间
  * @note    7)设备读函数测试（128K内存）测试写入读出数据的正确性  
  * @note    8)设备读函数测试（128K内存）测试读出运行时间 
  * @note    9)设备掉电数据保存测试 调试状态将内存中写入数据 0x55
  * @note      断电重启,设备初始化之后,读出数据，判断数据全为0x55      
  * @note    10)设备块保护测试  NVSRAM_BPL0 （128K内存）写入0x11  读出数据判断是否一致
  * @note                       NVSRAM_BPL1 （128K内存）写入0x22  读出数据判断是否一致
  * @note                       NVSRAM_BPL2 （128K内存）写入0x33  读出数据判断是否一致
  * @note                       NVSRAM_BPL3 （128K内存）写入0x44  读出数据判断是否一致 
  * @note                       解除块保护  （128K内存）写入0x55  读出数据判断是否一致 
 */
void Nvsram_Test(void)
{
   uint32_t start = 0,end = 0,txlen = 0,rxlen = 0,bp_start = 0, bp_len= 0;
   uint8_t ret = 0;
   uint32_t idx = 0,j = 0;
   
   for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
     test_tx[idx] = (uint8_t) idx;
   }
   
   printf("\r\n1)NVSRAM 初始化函数 测试\n");
   start = Nvsram_Get_tick();
   ret = Nvsram_Init();
   end = Nvsram_Get_tick();
   if(ret == NVSRAM_SUCCEED){
     printf("\rNVSRAM 初始化成功, 函数耗时 %d MS\n",end - start); 
     printf("\rNVSRAM 设备ID 0x%x\n",_nvsram.id);
     if(_nvsram.snw == 1){
       printf("\rNVSRAM 设备序列号已写入\n");
     }
     else{
       printf("\rNVSRAM 设备序列号未写入\n"); 
     }
   }
   
   Nvsram_Burst_Read_Buff(0x00000, test_rx, 1);
//   if(test_rx[0] == 0x55){
//     printf("\r\n9)NVSRAM 掉电保存读函数(内存128K 地址 0X00000~0X1FFFF) 验证数据是否正确测试\n");
//     rxlen = 0;
//     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
//       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
//       for(j=0; j<NVSRAM_BUF_SIZE; j++){
//         if(test_rx[j] != 0x55){
//           printf("\rNVSRAM 读出错误 地址 0x%x\n",idx * NVSRAM_BUF_SIZE  + j);
//         }   
//       }
//     }
//     printf("\rNVSRAM 读出字节数%d\n",rxlen);
//   }
   
   printf("\r\n2)NVSRAM 数据保存函数 测试\n");
   start = Nvsram_Get_tick();
   ret = Nvsram_Soft_Store();
   end = Nvsram_Get_tick();
   if(ret == NVSRAM_SUCCEED){
     printf("\rNVSRAM 数据保存成功, 函数耗时 %d MS\n",end - start); 
   }
   printf("\r\n3)NVSRAM 数据加载函数 测试\n");
   start = Nvsram_Get_tick();
   ret = Nvsram_Soft_Recall();
   end = Nvsram_Get_tick();
   if(ret == NVSRAM_SUCCEED){
     printf("\rNVSRAM 数据加载成功, 函数耗时 %d MS\n",end - start); 
   }   
   
   printf("\r\n4)NVSRAM 写函数(一次写入%d个字节) 测试\n", NVSRAM_BUF_SIZE);
   start = Nvsram_Get_tick();
   txlen = Nvsram_Burst_Write_Buff(0x00000, test_tx, NVSRAM_BUF_SIZE);
   end = Nvsram_Get_tick();
   printf("\rNVSRAM 写入字节数%d, 函数耗时 %d MS\n",txlen, end - start);
   
   printf("\r\n5)NVSRAM 读函数(一次读出%d个字节) 测试\n", NVSRAM_BUF_SIZE);
   start = Nvsram_Get_tick();
   rxlen = Nvsram_Burst_Read_Buff(0x00000, test_rx, NVSRAM_BUF_SIZE);
   end = Nvsram_Get_tick();
   printf("\rNVSRAM 读出字节数%d, 函数耗时 %d MS\n", rxlen, end - start);
   for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
     if(test_rx[idx] != test_tx[idx]){
       printf("\rNVSRAM 读出错误 地址 %d\n",idx);
     }   
   }
   
   printf("\r\n6)NVSRAM 写函数(内存128K 地址 0X00000~0X1FFFF) 测试\n");
   txlen = 0;
   start = Nvsram_Get_tick();
   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
      txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
   }
   end = Nvsram_Get_tick();
   printf("\rNVSRAM 写入字节数%d, 耗时 %d MS, %f KByte/s\n",txlen, end - start, (NVSRAM_MAX_ADDR+1)/1024*1000/((end-start) * 1.0));
 
   printf("\r\n7)NVSRAM 读函数(内存128K 地址 0X00000~0X1FFFF) 验证数据是否正确测试\n");
   rxlen = 0;
   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
     rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
     for(j=0; j<NVSRAM_BUF_SIZE; j++){
       if(test_rx[j] != test_tx[j]){
         printf("\rNVSRAM 读出错误 地址 0x%x\n",idx * NVSRAM_BUF_SIZE  + j);
       }   
     }
   }
   printf("\rNVSRAM 读出字节数%d\n",rxlen);
 
   printf("\r\n8)NVSRAM 读函数(内存128K 地址 0X00000~0X1FFFF) 速率测试\n");
   rxlen = 0;
   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
     rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
   }
   printf("\rNVSRAM 读出字节数%d, 耗时 %d MS, %f KByte/s\n",rxlen, end - start, (NVSRAM_MAX_ADDR+1)/1024*1000/((end-start) * 1.0));
   
//   printf("\r\n9)NVSRAM 掉电保存(内存128K 地址 0X00000~0X1FFFF) 测试（写入）\n");
//   for(idx=0; idx<NVSRAM_BUF_SIZE; idx++){
//     test_tx[idx] = (uint8_t) 0x55;
//   }
//   txlen = 0;
//   for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
//      txlen += Nvsram_Burst_Write_Buff(idx*NVSRAM_BUF_SIZE, test_tx, NVSRAM_BUF_SIZE);
//   }
//   printf("\rNVSRAM 写入字节数%d\n",txlen);
   
   printf("\r\n10-1)NVSRAM 块保护 NVSRAM_BPL0 (内存不保护) 测试\n");
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
     printf("\rNVSRAM 写入字节数%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
              bp_start = idx * NVSRAM_BUF_SIZE + j;
              printf("\rNVSRAM 内存保护起始地址 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM 读出字节数%d, 内存保护大小%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL0){
       printf("\rNVSRAM 块保护 NVSRAM_BPL0 设置成功");
     }
   }
   
   printf("\r\n10-2)NVSRAM 块保护 NVSRAM_BPL1 (0x18000–0x1FFFF) 测试\n");
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
     printf("\rNVSRAM 写入字节数%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
             bp_start = idx * NVSRAM_BUF_SIZE + j;
             printf("\rNVSRAM 内存保护起始地址 0x%x\n",bp_start);
           }
           bp_len++;
         }  
         
       }
     }
     printf("\rNVSRAM 读出字节数%d, 内存保护大小%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL1){
       printf("\rNVSRAM 块保护 NVSRAM_BPL1 设置成功\n");
     }
   }
   
   printf("\r\n10-3)NVSRAM 块保护 NVSRAM_BPL2 (0x10000–0x1FFFF) 测试\n");
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
     printf("\rNVSRAM 写入字节数%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
             bp_start = idx * NVSRAM_BUF_SIZE + j;
             printf("\rNVSRAM 内存保护起始地址 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM 读出字节数%d, 内存保护大小%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL2){
       printf("\rNVSRAM 块保护 NVSRAM_BPL2 设置成功\n");
     }
   }  
   
   printf("\r\n10-4)NVSRAM 块保护 NVSRAM_BPL3 (0x00000–0x1FFFF) 测试\n");
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
     printf("\rNVSRAM 写入字节数%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0 && bp_len == 0){
             bp_start = idx * NVSRAM_BUF_SIZE + j;
             printf("\rNVSRAM 内存保护起始地址 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM 读出字节数%d, 内存保护大小%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL3){
       printf("\rNVSRAM 块保护 NVSRAM_BPL3 设置成功\n");
     }
   } 
   
   printf("\r\n10-5)NVSRAM 块保护 解除(内存不保护) 测试\n");
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
     printf("\rNVSRAM 写入字节数%d\n",txlen);
     
     rxlen = 0;
     for(idx = 0; idx < (NVSRAM_MAX_ADDR+1)/NVSRAM_BUF_SIZE; idx++){
       rxlen += Nvsram_Burst_Read_Buff(idx*NVSRAM_BUF_SIZE, test_rx, NVSRAM_BUF_SIZE);
       for(j=0; j<NVSRAM_BUF_SIZE; j++){
         if(test_rx[j] != test_tx[j]){
           if(bp_start == 0){
              bp_start = idx * NVSRAM_BUF_SIZE + j;
              printf("\rNVSRAM 内存保护起始地址 0x%x\n",bp_start);
           }
           bp_len++;
         }   
       }
     }
     printf("\rNVSRAM 读出字节数%d, 内存保护大小%d kbyte\n",rxlen,bp_len/1024);
     
     ret = Nvsram_Get_Bp();
     if(ret == NVSRAM_BPL0){
       printf("\rNVSRAM 块保护 解除成功\n");
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










































