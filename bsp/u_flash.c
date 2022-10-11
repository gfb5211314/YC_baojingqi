#include "u_flash.h"
#include "stm32l0xx.h"
#include "stdio.h"
#include "string.h"

#define SN_CODE                  "YCBJQ0112345"    //
//stm32L0xx    PAGE  128BYTE     64*1024/128=512页
#define PAGE_SIZE                128
#define PAGE_NUM                 512
#define ADDR_FLASH_PAGE_505      0X08000000 + 128*504   //
#define ADDR_FLASH_PAGE_506      0X08000000 + 128*505   //
#define ADDR_FLASH_PAGE_507      0X08000000 + 128*506   //
#define ADDR_FLASH_PAGE_508      0X08000000 + 128*507   //
#define ADDR_FLASH_PAGE_509      0X08000000 + 128*508   //
#define ADDR_FLASH_PAGE_510      0X08000000 + 128*509   //
#define ADDR_FLASH_PAGE_511      0X08000000 + 128*510   //
#define ADDR_FLASH_PAGE_512      0X08000000 + 128*511   //最后一页

uint8_t sn_code[12];
 //HAL_FLASH_Unlock(void);
 //HAL_FLASH_Lock(void);
// HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint32_t Data)；
 //HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint32_t Data)；
void Flash_Write_Num_Word(uint32_t WriteAddr, uint32_t * pBuffer, uint32_t NumToWrite )
{
	uint32_t PAGEError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	if(WriteAddr<FLASH_BASE||(WriteAddr>=(FLASH_BASE+512*PAGE_SIZE)))
	{	
		printf("非法地址\r\n");
	  return;//非法地址
	}
		HAL_FLASH_Unlock(); 
	 //  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP  | FLASH_FLAG_WRPERR);   //擦除之前先清标志，（一定要加这个）
	
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;	// 刷除方式
	  EraseInitStruct.PageAddress = WriteAddr;	// 起始地址
     EraseInitStruct.NbPages = 1;
	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
      {
        // 如果刷除错误
        printf("\r\n FLASH Erase Fail\r\n");
        printf("Fail Code:%d\r\n",HAL_FLASH_GetError());
        printf("Fail Page:%d\r\n",PAGEError);
      }
			for(uint32_t i=0;i<NumToWrite;i++)
			{
			if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, WriteAddr+i*4, *(pBuffer+i)) == HAL_OK){
         printf("write data 0x%x OK\n", *(pBuffer+i));
      }
      else{
        printf("failed!!!\n");
      }
		  }
      HAL_FLASH_Lock();
}
/**
  * 函数功能: 读取指定地址的半字(16位数据)
  * 输入参数: faddr:读地址(此地址必须为2的倍数!!)
  * 返 回 值: 返回值:对应数据.
  * 说    明：无
  */
uint32_t STMFLASH_ReadWord ( uint32_t faddr )
{
	return *(__IO uint32_t*)faddr; 
}
/**
  * 函数功能: 从指定地址开始读出指定长度的数据
  * 输入参数: ReadAddr:起始地址
  *           pBuffer:数据指针
  *           NumToRead:半字(16位)数
  * 返 回 值: 无
  * 说    明：无
  */
void Flash_Read_Word( uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead )   	
{
	uint16_t i;
	
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//读取4个字节.
		ReadAddr+=4;//偏移4个字节.	
		    printf("read data 0x%x OK\n", *(pBuffer+i));
	}
}
uint32_t ab[2]={0x12345678,0x88997766};
uint32_t ac[2];
void test_flash()
{
	 memcpy(sn_code, SN_CODE, 12);
	Flash_Write_Num_Word(ADDR_FLASH_PAGE_512,(uint32_t *) sn_code, 3 );
	
	Flash_Read_Word( ADDR_FLASH_PAGE_512, ac,3 ) ;
	  printf("read data 0x%x OK\n", *ac);
	 printf("read data 0x%x OK\n", *(ac+1));
}
