#include "u_flash.h"
#include "stm32l0xx.h"
#include "stdio.h"
#include "string.h"

#define SN_CODE                  "YCBJQ0112345"    //
//stm32L0xx    PAGE  128BYTE     64*1024/128=512ҳ
#define PAGE_SIZE                128
#define PAGE_NUM                 512
#define ADDR_FLASH_PAGE_505      0X08000000 + 128*504   //
#define ADDR_FLASH_PAGE_506      0X08000000 + 128*505   //
#define ADDR_FLASH_PAGE_507      0X08000000 + 128*506   //
#define ADDR_FLASH_PAGE_508      0X08000000 + 128*507   //
#define ADDR_FLASH_PAGE_509      0X08000000 + 128*508   //
#define ADDR_FLASH_PAGE_510      0X08000000 + 128*509   //
#define ADDR_FLASH_PAGE_511      0X08000000 + 128*510   //
#define ADDR_FLASH_PAGE_512      0X08000000 + 128*511   //���һҳ

uint8_t sn_code[12];
 //HAL_FLASH_Unlock(void);
 //HAL_FLASH_Lock(void);
// HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint32_t Data)��
 //HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint32_t Data)��
void Flash_Write_Num_Word(uint32_t WriteAddr, uint32_t * pBuffer, uint32_t NumToWrite )
{
	uint32_t PAGEError = 0;
	FLASH_EraseInitTypeDef EraseInitStruct;
	if(WriteAddr<FLASH_BASE||(WriteAddr>=(FLASH_BASE+512*PAGE_SIZE)))
	{	
		printf("�Ƿ���ַ\r\n");
	  return;//�Ƿ���ַ
	}
		HAL_FLASH_Unlock(); 
	 //  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP  | FLASH_FLAG_WRPERR);   //����֮ǰ�����־����һ��Ҫ�������
	
	   EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;	// ˢ����ʽ
	  EraseInitStruct.PageAddress = WriteAddr;	// ��ʼ��ַ
     EraseInitStruct.NbPages = 1;
	  if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
      {
        // ���ˢ������
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
  * ��������: ��ȡָ����ַ�İ���(16λ����)
  * �������: faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
  * �� �� ֵ: ����ֵ:��Ӧ����.
  * ˵    ������
  */
uint32_t STMFLASH_ReadWord ( uint32_t faddr )
{
	return *(__IO uint32_t*)faddr; 
}
/**
  * ��������: ��ָ����ַ��ʼ����ָ�����ȵ�����
  * �������: ReadAddr:��ʼ��ַ
  *           pBuffer:����ָ��
  *           NumToRead:����(16λ)��
  * �� �� ֵ: ��
  * ˵    ������
  */
void Flash_Read_Word( uint32_t ReadAddr, uint32_t *pBuffer, uint32_t NumToRead )   	
{
	uint16_t i;
	
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);//��ȡ4���ֽ�.
		ReadAddr+=4;//ƫ��4���ֽ�.	
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
