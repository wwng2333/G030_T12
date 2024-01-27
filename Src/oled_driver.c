#include "oled_driver.h"
#include "stdlib.h"
#include "spi.h"
#include "u8g2.h"

/* USER CODE BEGIN PD */
#define SWAP8(a) ((((a)&0x80) >> 7) | (((a)&0x40) >> 5) | (((a)&0x20) >> 3) | (((a)&0x10) >> 1) | (((a)&0x08) << 1) | (((a)&0x04) << 3) | (((a)&0x02) << 5) | (((a)&0x01) << 7))
/* USER CODE END PD */

/* USER CODE BEGIN PV */
//static uint8_t dma_memory[1024];
/* USER CODE END PV */

//void u8g2_WaitDMA(u8g2_t *u8g2)
//{
//	if (LL_DMA_IsEnabledChannel(DMA1, LL_DMA_CHANNEL_1) == SET)
//	{
//		while (LL_DMA_IsActiveFlag_TC3(DMA1) == RESET)
//		{
//			__NOP();
//		}
//	}
//	while(LL_SPI_IsActiveFlag_BSY(SPI1) == SET)
//	{
//		__NOP();
//	}
//	u8x8_byte_EndTransfer(&u8g2->u8x8);
//	LL_SPI_DisableDMAReq_TX(SPI1);
//	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
//	LL_DMA_ClearFlag_TC3(DMA1);
//}

//void u8g2_SendBufferDMA(u8g2_t *u8g2)
//{
//	uint16_t i, j;
//	uint8_t *dest;
//	uint8_t *src;
//	
//	u8g2_WaitDMA(u8g2);
//	
//	dest = dma_memory;
//	
//	for (i = 0; i < 128; i++)
//	{
//		src = u8g2->tile_buf_ptr + i;
//		for (j = 0; j < 7; j++)
//		{
//			*dest = SWAP8(*(src + (j * 128)));
//			dest += 1;
//		}
//	}
//	
//	u8x8_cad_StartTransfer(&u8g2->u8x8);
//	
//	u8x8_cad_SendCmd(&u8g2->u8x8, SWAP8(0x0F0));
//	u8x8_cad_SendArg(&u8g2->u8x8, SWAP8(0));
//	u8x8_cad_SendArg(&u8g2->u8x8, SWAP8(0 + 4));
//	u8x8_cad_SendArg(&u8g2->u8x8, SWAP8(0x037));
//	
//	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, sizeof(dma_memory));
//	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&dma_memory, LL_SPI_DMA_GetRegAddr(SPI1), LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
//	
//	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_3);
//	LL_SPI_EnableDMAReq_TX(SPI1);
//}

void GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint8_t arg_int)
{
	if(arg_int)
	{
		LL_GPIO_SetOutputPin(GPIOx, PinMask);
	}
	else
	{
		LL_GPIO_ResetOutputPin(GPIOx, PinMask);
	}
}

uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
  uint8_t *data = NULL;
  switch (msg)
  {
  case U8X8_MSG_BYTE_INIT:
		LL_SPI_Enable(SPI1);
    break;
  case U8X8_MSG_BYTE_SEND:
    data = (uint8_t *)arg_ptr;
    do
    {
      while (LL_SPI_IsActiveFlag_TXE(SPI1) == RESET)
        ;
      LL_SPI_TransmitData8(SPI1, *(data++));
      while (LL_SPI_IsActiveFlag_BSY(SPI1) == SET)
      {
      }
      arg_int--;
    } while (arg_int > 0);
    break;
  case U8X8_MSG_BYTE_SET_DC:
		GPIO_WriteBit(GPIOA, LL_GPIO_PIN_6, arg_int);
    break;
  case U8X8_MSG_BYTE_START_TRANSFER:
    break;
  case U8X8_MSG_BYTE_END_TRANSFER:
    break;
  default:
    return 0;
  }
  return 1;
}

uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
                                  U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
                                  U8X8_UNUSED void *arg_ptr)
{
  switch (msg)
  {
  case U8X8_MSG_GPIO_AND_DELAY_INIT:
    LL_mDelay(1);
    break;
  case U8X8_MSG_DELAY_MILLI:
    LL_mDelay(arg_int);
    break;
  case U8X8_MSG_GPIO_CS:
    GPIO_WriteBit(GPIOA, LL_GPIO_PIN_12, arg_int);
  case U8X8_MSG_GPIO_DC:
    GPIO_WriteBit(GPIOA, LL_GPIO_PIN_6, arg_int);
    break;
  case U8X8_MSG_GPIO_RESET:
    GPIO_WriteBit(GPIOA, LL_GPIO_PIN_4, arg_int);
    break;
  }
  return 1;
}

void draw(u8g2_t *u8g2)
{
    u8g2_SetFontMode(u8g2, 1);  // Transparent
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 0, 20, "U");
    
    u8g2_SetFontDirection(u8g2, 1);
    u8g2_SetFont(u8g2, u8g2_font_inb30_mn);
    u8g2_DrawStr(u8g2, 21,8,"8");
        
    u8g2_SetFontDirection(u8g2, 0);
    u8g2_SetFont(u8g2, u8g2_font_inb24_mf);
    u8g2_DrawStr(u8g2, 51,30,"g");
    u8g2_DrawStr(u8g2, 67,30,"\xb2");
    
    u8g2_DrawHLine(u8g2, 2, 35, 47);
    u8g2_DrawHLine(u8g2, 3, 36, 47);
    u8g2_DrawVLine(u8g2, 45, 32, 12);
    u8g2_DrawVLine(u8g2, 46, 33, 12);
  
    u8g2_SetFont(u8g2, u8g2_font_4x6_tr);
    u8g2_DrawStr(u8g2, 1,54,"github.com/olikraus/u8g2");
}