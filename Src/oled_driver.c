#include "oled_driver.h"
#include "stdlib.h"
#include "spi.h"
#include "u8g2.h"

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
  case U8X8_MSG_BYTE_INIT:
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