#ifndef __OLED_DRIVER_H__
#define __OLED_DRIVER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "u8g2.h"

	void u8g2_WaitDMA(u8g2_t *u8g2);
	void u8g2_SendBufferDMA(u8g2_t *u8g2);
  void GPIO_WriteBit(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint8_t arg_int);
  uint8_t u8x8_byte_4wire_hw_spi(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
  uint8_t u8x8_stm32_gpio_and_delay(U8X8_UNUSED u8x8_t *u8x8,
                                    U8X8_UNUSED uint8_t msg, U8X8_UNUSED uint8_t arg_int,
                                    U8X8_UNUSED void *arg_ptr);
  void draw(u8g2_t *u8g2);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_DRIVER_H__ */
