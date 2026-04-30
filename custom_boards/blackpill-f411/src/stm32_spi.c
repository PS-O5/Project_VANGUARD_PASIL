#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>
#include <nuttx/spi/spi.h>
#include "stm32_gpio.h"
#include "stm32_spi.h"
#include <arch/board/board.h> /* Pulls in GPIO_NRF24L01_CSN */

/* =======================================================
 * 1. The Missing SPI Initialization Hook
 * Called by stm32_boot.c to set up physical GPIOs
 * ======================================================= */
void stm32_spidev_initialize(void)
{
#ifdef CONFIG_STM32_SPI2
  /* Pre-configure the Chip Select pin as an output, initialized high */
  stm32_configgpio(GPIO_NRF24L01_CSN);
#endif
}

/* =======================================================
 * 2. The Missing SPI2 Select and Status Hooks
 * Required by the lower-half STM32 SPI driver
 * ======================================================= */
#ifdef CONFIG_STM32_SPI2
void stm32_spi2select(struct spi_dev_s *dev, uint32_t devid, bool selected)
{
  /* SPIDEV_WIRELESS(0) is the standard identifier for the first RF device */
  if (devid == SPIDEV_WIRELESS(0))
    {
      /* NRF24L01 CSN is active low (!selected pulls it to 0V) */
      stm32_gpiowrite(GPIO_NRF24L01_CSN, !selected);
    }
}

uint8_t stm32_spi2status(struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}
#endif
