/****************************************************************************
 * boards/arm/stm32/nucleo-f411re/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <syslog.h>
#include <errno.h>

#include <sys/mount.h>

#include <nuttx/arch.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <stm32.h>
#include <stm32_uart.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/bmi160.h>
#include "stm32_i2c.h"

#include <arch/board/board.h>

#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include "blackpill-f411.h"

#include <nuttx/board.h>

#ifdef CONFIG_SENSORS_QENCODER
#include "board_qencoder.h"
#endif

#undef HAVE_LEDS
#if !defined(CONFIG_ARCH_LEDS) && defined(CONFIG_USERLED_LOWER)
#  define HAVE_LEDS 1
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

  /* Seize control of the PROCFS mount */
  syslog(LOG_INFO, "[BOOT] Hard-mounting PROCFS to /proc...\n");
  ret = mount(NULL, "/proc", "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Hardware mount of PROCFS failed.\n");
    }

  syslog(LOG_INFO, "[BOOT] Project VANGUARD PASIL: Booted!");

#ifdef HAVE_LEDS
  /* Register the LED driver */

  ret = userled_lower_initialize("/dev/userleds");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  /* Register the BUTTON driver */

  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

  /* Configure SPI-based devices */

#ifdef CONFIG_STM32_SPI1
  /* Get the SPI port */

  struct spi_dev_s *spi;

  spi = stm32_spibus_initialize(1);
  if (!spi)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SPI port 1\n");
      return -ENODEV;
    }

#if defined(CONFIG_LCD_SSD1306_SPI) && !defined(CONFIG_VIDEO_FB)
  board_lcd_initialize();
#endif

#ifdef CONFIG_VIDEO_FB
  ret = fb_register(0, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: fb_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_CAN_MCP2515
#ifdef CONFIG_STM32_SPI1
  stm32_configgpio(GPIO_MCP2515_CS);    /* MEMS chip select */
#endif

  /* Configure and initialize the MCP2515 CAN device */

  ret = stm32_mcp2515initialize("/dev/can0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_mcp2515initialize() failed: %d\n", ret);
    }
#endif
#endif

#ifdef HAVE_MMCSD
  /* First, get an instance of the SDIO interface */

  g_sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  /* Then let's guess and say that there is a card in the slot. There is no
   * card detect GPIO.
   */

  sdio_mediachange(g_sdio, true);

  syslog(LOG_INFO, "[boot] Initialized SDIO\n");
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0, CONFIG_NUCLEO_F411RE_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_INPUT_AJOYSTICK
  /* Initialize and register the joystick driver */

  ret = board_ajoy_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the joystick driver: %d\n",
             ret);
      return ret;
    }
#endif


#ifdef CONFIG_STM32_I2C1
  struct i2c_master_s *i2c;

  syslog(LOG_INFO, "Bringing up I2C1...\n");
  i2c = stm32_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize I2C1\n");
    }
  else
    {
      syslog(LOG_INFO, "I2C1 Initialized. Probing BMI160...\n");
      
      /* The legacy BMI160 driver manages the I2C address (0x68/0x69) internally */
      ret = bmi160_register("/dev/imu0", i2c); 
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register BMI160: %d\n", ret);
        }
    }
#endif


  return ret;
}
