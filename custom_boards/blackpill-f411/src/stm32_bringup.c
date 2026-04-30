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

#include <nuttx/spi/spi.h>         

#include <nuttx/timers/pwm.h>
#include "stm32_pwm.h"

#include <stm32_gpio.h>
#include <stm32_exti.h>
#include <hardware/stm32_pinmap.h>

#include <nuttx/sensors/bmi160.h>
#include <nuttx/sensors/bmp280.h>
//#include <nuttx/sensors/qmc5883l.h>
#include <nuttx/sensors/vl53l1x.h>
#include <sched.h>

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

#ifdef CONFIG_WL_NRF24L01
#include <nuttx/wireless/nrf24l01.h>
#include "stm32_exti.h"
#include "stm32_gpio.h"

/* Forward declaration for the ISR hook */
static int nrf24_irq_attach(xcpt_t isr, FAR void *arg)
{
  /* Attach PB5 to the falling edge EXTI interrupt */
  return stm32_gpiosetevent(GPIO_NRF24L01_IRQ, false, true, false, isr, arg);
}

static void nrf24_chip_enable(bool enable)
{
  stm32_gpiowrite(GPIO_NRF24L01_CE, enable);
}

/* The strictly compliant configuration structure */
static struct nrf24l01_config_s g_nrf24l01_config =
{
  .irqattach  = nrf24_irq_attach,
  .chipenable = nrf24_chip_enable,
};
#endif


extern int pasil_imu_task(int argc, char *argv[]);

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
      syslog(LOG_INFO, "I2C1 Initialized.\n");
      
      /* Phase 2 Relic: Expose the bus to the user-space NSH tool for radar sweeps */
      ret = i2c_register(i2c, 1);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register /dev/i2c1\n");
        }
      else
        {
          syslog(LOG_INFO, "I2C1 bus registered at /dev/i2c1\n");
        }
      
      syslog(LOG_INFO, "[PASIL] Registering Sensor Fleet...\n");
      
      /* 1. IMU - PHASE 1 TARGET ALOCKED */
      syslog(LOG_INFO, "--> Probing BMI160 at /dev/imu0...\n");
      /* NOTE: Change the sensor address in menuconfig to match with actual address */
      ret = bmi160_register("/dev/imu0", i2c);
      if (ret < 0) {
          syslog(LOG_ERR, "ERROR: BMI160 failed: %d\n", ret);
      }
      else
      {
            syslog(LOG_INFO, "    BMI160 Registered Successfully.\n");

      /* ==========================================================
       * PHASE 6B: KINETIC ACTUATION NETWORK (MX1508)
       * ========================================================== */
      syslog(LOG_INFO, "[PASIL] Arming Kinetic Actuation Network...\n");
      struct pwm_lowerhalf_s *pwm2;
      struct pwm_lowerhalf_s *pwm3;

      /* 1. Initialize TIM2 (PA1, PA2, PA3 - Motors 1, 2, 3) */
      syslog(LOG_INFO, "    Routing TIM2 (CH2, CH3, CH4) to /dev/pwm0...\n");
      pwm2 = stm32_pwminitialize(2);
      if (!pwm2)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize TIM2 PWM\n");
        }
      else
        {
          ret = pwm_register("/dev/pwm0", pwm2);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm0: %d\n", ret);
            }
          else
            {
              syslog(LOG_INFO, "    TIM2 PWM Registered Successfully at /dev/pwm0.\n");
            }
        }

      /* 2. Initialize TIM3 (PA4 - Motor 4) */
      syslog(LOG_INFO, "    Routing TIM3 (CH1) to /dev/pwm1...\n");
      pwm3 = stm32_pwminitialize(3);
      if (!pwm3)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize TIM3 PWM\n");
        }
      else
        {
          ret = pwm_register("/dev/pwm1", pwm3);
          if (ret < 0)
            {
              syslog(LOG_ERR, "ERROR: Failed to register /dev/pwm1: %d\n", ret);
            }
          else
            {
              syslog(LOG_INFO, "    TIM3 PWM Registered Successfully at /dev/pwm1.\n");
            }
        }

      /* Spawn the IMU polling thread dynamically */
      syslog(LOG_INFO, "[PASIL] Spawning Deterministic IMU Thread (Priority 255)...\n");
      ret = task_create("pasil_imu", 255, 2048, pasil_imu_task, NULL);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to spawn IMU task\n");
        }

      }

      /* * 2. Barometer 
       */
       //syslog(LOG_INFO, "--> Probing BMP280...\n");
       //ret = bmp280_register("/dev/baro0", i2c);
       //if (ret < 0) syslog(LOG_ERR, "ERROR: BMP280 failed: %d\n", ret);

      /* * 3. Magnetometer 
       *[OBSOLETE] Directly injected in pasil_imu.c task.
       */
       //ret = qmc5883l_register("/dev/mag0", i2c);
       //if (ret < 0) syslog(LOG_ERR, "ERROR: QMC5883 failed: %d\n", ret);

      /* * 4. Time-of-Flight Laser
       * [QUARANTINED] L1X driver polling loop hangs RTOS on L0X silicon. 
       * Do NOT uncomment until a dynamic driver is written.
       */
      // syslog(LOG_INFO, "--> Probing VL53...\n");
      // ret = vl53l1x_register("/dev/tof0", i2c);
      // if (ret < 0) syslog(LOG_ERR, "ERROR: VL53L1X failed: %d\n", ret);

    }
#endif

#ifdef CONFIG_WL_NRF24L01
  struct spi_dev_s *spi2;
  
  syslog(LOG_INFO, "[PASIL] Arming SPI2 for NRF24L01+PA+LNA...\n");
  
  /* Configure GPIOs before SPI init */
  stm32_configgpio(GPIO_NRF24L01_CSN);
  stm32_configgpio(GPIO_NRF24L01_CE);

  spi2 = stm32_spibus_initialize(2);
  if (!spi2)
    {
      syslog(LOG_ERR, "ERROR: SPI2 bus acquisition failed\n");
    }
  else
    {
      /* Register using the verified struct */
      ret = nrf24l01_register(spi2, &g_nrf24l01_config);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: NRF24L01 registration failed: %d\n", ret);
        }
      else
        {
          syslog(LOG_INFO, "[PASIL] NRF24L01 initialized at /dev/nrf24l01\n");
        }
    }
#endif


  return ret;
}
