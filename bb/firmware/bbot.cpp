/****************************************************************************
 *
 *   Copyright (C) 2015 Jonathan Binney. All rights reserved.
 *   Author: @author Jonathan Binney
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file bbot.cpp
 * Controller application for my battlebot.
 */

#include <px4_config.h>
#include <nuttx/config.h>
#include <px4_posix.h> 
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <time.h>
#include <drivers/device/spi.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

/* External SPI device 1 has its chipselect on
 * PORT C, Pin 14, which is connected to GPIO_EXT_1 on the board
 * (this is on the SPI DF13 connector)
 */
#define BBOT_ENCODER1_BUS PX4_SPI_BUS_EXT
#define BBOT_ENCODER1_DEVICE PX4_SPIDEV_EXT0

/* External SPI device 0 has its chipselect (inverted) on
 * PORT E, Pin 4, which is connected to SPI_EXT_NSS on the board
 * (this is on the SPI DF13 connector)
 */
#define BBOT_ENCODER2_BUS PX4_SPI_BUS_EXT
#define BBOT_ENCODER2_DEVICE PX4_SPIDEV_EXT1

#define NUM_PWM_OUTPUTS 6
#define PWM_RATE 2500
#define PWM_CLOCK_SPEED 1000000

struct GPIOConfig {
  uint32_t input;
  uint32_t output;
  uint32_t alt;
};

/** List of pins to configure as input or output */
const GPIOConfig gpio_tab[] = {
  {0,       GPIO_GPIO0_OUTPUT,       0},
  {0,       GPIO_GPIO1_OUTPUT,       0},
  {0,       GPIO_GPIO2_OUTPUT,       0},
  {0,       GPIO_GPIO3_OUTPUT,       0},
  {0,       GPIO_GPIO4_OUTPUT,       0},
  {0,       GPIO_GPIO5_OUTPUT,       0},

  {0,                      GPIO_VDD_5V_PERIPH_EN,   0},
  {0,                      GPIO_VDD_3V3_SENSORS_EN, 0},
  {GPIO_VDD_BRICK_VALID,   0,                       0},
  {GPIO_VDD_SERVO_VALID,   0,                       0},
  {GPIO_VDD_5V_HIPOWER_OC, 0,                       0},
  {GPIO_VDD_5V_PERIPH_OC,  0,                       0},
};
const unsigned ngpio = sizeof(gpio_tab) / sizeof(gpio_tab[0]);

extern "C"
{

class AS5045 : device::SPI
{
  public:
  AS5045(const char *dev_name, const char *dev_path, spi_dev_e spi_dev) :
    device::SPI(dev_name, dev_path, PX4_SPI_BUS_EXT, spi_dev,
      SPIDEV_MODE1, /* CPOL=0 CPHA=1 (setup data on rising edge, read on falling edge of clock */
      1000*1000 /* 1 MHz is the max for the AS5045 */)
  {
    if (SPI::init() != OK) {
      warnx("SPI init failed");
    }
    printf("SPI initialized\n");
  }

  uint32_t measure() {
    uint32_t data = 0;
    transfer(nullptr, (uint8_t *) (&data), 18);
    uint32_t position = (0x000FF & data) * 256 + (0x00f00 & data);
    // TODO: Check status bits
    return position;
  }
};

class HighSpeedPWM
{
public:
  HighSpeedPWM(unsigned int pwm_rate) :
    _pwm_rate(pwm_rate), _pwm_period(1.0f / (float) pwm_rate)
  {
    /* There's some sleeps in here that might not all be necessary... */

    printf("Resetting GPIO\n");
    gpioReset();

    usleep(500000);

    /* Magic number borrowed from src/drivers/px4fmu/fmu.cpp */
    printf("Initializing PWM\n");
    up_pwm_servo_init(0x3f);

    usleep(500000);

    /* Set PWM Rate for all outputs */
    printf("Setting PWM rate\n");
    if(up_pwm_servo_set_rate(_pwm_rate) != OK) {
      printf("set rate failed");
      return;
    }

    usleep(500000);

    printf("Arming PWM\n");
    up_pwm_servo_arm(true);

    usleep(500000);
  }

  void gpioReset(void)
  {
    /*
     * Setup default GPIO config - all pins as GPIOs, input if
     * possible otherwise output if possible.
     */
    for (unsigned gpio_i = 0; gpio_i < ngpio; gpio_i++) {
      if (gpio_tab[gpio_i].input != 0) {
        stm32_configgpio(gpio_tab[gpio_i].input);

      } else if (gpio_tab[gpio_i].output != 0) {
        stm32_configgpio(gpio_tab[gpio_i].output);
      }
    }
  }

  void set_fractions(const float pwm_fractions[]) {
    /* Set pulse width for all outputs */
    for(size_t pwm_i = 0; pwm_i < 6; ++pwm_i) {
      unsigned int pwm_value = (unsigned int) (pwm_fractions[pwm_i] * _pwm_period * PWM_CLOCK_SPEED);
      if(pwm_i == 0) {
        printf("%.2e, %d\n", (double) pwm_fractions[pwm_i], pwm_value);
      }

      up_pwm_servo_set(pwm_i, pwm_value);
    }
  }

private:
  unsigned _pwm_rate;
  float _pwm_period;
};

__EXPORT int bbot_main(int argc, char *argv[]);

int bbot_main(int argc, char *argv[]) {
  printf("Battlebot starting up!\n");
  //AS5045 enc1("enc1", "/dev/enc1", (spi_dev_e) BBOT_ENCODER2_DEVICE);
  //printf("Encoder value: %d\n", enc1.measure());
  HighSpeedPWM pwm_driver(PWM_RATE);

  int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));

  float delta_t = 0.1;

  uint64_t t = hrt_absolute_time();
  uint64_t t_last_printf = t;
  float yaw = 0.0;

  for(int ii = 0; ii < 10000000; ii++) {
    bool gyro_updated;
    orb_check(gyro_sub, &gyro_updated);
    if (gyro_updated) {
      struct gyro_report gyro_report;
      orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro_report);

      math::Vector<3> vect(gyro_report.x, gyro_report.y, gyro_report.z);

      // Figure out how long it has been since the last loop
      uint64_t t_new = hrt_absolute_time();
      float delta_t = (t_new - t) / 1e6;
      t = t_new;

      yaw += delta_t * gyro_report.z;
      // printf("gyro_report: (%.3f, %.3f, %.3f)\n",
      //    (double) gyro_report.x, (double) gyro_report.y, (double) gyro_report.z);
      if((t - t_last_printf) > 500000) {
        printf("yaw: %.3f\n", (double) yaw);
        t_last_printf = t;
      }
    }
    usleep((unsigned int) (100));
  }
  return 0;

  float pwm_fractions[NUM_PWM_OUTPUTS] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
  float cycle_rate = 0.5;
  while(true) {
    printf("Setting pwm\n");
    pwm_driver.set_fractions(pwm_fractions);
    for(size_t pwm_i = 0; pwm_i < NUM_PWM_OUTPUTS; pwm_i++) {
      pwm_fractions[pwm_i] = pwm_fractions[pwm_i] + delta_t * cycle_rate;
      while(pwm_fractions[pwm_i] > 1.0f) {
        pwm_fractions[pwm_i] -= 1.0f;
      }
    }
    usleep((unsigned int) (delta_t * 1000000));
  }
  return OK;
}

} /* extern "C" */
