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
#include <stdio.h>
#include <errno.h>
#include <drivers/device/spi.h>
#include <board_config.h>

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

__EXPORT int bbot_main(int argc, char *argv[]);

int bbot_main(int argc, char *argv[]) {
  printf("Battlebot starting up!\n");
  AS5045 enc1("enc1", "/dev/enc1", (spi_dev_e) BBOT_ENCODER2_DEVICE);
  printf("Encoder value: %d\n", enc1.measure());
  return OK;
}

} /* extern "C" */
