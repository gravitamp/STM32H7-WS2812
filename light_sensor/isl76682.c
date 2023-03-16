/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_isl76682
 * @{
 *
 * @file
 * @brief       Device driver implementation for the ISL76682 light sensor
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Peter Kietzmann <peter.kietzmann@haw-hamburg.de>
 *
 * @}
 */

/**
 * @brief   Device descriptor for ISL76682 sensors
 */

float lux_fac;              /**< factor to calculate actual lux value */

#include <isl76682.h>
#include <isl76682-internal.h>
#include "i2c.h"

#define DEV_RANGE    ISL76682_RANGE_16K
#define DEV_MODE     ISL76682_MODE_AMBIENT

int isl76682_init(uint16_t addr)
{
	int res;
	uint8_t config;

	/* initialize device descriptor */

	lux_fac = (float)((1 << (10 + (2 * DEV_RANGE))) - 1) / 0xffff;

	/* acquire access to device */
	if (HAL_I2C_IsDeviceReady(&hi2c4, addr, 3, 1000) != HAL_OK) // shift karena ada start bit
	{
	  /* Return false */
	  return -1;
	}

	/* configure and enable the sensor */

	config = (ISL76682_CMD_EN | ISL76682_CMD_MODE |
	    ISL76682_RES_INT_16 | DEV_RANGE | (DEV_MODE << 5));
	res = HAL_I2C_Mem_Write(&hi2c4, addr, ISL76682_REG_CMD,
	                        I2C_MEMADD_SIZE_8BIT, &config, sizeof(config), 1000);
	if (res < 0)
	{
	  return -1;
	}

	return 0;
}

int isl76682_read(const uint16_t addr)
{
  uint8_t low, high;
  uint16_t res;
  int ret;

  /* read lighting value */
  ret = HAL_I2C_Mem_Read(&hi2c4, addr, ISL76682_REG_LDATA,
                         I2C_MEMADD_SIZE_8BIT, &low, 1, 1000);
  ret += HAL_I2C_Mem_Read(&hi2c4, addr, ISL76682_REG_HDATA,
                          I2C_MEMADD_SIZE_8BIT, &high, 1, 1000);
  if (ret != 0) //check error
  {
    return -1;
  }
  res = (high << 8) | low;
  /* calculate and return the actual lux value */
  return (int)(lux_fac * res);
}
