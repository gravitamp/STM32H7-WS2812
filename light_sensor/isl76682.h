/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_isl76682 ISL76682 light sensor
 * @ingroup     drivers_sensors
 * @ingroup     drivers_saul
 * @brief       Device driver for the ISL76682 light sensor
 *
 * This driver provides @ref drivers_saul capabilities.
 * @{
 *
 * @file
 * @brief       Device driver interface for the ISL76682 light sensor
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef ISL76682_H
#define ISL76682_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup drivers_isl76682_config     ISL76682 light sensor driver compile configuration
 * @ingroup config_drivers_sensors
 * @{
 */
/**
 * @brief Default address
 *
 * The address depends on the status of A0 Pin. Default address corresponds to
 * A0 connected to GND. For more information refer to the section 'I2C
 * Interface' in the datasheet.
 */
#ifndef CONFIG_ISL76682_DEFAULT_ADDRESS
#define CONFIG_ISL76682_DEFAULT_ADDRESS        (0x44<<1)
#endif
/** @} */

/**
 * @brief   Possible modes for the ISL76682 sensor
 */
typedef enum {
    ISL76682_MODE_AMBIENT = 0,  /**< set sensor to detect ambient light */
    ISL76682_MODE_IR      = 1   /**< set sensor to detect infrared light */
} isl76682_mode_t;

/**
 * @brief   Possible range values for the ISL76682 sensor
 */
typedef enum {
    ISL76682_RANGE_1K = 0,      /**< set range to 0-1000 lux */
    ISL76682_RANGE_4K = 1,      /**< set range to 0-4000 lux */
    ISL76682_RANGE_16K = 2,     /**< set range to 0-16000 lux */
    ISL76682_RANGE_64K = 3      /**< set range to 0-64000 lux */
} isl76682_range_t;

/**
 * @brief   Data structure holding the full set of configuration parameters
 */
typedef struct {
    uint8_t addr;               /**< address on that bus */
    isl76682_range_t range;     /**< range setting to use */
    isl76682_mode_t mode;       /**< measurement mode to use */
} isl76682_params_t;


/**
 * @brief   Initialize a new ISL76682 device
 *
 * @param[in] dev       device descriptor of an ISL76682 device
 * @param[in] params    initialization parameters
 *
 * @return              0 on success
 * @return              -1 on error
 */
int isl76682_init(uint16_t addr);

/**
 * @brief   Read a lighting value from the sensor, the result is given in lux
 *
 * @param[in] dev       device descriptor of an ISL76682 device
 *
 * @return              the measured brightness in lux
 * @return              -1 on error
 */
int isl76682_read(const uint16_t addr);

#ifdef __cplusplus
}
#endif

#endif /* ISL76682_H */
/** @} */
