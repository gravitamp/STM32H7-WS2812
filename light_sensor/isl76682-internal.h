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
 * @brief       Definitions for the ISL76682 light sensor
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 */

#ifndef ISL76682_INTERNAL_H
#define ISL76682_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name ISL76682 registers
 * @{
 */
#define ISL76682_REG_CMD        0x00
#define ISL76682_REG_LDATA      0x01
#define ISL76682_REG_HDATA      0x02
/** @} */

/**
 * @name Masks for the ISL76682 command register
 * @{
 */
#define ISL76682_CMD_EN         0x80
#define ISL76682_CMD_MODE       0x40
#define ISL76682_CMD_LIGHT      0x20
#define ISL76682_CMD_RES        0x1c
#define ISL76682_CMD_RAGNE      0x03
/** @} */

/**
 * @name Resolution options
 * @{
 */
#define ISL76682_RES_INT_16     0x00
#define ISL76682_RES_INT_12     0x04
#define ISL76682_RES_INT_8      0x08
#define ISL76682_RES_INT_4      0x0c
#define ISL76682_RES_EXT_ADC    0x10
#define ISL76682_RES_EXT_TIM    0x14
/** @} */

/**
 * @name Range options
 * @{
 */
#define ISL76682_RANGE_1        0x00
#define ISL76682_RANGE_2        0x01
#define ISL76682_RANGE_3        0x02
#define ISL76682_RANGE_4        0x03
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* ISL76682_INTERNAL_H */
/** @} */
