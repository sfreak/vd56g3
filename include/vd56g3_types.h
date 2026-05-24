/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * VD56G3 camera sensor register type definition.
 */
typedef struct {
    uint32_t reg;
    uint32_t val;
} vd56g3_reginfo_t;

#ifdef __cplusplus
}
#endif
