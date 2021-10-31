/** SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2021 Peter Niebert <peter.niebert@univ-amu.fr>
 *
 * @brief BBC micro:bit legacy display APIs.
 *
 * This file is here for compatibility with the replacement
 * microbit_display.h for old projects. See microbit_display.h
 * for type and function definitions.
 *
 */


#ifndef ZEPHYR_INCLUDE_DISPLAY_MB_DISPLAY_H_
#define ZEPHYR_INCLUDE_DISPLAY_MB_DISPLAY_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "microbit_display.h"

struct mb_display;

/**
 * @brief Get a pointer to the BBC micro:bit display object.
 *
 * @return Pointer to display object.
 */
inline struct mb_display *mb_display_get(void)
{
	return NULL;
}

static inline void mb_display_image(struct mb_display *disp, uint32_t mode, int32_t duration,
		      const struct mb_image *img, uint8_t img_count)
{
	(void) disp;
	mb_display_image_v2(mode, duration, img, img_count);
}

static inline void mb_display_print(struct mb_display *disp, uint32_t mode, int32_t duration,
			const char *fmt, ...)
{
	va_list args;

	(void) disp;
	va_start(args, fmt);
	mb_display_print_v2(mode, duration, fmt, args);
	va_end(args);
}

static inline void mb_display_stop(struct mb_display *disp)
{
	(void) disp;
	mb_display_stop_v2();
}

#ifdef __cplusplus
}
#endif
#endif
