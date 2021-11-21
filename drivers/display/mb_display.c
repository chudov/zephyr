/*
 * Copyright (c) 2017 Intel Corporation
 * Copyright (c) 2021 Alexander Chudov <chudov@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * References:
 *
 * https://www.microbit.co.uk/device/screen
 * https://lancaster-university.github.io/microbit-docs/ubit/display/
 */

#define DT_DRV_COMPAT microbit_matrixled

#include <zephyr.h>
#include <init.h>
#include <drivers/gpio.h>
#include <device.h>
#include <string.h>
#include <sys/printk.h>

#include <display/mb_display.h>

#include "mb_font.h"

#define MODE_MASK    BIT_MASK(16)

#define DISPLAY_USES_COL_PORTS DT_INST_PROP_LEN(0, col_gpios_used)
#define DISPLAY_ROWS (DT_INST_PROP_LEN(0, row_gpios))
#define DISPLAY_COLS (DT_INST_PROP_LEN(0, col_gpios))

#define MB_DISPLAY_HEIGHT 5
#define MB_DISPLAY_WIDTH 5

#define SCROLL_OFF   0
#define SCROLL_START 1

struct mb_display {
	struct k_timer  timer;       /* Rendering timer */

	uint8_t            img_count;   /* Image count */

	uint8_t            cur_img;     /* Current image or character to show */

	uint8_t            scroll:3,    /* Scroll shift */
			first:1,     /* First frame of a scroll sequence */
			loop:1,      /* Loop to beginning */
			text:1,      /* We're showing a string (not image) */
			img_sep:1;   /* One column image separation */

	/* The following variables track the currently shown image */
	uint8_t            cur;         /* Currently rendered row */
	uint32_t           row[DISPLAY_ROWS][DISPLAY_USES_COL_PORTS]; /* Content for each row */
	int64_t           expiry;      /* When to stop showing current image */
	int32_t           duration;    /* Duration for each shown image */

	uint32_t			mask[DISPLAY_USES_COL_PORTS]; /* Mask for proper updating columns bits */
	union {
		const struct mb_image *img; /* Array of images to show */
		const char            *str; /* String to be shown */
	};

	/* Buffer for printed strings */
	char            str_buf[CONFIG_MICROBIT_DISPLAY_STR_MAX];
};

struct x_y {
	uint8_t x:4,
	y:4;
};

struct gpio_dt_spec_port_num {
	const struct gpio_dt_spec dt_spec;
	const uint8_t port_num;
};

struct mb_display_cfg {
	const struct device *dev[DT_INST_PROP_LEN(0, col_gpios_used)]; /* GPIO devices */

	/* Mapping of the display logical pixels to physical LED, connected at one side to a specific
 	 * row and on another side to a specific pin of a specific GPIO port.
 	 * The top left corner has the coordinates 0,0.
 	 */
	const struct x_y pixels_map[MB_DISPLAY_HEIGHT][MB_DISPLAY_WIDTH];
	/* Map of rows GPIO pins*/	
	const struct gpio_dt_spec_port_num row_map[DT_INST_PROP_LEN(0, row_gpios)];
	/* Map of columns GPIO pins */
	const struct gpio_dt_spec_port_num col_map[DT_INST_PROP_LEN(0, col_gpios)];
	/* Masks for columns ports */
	const uint32_t mask[DT_INST_PROP_LEN(0, col_gpios_used)];
};

/* Helper to use two different DT macros with similar parameters */
#define GPIO_MAPPING(node_id, prop, idx) \
	{GPIO_DT_SPEC_GET_BY_IDX(node_id, prop, idx),\
	 DT_PROP_BY_PHANDLE_IDX(node_id, prop, idx, port)},

#define GET_PORT(node_id, prop, idx) \
	DEVICE_DT_GET(DT_PROP_BY_IDX(node_id, prop, idx)),


#define BUILD_A_MASK(node_id, prop, idx) \
	BIT(DT_GPIO_PIN_BY_IDX(node_id, prop, idx))|

static const struct mb_display_cfg display_cfg = {
	.mask = {
		DT_INST_FOREACH_PROP_ELEM(0, col_gpios, BUILD_A_MASK)
	0},
	.dev = {
		DT_INST_FOREACH_PROP_ELEM(0, col_gpios_used, GET_PORT)
	},

	.row_map = { 
		DT_INST_FOREACH_PROP_ELEM(0, row_gpios,GPIO_MAPPING)
	},

	.col_map = {
		DT_INST_FOREACH_PROP_ELEM(0, col_gpios,GPIO_MAPPING)
	},

#if defined(CONFIG_BOARD_BBC_MICROBIT)
	.pixels_map = {
		{{0, 0}, {1, 3}, {0, 1}, {1, 4}, {0, 2}},
		{{2, 3}, {2, 4}, {2, 5}, {2, 6}, {2, 7}},
		{{1, 1}, {0, 8}, {1, 2}, {2, 8}, {1, 0}},
		{{0, 7}, {0, 6}, {0, 5}, {0, 4}, {0, 3}},
		{{2, 2}, {1, 6}, {2, 0}, {1, 5}, {2, 1}}
	},
#elif defined(CONFIG_BOARD_BBC_MICROBIT_V2)
	.pixels_map = {
		{{0, 0}, {0, 1}, {0, 2}, {0, 3}, {0, 4}},
		{{1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}},
		{{2, 0}, {2, 1}, {2, 2}, {2, 3}, {2, 4}},
		{{3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}},
		{{4, 0}, {4, 1}, {4, 2}, {4, 3}, {4, 4}}
	},
#endif
};

static inline const struct mb_image *get_font(char ch)
{
	if (ch < MB_FONT_START || ch > MB_FONT_END) {
		return &mb_font[' ' - MB_FONT_START];
	}

	return &mb_font[ch - MB_FONT_START];
}

#define GET_PIXEL(img, x, y) ((img)->row[y] & BIT(x))

/* Precalculate all rows of an image and start the rendering. */
static void start_image(struct mb_display *disp, const struct mb_image *img)
{
	uint8_t row, col, port;
	const struct mb_display_cfg * cfg = &display_cfg;

	for (port = 0; port < DISPLAY_USES_COL_PORTS; port++) {
		for (row = 0; row < DISPLAY_ROWS; row++) {
			disp->row[row][port] = 0;
		}
	}

	for (row = 0; row < MB_DISPLAY_HEIGHT; row++) {
		for (col = 0; col < MB_DISPLAY_WIDTH; col++) {
			if (GET_PIXEL(img, col, row)) {
				disp->row[cfg->pixels_map[row][col].row]
					[cfg->col_map[cfg->pixels_map[row][col].col].port_num] |=
					BIT(cfg->col_map[cfg->pixels_map[row][col].col].dt_spec.pin);
			}
		}
	}
	disp->cur = 0U;

	if (disp->duration == SYS_FOREVER_MS) {
		disp->expiry = SYS_FOREVER_MS;
	} else {
		disp->expiry = k_uptime_get() + disp->duration;
	}

	k_timer_start(&disp->timer, K_NO_WAIT, K_MSEC(4));
}

static inline void update_pins(struct mb_display *disp, uint32_t val[])
{
	uint32_t prev = (disp->cur + (DISPLAY_ROWS-1)) % DISPLAY_ROWS;

	/* Disable the previous row */
	gpio_pin_set_dt(&display_cfg.row_map[prev].dt_spec, 0);

	/* Set the column pins to their correct values */
	for (uint8_t port = 0; port < DISPLAY_USES_COL_PORTS; port++) {
		gpio_port_set_masked(display_cfg.dev[port], disp->mask[port], val[port]);
	}

	/* Enable the new row */
	gpio_pin_set_dt(&display_cfg.row_map[disp->cur].dt_spec, 1);
}

static void reset_display(struct mb_display *disp)
{
	k_timer_stop(&disp->timer);

	disp->str = NULL;
	disp->cur_img = 0U;
	disp->img = NULL;
	disp->img_count = 0U;
	disp->scroll = SCROLL_OFF;
}

static const struct mb_image *current_img(struct mb_display *disp)
{
	if (disp->scroll && disp->first) {
		return get_font(' ');
	}

	if (disp->text) {
		return get_font(disp->str[disp->cur_img]);
	} else {
		return &disp->img[disp->cur_img];
	}
}

static const struct mb_image *next_img(struct mb_display *disp)
{
	if (disp->text) {
		if (disp->first) {
			return get_font(disp->str[0]);
		} else if (disp->str[disp->cur_img]) {
			return get_font(disp->str[disp->cur_img + 1]);
		} else {
			return get_font(' ');
		}
	} else {
		if (disp->first) {
			return &disp->img[0];
		} else if (disp->cur_img < (disp->img_count - 1)) {
			return &disp->img[disp->cur_img + 1];
		} else {
			return get_font(' ');
		}
	}
}

static inline bool last_frame(struct mb_display *disp)
{
	if (disp->text) {
		return (disp->str[disp->cur_img] == '\0');
	} else {
		return (disp->cur_img >= disp->img_count);
	}
}

static inline uint8_t scroll_steps(struct mb_display *disp)
{
	return MB_DISPLAY_WIDTH + disp->img_sep;
}

static void update_scroll(struct mb_display *disp)
{
	if (disp->scroll < scroll_steps(disp)) {
		struct mb_image img;
		int i;

		for (i = 0; i < MB_DISPLAY_HEIGHT; i++) {
			const struct mb_image *i1 = current_img(disp);
			const struct mb_image *i2 = next_img(disp);

			img.row[i] = ((i1->row[i] >> disp->scroll) |
				      (i2->row[i] << (scroll_steps(disp) -
						      disp->scroll)));
		}

		disp->scroll++;
		start_image(disp, &img);
	} else {
		if (disp->first) {
			disp->first = 0U;
		} else {
			disp->cur_img++;
		}

		if (last_frame(disp)) {
			if (!disp->loop) {
				reset_display(disp);
				return;
			}

			disp->cur_img = 0U;
			disp->first = 1U;
		}

		disp->scroll = SCROLL_START;
		start_image(disp, current_img(disp));
	}
}

static void update_image(struct mb_display *disp)
{
	disp->cur_img++;

	if (last_frame(disp)) {
		if (!disp->loop) {
			reset_display(disp);
			return;
		}

		disp->cur_img = 0U;
	}

	start_image(disp, current_img(disp));
}

static void show_row(struct k_timer *timer)
{
	struct mb_display *disp = CONTAINER_OF(timer, struct mb_display, timer);
	update_pins(disp, disp->row[disp->cur]);
	disp->cur = (disp->cur + 1) % DISPLAY_ROWS;

	if (disp->cur == 0U && disp->expiry != SYS_FOREVER_MS &&
	    k_uptime_get() > disp->expiry) {
		if (disp->scroll) {
			update_scroll(disp);
		} else {
			update_image(disp);
		}
	}
}

static void clear_display(struct k_timer *timer)
{
	struct mb_display *disp = CONTAINER_OF(timer, struct mb_display, timer);
	uint32_t clean_disp[DISPLAY_USES_COL_PORTS];

	memset(clean_disp, 0, sizeof(clean_disp));
	update_pins(disp, clean_disp);
}

static struct mb_display display = {
	.timer = Z_TIMER_INITIALIZER(display.timer, show_row, clear_display),
};

static void start_scroll(struct mb_display *disp, int32_t duration)
{
	/* Divide total duration by number of scrolling steps */
	if (duration) {
		disp->duration = duration / scroll_steps(disp);
	} else {
		disp->duration = CONFIG_MICROBIT_SCROLL_DEFAULT_DURATION_MS;
	}

	disp->scroll = SCROLL_START;
	disp->first = 1U;
	disp->cur_img = 0U;
	start_image(disp, get_font(' '));
}

static void start_single(struct mb_display *disp, int32_t duration)
{
	disp->duration = duration;

	if (disp->text) {
		start_image(disp, get_font(disp->str[0]));
	} else {
		start_image(disp, disp->img);
	}
}

void mb_display_image(struct mb_display *disp, uint32_t mode, int32_t duration,
		      const struct mb_image *img, uint8_t img_count)
{
	reset_display(disp);

	__ASSERT(img && img_count > 0, "Invalid parameters");

	disp->text = 0U;
	disp->img_count = img_count;
	disp->img = img;
	disp->img_sep = 0U;
	disp->cur_img = 0U;
	disp->loop = !!(mode & MB_DISPLAY_FLAG_LOOP);

	switch (mode & MODE_MASK) {
	case MB_DISPLAY_MODE_DEFAULT:
	case MB_DISPLAY_MODE_SINGLE:
		start_single(disp, duration);
		break;
	case MB_DISPLAY_MODE_SCROLL:
		start_scroll(disp, duration);
		break;
	default:
		__ASSERT(0, "Invalid display mode");
	}
}

void mb_display_stop(struct mb_display *disp)
{
	reset_display(disp);
}

void mb_display_print(struct mb_display *disp, uint32_t mode,
		      int32_t duration, const char *fmt, ...)
{
	va_list ap;

	reset_display(disp);

	va_start(ap, fmt);
	vsnprintk(disp->str_buf, sizeof(disp->str_buf), fmt, ap);
	va_end(ap);

	if (disp->str_buf[0] == '\0') {
		return;
	}

	disp->str = disp->str_buf;
	disp->text = 1U;
	disp->img_sep = 1U;
	disp->cur_img = 0U;
	disp->loop = !!(mode & MB_DISPLAY_FLAG_LOOP);

	switch (mode & MODE_MASK) {
	case MB_DISPLAY_MODE_DEFAULT:
	case MB_DISPLAY_MODE_SCROLL:
		start_scroll(disp, duration);
		break;
	case MB_DISPLAY_MODE_SINGLE:
		start_single(disp, duration);
		break;
	default:
		__ASSERT(0, "Invalid display mode");
	}
}

struct mb_display *mb_display_get(void)
{
	return &display;
}

static int mb_display_init(const struct device *dev)
{
	//struct mb_display *disp = dev->data;
	//const struct mb_display_cfg *cfg = dev->config;

	for(uint8_t i = 0; i< DISPLAY_ROWS; i++) {
		gpio_pin_configure_dt(&display_cfg.row_map[i].dt_spec, GPIO_OUTPUT_INACTIVE | GPIO_DS_ALT_HIGH);
	}

	for(uint8_t i = 0; i< DISPLAY_COLS; i++) {
		gpio_pin_configure_dt(&display_cfg.col_map[i].dt_spec, GPIO_OUTPUT_INACTIVE | GPIO_DS_ALT_LOW);
	}
 
	display.mask[0] = 0;
	display.mask[1] = 0;

	for(uint8_t i=0; i < DISPLAY_COLS; i++) {
		display.mask[display_cfg.col_map[i].port_num] |= 
			BIT(display_cfg.col_map[i].dt_spec.pin);
	}

	return 0;
}
/*
#define LED_GPIO_DT_SPEC(led_node_id)				\
	GPIO_DT_SPEC_GET(led_node_id, gpios),			\

#define LED_GPIO_DEVICE(instance)						\
									\
static const struct gpio_dt_spec row_gpio_dt_spec_##instance[] = {	\
	DT_INST_FOREACH_CHILD(instance, LED_GPIO_DT_SPEC)		\
};									\
									\
static const struct gpio_dt_spec col_gpio_dt_spec_##instance[] = {	\
	DT_INST_FOREACH_CHILD(instance, LED_GPIO_DT_SPEC)		\
};									\
									\
static const struct mb_display_cfg mb_display_cfg_##instance = {	\
	.row		= row_gpio_dt_spec_##instance,			\
	.col		= col_gpio_dt_spec_##instance,			\
};									\
									\
DEVICE_DT_INST_DEFINE(instance, &mb_display_init, NULL,			\
		      NULL, &mb_display_cfg_##instance,			\
		      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,	\
		      NULL);

DT_INST_FOREACH_STATUS_OKAY(LED_GPIO_DEVICE)
*/

//DEVICE_DT_INST_DEFINE(0, mb_display_init, NULL,
//		      &display, &display_cfg,
//		      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,
//		      NULL/*&ls0xx_driver_api*/);


 SYS_INIT(mb_display_init, POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY);
