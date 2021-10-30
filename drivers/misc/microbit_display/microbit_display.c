/* SPDX-License-Identifier: Apache-2.0
 * Copyright (c) 2021 Peter Niebert, Aix-Marseille University
 * Copyright (c) 2021 Alexander Chudov, chudov@gmail.com
 *
 *
 * New implementation of mb_display.h for compatibility with Microbit V2.
 * This implementation uses a hardware timer of the Nordic SOC for periodic
 * interrupts independent of Kernel timing for image refresh.
 * The animation is timed separately by delayed work submitted to the kernel.
 *
 * References:
 *
 * https://www.microbit.co.uk/device/screen
 * https://lancaster-university.github.io/microbit-docs/ubit/display/
 * https://github.com/zephyrproject-rtos/zephyr/blob/main/include/...
 * ...display/mb_display.h
 */

#include <sys/__assert.h>

#include <zephyr.h>
#include <init.h>
#include <logging/log.h>
#include <sw_isr_table.h>

#include <stdint.h>
#include <stdarg.h>

#include <microbit_display.h>
#include <mb_font.h>

#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include <hal/nrf_gpio.h>
#include <nrfx_gpiote.h>

LOG_MODULE_REGISTER(mb2_display, LOG_LEVEL_DBG);

/************ pin related definitions for micro:bit V1 and V2 *********/

#define DISPLAY_ROWS (5)
#define DISPLAY_COLS (5)
#define SYMBOL_WIDTH (DISPLAY_COLS)

#ifdef CONFIG_BOARD_BBC_MICROBIT
/*
1.1 2.4 1.2 2.5 1.3
3.4 3.5 3.6 3.7 3.8
2.2 1.9 2.3 3.9 2.1
1.8 1.7 1.6 1.5 1.4
3.3 2.7 3.1 2.6 3.2
*/

static const uint16_t led_row_column[DISPLAY_ROWS * DISPLAY_COLS] = { 
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_4), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_4), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_6), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_7), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_8), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_9), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_9), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_8), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_7), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_6), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_4), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_7), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_6), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios)
};

static const uint32_t col_mask0 = 1 << DT_GPIO_PIN(DT_NODELABEL(col_1), gpios) |
								  1 << DT_GPIO_PIN(DT_NODELABEL(col_2), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_3), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_4), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_5), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_6), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_7), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_8), gpios) |
								  1 << DT_GPIO_PIN(DT_NODELABEL(col_9), gpios);

static const uint32_t row_mask = 1 << DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) |
								 1 << DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) |
								 1 << DT_GPIO_PIN(DT_NODELABEL(row_3), gpios);

/* use low level inline function for pin set */
#define mypinset nrf_gpio_pin_set
#define mypinclear nrf_gpio_pin_clear

#else
#ifdef CONFIG_BOARD_BBC_MICROBIT_V2

/* columns/anodes followed by rows/cathodes */

static const uint16_t led_row_column[DISPLAY_ROWS * DISPLAY_COLS] = {
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | (32 + DT_GPIO_PIN(DT_NODELABEL(col_4), gpios)),
	DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | (32 + DT_GPIO_PIN(DT_NODELABEL(col_4), gpios)),
	DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | (32 + DT_GPIO_PIN(DT_NODELABEL(col_4), gpios)),
	DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_4), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_4), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_4), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_4), gpios) << 8 | (32 + DT_GPIO_PIN(DT_NODELABEL(col_4), gpios)),
	DT_GPIO_PIN(DT_NODELABEL(row_4), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios),

	DT_GPIO_PIN(DT_NODELABEL(row_5), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_1), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_5), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_2), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_5), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_3), gpios),
	DT_GPIO_PIN(DT_NODELABEL(row_5), gpios) << 8 | (32 + DT_GPIO_PIN(DT_NODELABEL(col_4), gpios)),
	DT_GPIO_PIN(DT_NODELABEL(row_5), gpios) << 8 | DT_GPIO_PIN(DT_NODELABEL(col_5), gpios)

};

/* TODO check if C compiler handles this efficiently */
static const uint32_t row_mask = 1 << DT_GPIO_PIN(DT_NODELABEL(row_1), gpios) |
								 1 << DT_GPIO_PIN(DT_NODELABEL(row_2), gpios) |
								 1 << DT_GPIO_PIN(DT_NODELABEL(row_3), gpios) |
								 1 << DT_GPIO_PIN(DT_NODELABEL(row_4), gpios) | 
								 1 << DT_GPIO_PIN(DT_NODELABEL(row_5), gpios);
static const uint32_t col_mask0 = 1 << DT_GPIO_PIN(DT_NODELABEL(col_1), gpios) |
								  1 << DT_GPIO_PIN(DT_NODELABEL(col_2), gpios) |
                                  1 << DT_GPIO_PIN(DT_NODELABEL(col_3), gpios) |
								  1 << DT_GPIO_PIN(DT_NODELABEL(col_5), gpios);
static const uint32_t col_mask1 = 1 << DT_GPIO_PIN(DT_NODELABEL(col_4), gpios);

/**
 * optimized function for fast ISR, avoids port test
 */
static inline void mypinset(uint8_t pin)
{
	nrf_gpio_port_out_set(NRF_P0, 1 << pin);
}

static inline void mypinclear(uint8_t pin)
{
	int port = (pin & 32) != 0;

	if (port == 0) {
		nrf_gpio_port_out_clear(NRF_P0, 1 << pin);
	} else {
		nrf_gpio_port_out_clear(NRF_P1, 1 << (pin - 32));
	}
}

#else
#error "microbit led matrix only works on boards bbc_microbit and bbc_microbit_v2."
#endif
#endif

/**
 * NRFX Timer access to NRF hardware timer. The timer is configured
 * as MICROBIT_DISPLAY_TIMERn. Verify activation of NRFX_TIMERn, as
 * this may not be enforced by configuration.
 */
#ifdef CONFIG_MICROBIT_DISPLAY_TIMER0
#ifdef CONFIG_NRFX_TIMER0
#define DISPLAY_TIMER 0
#define DISPLAY_TIMER_IRQN TIMER0_IRQn
#else
#error "config. inconsistent: MICROBIT_DISPLAY_TIMER0=y and NRFX_TIMER0=n "
#endif
#endif
#ifdef CONFIG_MICROBIT_DISPLAY_TIMER1
#ifdef CONFIG_NRFX_TIMER1
#define DISPLAY_TIMER 1
#define DISPLAY_TIMER_IRQN TIMER1_IRQn
#else
#error "config. inconsistent: MICROBIT_DISPLAY_TIMER1=y and NRFX_TIMER1=n "
#endif
#endif
#ifdef CONFIG_MICROBIT_DISPLAY_TIMER2
#ifdef CONFIG_NRFX_TIMER2
#define DISPLAY_TIMER 2
#define DISPLAY_TIMER_IRQN TIMER1_IRQn
#else
#error "config. inconsistent: MICROBIT_DISPLAY_TIMER2=y and NRFX_TIMER2=n "
#endif
#endif
#ifdef CONFIG_MICROBIT_DISPLAY_TIMER3
#ifdef CONFIG_NRFX_TIMER3
#define DISPLAY_TIMER 3
#define DISPLAY_TIMER_IRQN TIMER3_IRQn
#else
#error "config. inconsistent: MICROBIT_DISPLAY_TIMER3=y and NRFX_TIMER3=n "
#endif
#endif
#ifdef CONFIG_MICROBIT_DISPLAY_TIMER4
#ifdef CONFIG_NRFX_TIMER4
#define DISPLAY_TIMER 4
#define DISPLAY_TIMER_IRQN TIMER4_IRQn
#else
#error "config. inconsistent: MICROBIT_DISPLAY_TIMER4=y and NRFX_TIMER4=n "
#endif
#endif

static nrfx_timer_t display_timer;

/**
 * most driver variables in a single struct
 */
struct myscreen_struct {
	int current_pixel; /* counter for the refresh */
	volatile uint32_t image_buffer;

	/* animation temporal logic done by submitting work to kernel */
	struct k_work_delayable work;
	int delay; /* ms delay in animation steps */

	bool frame_available; /* whether to continue animation */

	/* animation parameters */
	bool scrolling; /* whether to scroll or jump from frame to frame */
	bool loop; /* whether to repeat the animation until cancelled */
	bool text; /* if images are taken from text buffer */
	uint16_t textlen; /* length of text to be shown */

	/* Animation logic variables */
	int16_t current_image;
	uint16_t frame_number;

	const struct mb_image *img; /* pointer to array of images to show */
	int8_t image_number;

	/* Buffer for printed strings */
	char str_buf[CONFIG_MICROBIT_DISPLAY_STR_MAX];
};

/* unique instance of this struct */
static struct myscreen_struct myscreen;

/** @brief: Shut down the display, stop all activity.
 *
 */

static void mb_display_deactivate(void)
{
	/* stop timer */
	nrfx_timer_compare_int_disable(&display_timer, 0);
	nrfx_timer_disable(&display_timer);
	nrfx_timer_clear(&display_timer);
	nrf_timer_event_t event = nrf_timer_compare_event_get(0);

	nrf_timer_event_clear((display_timer.p_reg), event);

	/* extinct whatever is active */
	nrf_gpio_port_out_clear(NRF_P0, row_mask);
	nrf_gpio_port_out_set(NRF_P0, col_mask0);
#ifdef CONFIG_BOARD_BBC_MICROBIT_V2
	nrf_gpio_port_out_set(NRF_P1, col_mask1);
#endif
	myscreen.current_pixel = (DISPLAY_ROWS * DISPLAY_COLS) -1;
}

/**
 * @brief: The animation logic. This function computes
 * the next image to be shown.
 */

static void prepare_next_frame(void)
{
	myscreen.current_image++;
	if (myscreen.current_image >= myscreen.frame_number) {
		if (myscreen.loop) {
			myscreen.current_image = 0;
		} else {
			myscreen.frame_available = false;
			return;
		}
	}
	const struct mb_image *img1;
	const struct mb_image *img2 = NULL;
	int scroll_offset;
	int index;

	if (myscreen.text) {
		if (myscreen.scrolling) {
			index = myscreen.current_image / SYMBOL_WIDTH;
			scroll_offset = myscreen.current_image - SYMBOL_WIDTH * index;
		} else {
			index = myscreen.current_image;
			scroll_offset = 0;
		}

		char a = myscreen.str_buf[index];
		char b = myscreen.str_buf[index + 1];

		img1 = &mb_font[a - ' '];
		img2 = &mb_font[b - ' '];
	} else {
		if (myscreen.scrolling) {
			index = myscreen.current_image / SYMBOL_WIDTH;
			scroll_offset = myscreen.current_image - SYMBOL_WIDTH * index;
			if (index + 1 < myscreen.image_number) {
				img2 = &myscreen.img[index + 1];
			} else {
				img2 = (myscreen.loop ? &myscreen.img[0] : &mb_font[0]);
			}
		} else {
			index = myscreen.current_image;
			scroll_offset = 0;
		}
		img1 = &myscreen.img[index];
	}

	/* Now draw the new frame as superposition
	 * of two images.
	 */

	uint32_t frame = 0;

	for (int x = 0; x < DISPLAY_COLS; x++) {
		int offset = scroll_offset + x;

		if (offset < DISPLAY_COLS) {
			for (int y = 0; y < DISPLAY_ROWS; y++) {

				if (((img1->row[y]) >> offset) & 1) {
					frame |= 1 << (DISPLAY_ROWS * y + x);
				}
			}
		} else if (offset >= SYMBOL_WIDTH) {
			for (int y = 0; y < DISPLAY_ROWS; y++) {
				if (((img2->row[y]) >> (offset - SYMBOL_WIDTH)) & 1) {
					frame |= 1 << (DISPLAY_ROWS * y + x);
				}
			}
		}
		myscreen.image_buffer = frame;
	}
	myscreen.frame_available = true;
}

static void mb_display_worker(struct k_work *work)
{
	ARG_UNUSED(work);
	if (myscreen.frame_available) {
		/* schedule the next one, then compute a new frame */
		k_work_schedule(&myscreen.work, K_MSEC(myscreen.delay));
		prepare_next_frame();
	} else {
		mb_display_deactivate();
	}
}


/** @brief Pixel refresh interrupt handler.
 *
 * Interrupt handler for periodic interrupt of selected timer.
 * Supposes a compare event with channel 0 of the selected timer.
 * Calls inline functions only, so no function call.
 * On the whole, about 60 machine instructions, probably less
 * than 100 cycles, or 250 000 cycles per second. At 64MHz, this
 * amounts to 0.4% system load.
 *
 **/

ISR_DIRECT_DECLARE(mb_display_refresh_pixel)
{
	/* period event on channel 0 */
	nrf_timer_event_t event = nrf_timer_compare_event_get(0);

	nrf_timer_event_clear((display_timer.p_reg), event);

	/* deactivate active pixel */
	nrf_gpio_port_out_set(NRF_P0, col_mask0);
#ifdef CONFIG_BOARD_BBC_MICROBIT_V2
	nrf_gpio_port_out_set(NRF_P1, col_mask1);
#endif
	/* prepare next pixel */
	myscreen.current_pixel--;
	if (myscreen.current_pixel < 0) {
		myscreen.current_pixel = (DISPLAY_ROWS * DISPLAY_COLS) -1;
	}
	/* activate new pixel */
	if ((myscreen.image_buffer >> myscreen.current_pixel) & 1) {
		int row_col = led_row_column[myscreen.current_pixel];
		uint8_t column = row_col & 0xFF;

		mypinclear(column);
		nrf_gpio_port_out_clear(NRF_P0, row_mask);
		uint8_t row = row_col >> 8;

		mypinset(row);
	}

	return 0;
}

/** @brief: Dummy function to pass to nrfx timer initialization.
 */
static void dummyHandler(nrf_timer_event_t event_type, void *p_context)
{
}

/** @brief initialize the pixel refresh timer.
 *
 */
static void init_display_timer(void)
{
	/* one time timer initialization */
	display_timer = (nrfx_timer_t)NRFX_TIMER_INSTANCE(DISPLAY_TIMER);

	/* Connect Timer IRQ with priority 1, TODO add this to Kconfig */
	IRQ_DIRECT_CONNECT(DISPLAY_TIMER_IRQN, 1, mb_display_refresh_pixel, 0);

	nrfx_timer_config_t tcfg = NRFX_TIMER_DEFAULT_CONFIG;
	/* We program the timer to produce 2500 interrupts per second.
	 * Since the interrupt provides pixel refreshes, this means an image
	 * frequency of 100Hz, which is beyond the eyes perception.
	 *
	 * We achieve 2500Hz by using 62500Hz counting to 25. This means that
	 * 8 bits for counting are sufficient. It also means that it is flicker
	 * free to video cameras that film with 1/50 exposure time.
	 *
	 * TODO: 50Hz image refresh might be considered sufficient and would
	 * half the load the refresh puts on the system. Higher than 100Hz
	 * would be more agreable to the eye.
	 */

	tcfg.bit_width = NRF_TIMER_BIT_WIDTH_8;
	tcfg.frequency = NRF_TIMER_FREQ_62500Hz;
	tcfg.mode = NRF_TIMER_MODE_TIMER;

	/* initialize timer via nrfx API. However, the handler function
	 * is a dummy (required by the API), as we bypass the default nrfx
	 * timer interrupt handler directly declaring our handler to zephyr.
	 */
	nrfx_timer_init(&display_timer, &tcfg, dummyHandler);

	display_timer.p_reg->CC[0] = 62500 / (DISPLAY_ROWS * DISPLAY_COLS * CONFIG_MICROBIT_DISPLAY_REFRESH_RATE_HZ); /* 2500 Hz pixel frequency */
	display_timer.p_reg->SHORTS = NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK;
}


/**
 * Start the display
 */
static void start_animation(int duration)
{
	myscreen.delay = duration;
	myscreen.image_buffer = 0;

	/* start animation */
	myscreen.frame_available = true; /* so we can start work */
	if (duration != -1) {
		k_work_schedule(&myscreen.work, K_MSEC(0));
	} else {
		prepare_next_frame(); /* just once */
	}
	/* activate refresh ISR */
	nrfx_timer_compare_int_enable(&display_timer, 0);
	nrfx_timer_enable(&display_timer);
}



/********************** initialisation *********************************/

/**
 * the one time initialization function to be called by SYS_INIT
 */

static int init_driver(const struct device *dev);
SYS_INIT(init_driver, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

static int init_driver(const struct device *dev)
{
	ARG_UNUSED(dev); /* it is here to match prototype */

	LOG_DBG("one time initialization");

	k_work_init_delayable(&myscreen.work, mb_display_worker);

	init_display_timer();

	mb_display_deactivate(); /* initialize and deactivate */

	for (int i = 0; i < DISPLAY_ROWS * DISPLAY_ROWS; i++) {
		/* for uniformity between microbit v1 and v2, we accept to initialize
		 * some gpio pins several times.
		 */
		uint16_t col_row = led_row_column[i];
		int col = col_row >> 8;
		int row = col_row & 0xFF;
		/* column configuration */
		nrf_gpio_cfg(row, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			 NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);

		/* row configuration */
		nrf_gpio_cfg(col, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
			 NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_D0H1, NRF_GPIO_PIN_NOSENSE);
	}
	return 0;
}

/****************** implementation of mb_display.h API functions ************/

void mb_display_image_v2(uint32_t mode, int32_t duration,
		const struct mb_image *img, uint8_t img_count)
{
	mb_display_deactivate();
	myscreen.text = false;
	/* default mode is single pictures */
	myscreen.scrolling = (mode & MB_DISPLAY_MODE_SCROLL) != 0;
	myscreen.loop = mode & MB_DISPLAY_FLAG_LOOP;

	myscreen.img = img;
	myscreen.image_number = img_count;
	myscreen.frame_number =
		(myscreen.scrolling ? SYMBOL_WIDTH * myscreen.image_number : myscreen.image_number);
	myscreen.current_image = -1;
	/* The next image to be shown
	 * prepare empty frame, later add a frame each time we switch.
	 */
	start_animation(duration);
}

void mb_display_print_v2(uint32_t mode, int32_t duration, const char *fmt,
		...)
{
	va_list ap;

	va_start(ap, fmt);

	mb_display_deactivate();
	myscreen.scrolling = (mode & MB_DISPLAY_MODE_SINGLE) == 0;
	/* default is scrolling */
	myscreen.str_buf[0] = ' ';
	char *str;

	str = (myscreen.scrolling ? &myscreen.str_buf[1] : myscreen.str_buf);
	vsnprintk(str, sizeof(myscreen.str_buf) - 4, fmt, ap);
	va_end(ap);
	if (str[0] == '\0') {
		return;
	}
	myscreen.textlen = strnlen(myscreen.str_buf, sizeof(myscreen.str_buf) - 3);
	if (myscreen.scrolling) {
		myscreen.str_buf[myscreen.textlen] = ' ';
		myscreen.str_buf[myscreen.textlen + 1] = ' ';
		myscreen.textlen += 1; /* two spaces added, but only one is "shown" */
	}

	myscreen.loop = (mode & MB_DISPLAY_FLAG_LOOP) != 0;
	myscreen.frame_number =
		(myscreen.scrolling ? (myscreen.textlen - 1) * SYMBOL_WIDTH : myscreen.textlen);
	myscreen.current_image = -1;
	myscreen.text = true;
	start_animation(duration);
}

void mb_display_stop_v2(void)
{
	mb_display_deactivate();
}
