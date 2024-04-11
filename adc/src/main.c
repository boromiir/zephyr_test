/*
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/random/random.h>
#include <arm_math.h>
#include <stm32f4xx.h>

#define FFT_SAMPLES		512
#define FFT_SIZE		(FFT_SAMPLES / 2)

#define PIN_THREADS (IS_ENABLED(CONFIG_SMP) && IS_ENABLED(CONFIG_SCHED_CPU_MASK))

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7
#define PRIORITY5 5

/* delay between greetings (in ms) */
#define SLEEPTIME 1

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

/* Data of ADC io-channels specified in devicetree. */
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
			     DT_SPEC_AND_COMMA)
};

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define FFT_PIN	  DT_ALIAS(pin0)


K_QUEUE_DEFINE(FFT_queue);

struct k_queue FFT_queue;
struct k_msgq FFT_msgq;

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct gpio_dt_spec pin = GPIO_DT_SPEC_GET(FFT_PIN, gpios);

float32_t FFT_input[FFT_SAMPLES] = {0};
float32_t FFT_output[FFT_SAMPLES];
float32_t freqTable[FFT_SIZE];
float32_t freqOrder[FFT_SIZE];
arm_rfft_fast_instance_f32 FFTHandler;

float32_t baseFreq;

float32_t msgq_buffer;

void FFT_init(float32_t samplingFreq)
{
	for(uint16_t i = 0; i < FFT_SIZE; i++)
	{
		freqOrder[i] = i * samplingFreq / FFT_SAMPLES;
	}

	sys_rand_get(FFT_input, FFT_SAMPLES);
}

void ADC_task_entry(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int err;
	uint32_t count = 0;
	uint16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(buf),
	};

	float32_t msgq_rec;

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
		}
	}

	k_queue_init(&FFT_queue);
	k_msgq_init(&FFT_msgq, &msgq_buffer, sizeof(float32_t), 1);

	while(1)
	{
		//k_queue_get(&FFT_queue, K_FOREVER);
		k_msgq_get(&FFT_msgq, &msgq_rec, K_FOREVER);

		// printk("ADC reading[%u]:\n", count++);
		// for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		// 	int32_t val_mv;

		// 	printk("- %s, channel %d: ",
		// 	       adc_channels[i].dev->name,
		// 	       adc_channels[i].channel_id);

		// 	(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

		// 	err = adc_read_dt(&adc_channels[i], &sequence);
		// 	if (err < 0) {
		// 		printk("Could not read (%d)\n", err);
		// 		continue;
		// 	}

		// 	/*
		// 	 * If using differential mode, the 16 bit value
		// 	 * in the ADC sample buffer should be a signed 2's
		// 	 * complement value.
		// 	 */
		// 	if (adc_channels[i].channel_cfg.differential) {
		// 		val_mv = (int32_t)((int16_t)buf);
		// 	} else {
		// 		val_mv = (int32_t)buf;
		// 	}
		// 	printk("%"PRId32, val_mv);
		// 	err = adc_raw_to_millivolts_dt(&adc_channels[i],
		// 				       &val_mv);
		// 	/* conversion to mV may not be supported, skip if not */
		// 	if (err < 0) {
		// 		printk(" (value in mV not available)\n");
		// 	} else {
		// 		printk(" = %"PRId32" mV\n", val_mv);
		// 	}
		// }

	}
}
K_THREAD_DEFINE(ADC_task, STACKSIZE,
				ADC_task_entry, NULL, NULL, NULL,
				PRIORITY5, 0, 0);
extern const k_tid_t ADC_task;

void FFT_task_entry(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	int32_t received_from_ADC;
	uint32_t maxFFTValueIndex = 0;
	float32_t maxFFTValue = 0;
	uint16_t freqBufferIndex = 0;
	float32_t f_s = 25000000;

	FFT_init(f_s);
	arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

	while(1)
	{
		gpio_pin_set_dt(&pin, 1);
		//SEGGER_SYSVIEW_MarkStart(0);
		arm_rfft_fast_f32(&FFTHandler, FFT_input, FFT_output, 0);
		arm_cmplx_mag_f32(FFT_output, freqTable, FFT_SIZE);
		arm_max_f32(freqTable + 1, FFT_SIZE - 1, &maxFFTValue, &maxFFTValueIndex);

		baseFreq = freqOrder[maxFFTValueIndex];


		//gpio_pin_toggle_dt(&led);
		gpio_pin_set_dt(&pin, 0);
		//SEGGER_SYSVIEW_MarkStop(0);
		//k_queue_append(&FFT_queue, &baseFreq);
		k_msgq_put(&FFT_msgq, &baseFreq, K_FOREVER);
	}
}
K_THREAD_DEFINE(FFT_task, STACKSIZE,
				FFT_task_entry, NULL, NULL, NULL,
				PRIORITY, 0, 0);
extern const k_tid_t FFT_task;

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	if (!gpio_is_ready_dt(&pin)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	// k_queue_init(&FFT_queue);

	return 0;
}
