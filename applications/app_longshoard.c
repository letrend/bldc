#include "packet.h"
#include "ch.h" // ChibiOS
#include "commands.h" // terminal printf
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
 
#include <string.h>

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define SERIAL_RX_BUFFER_SIZE		1024

// Threads
static THD_FUNCTION(packet_process_thread, arg);
static THD_WORKING_AREA(packet_process_thread_wa, 4096);
static thread_t *process_tp;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static int is_running = 0;

// Private functions
static void process_packet(unsigned char *data, unsigned int len);
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void send_packet(unsigned char *data, unsigned int len);

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chSysLockFromISR();
	chEvtSignalI(process_tp, (eventmask_t) 1);
	chSysUnlockFromISR();
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

static void process_packet(unsigned char *data, unsigned int len) {
	commands_set_send_func(send_packet_wrapper);
	commands_process_packet(data, len);
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_HANDLER);
}

static void send_packet(unsigned char *data, unsigned int len) {
	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);

	uartStartSend(&HW_UART_DEV, len, buffer);
}

void app_uartcomm_start(void) {
	packet_init(send_packet, process_packet, PACKET_HANDLER);

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	is_running = 1;

	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa), NORMALPRIO, packet_process_thread, NULL);
}

void app_uartcomm_configure(uint32_t baudrate) {
	uart_cfg.speed = baudrate;

	if (is_running) {
		uartStart(&HW_UART_DEV, &uart_cfg);
	}
}

static THD_FUNCTION(packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("uartcomm process");

	process_tp = chThdGetSelfX();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], PACKET_HANDLER);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}

// longshoard thread
static THD_FUNCTION(longshoard_thread, arg);
static THD_WORKING_AREA(longshoard_thread_wa, 2048); // 2kb stack for this thread
 
void app_longshoard_init(void) {
	// Start the longshoard thread
	chThdCreateStatic(longshoard_thread_wa, sizeof(longshoard_thread_wa),
		NORMALPRIO, longshoard_thread, NULL);
}
 
static THD_FUNCTION(longshoard_thread, arg) {
	(void)arg;
 
	chRegSetThreadName("APP_LONGSHOARD");
 
	for(;;) {
		// Read the pot value and scale it to a number between 0 and 1 (see hw_46.h)
		float pot = (float)ADC_Value[ADC_IND_EXT];
		pot /= 4095.0;
 		
	 	send_packet((unsigned char*)&pot, 4);
		commands_printf("%f\n",pot);		 

		chThdSleepMilliseconds(1000);
 
		// Reset the timeout
		timeout_reset();
	}
}
