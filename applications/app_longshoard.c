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

#define pack754_32(f) (pack754((f), 32, 8))
#define pack754_64(f) (pack754((f), 64, 11))
#define unpack754_32(i) (unpack754((i), 32, 8))
#define unpack754_64(i) (unpack754((i), 64, 11))

// longshoard thread
static THD_FUNCTION(longshoard_thread, arg);
static THD_WORKING_AREA(longshoard_thread_wa, 2048); // 2kb stack for this thread

static THD_FUNCTION(longshoard_packet_process_thread, arg);
static THD_WORKING_AREA(longshoard_packet_process_thread_wa, 4096);
static thread_t *longshoard_process_tp;

// Variables
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static int is_running = 0;

float duty_command = 0, brake_current_command = 0;
bool brake_now, release_motor;

uint64_t pack754(long double f, unsigned bits, unsigned expbits)
{
    long double fnorm;
    int shift;
    long long sign, exp, significand;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (f == 0.0) return 0; // get this special case out of the way

    // check sign and begin normalization
    if (f < 0) { sign = 1; fnorm = -f; }
    else { sign = 0; fnorm = f; }

    // get the normalized form of f and track the exponent
    shift = 0;
    while(fnorm >= 2.0) { fnorm /= 2.0; shift++; }
    while(fnorm < 1.0) { fnorm *= 2.0; shift--; }
    fnorm = fnorm - 1.0;

    // calculate the binary form (non-float) of the significand data
    significand = fnorm * ((1LL<<significandbits) + 0.5f);

    // get the biased exponent
    exp = shift + ((1<<(expbits-1)) - 1); // shift + bias

    // return the final answer
    return (sign<<(bits-1)) | (exp<<(bits-expbits-1)) | significand;
}

long double unpack754(uint64_t i, unsigned bits, unsigned expbits)
{
    long double result;
    long long shift;
    unsigned bias;
    unsigned significandbits = bits - expbits - 1; // -1 for sign bit

    if (i == 0) return 0.0;

    // pull the significand
    result = (i&((1LL<<significandbits)-1)); // mask
    result /= (1LL<<significandbits); // convert back to float
    result += 1.0f; // add the one back on

    // deal with the exponent
    bias = (1<<(expbits-1)) - 1;
    shift = ((i>>significandbits)&((1LL<<expbits)-1)) - bias;
    while(shift > 0) { result *= 2.0; shift--; }
    while(shift < 0) { result /= 2.0; shift++; }

    // sign it
    result *= (i>>(bits-1))&1? -1.0: 1.0;

    return result;
}

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
	chEvtSignalI(longshoard_process_tp, (eventmask_t) 1);
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

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	packet_send_packet(data, len, PACKET_HANDLER);
}

static void longshoard_commands_process_packet(unsigned char *data, unsigned int len){
  if (!len) {
		return;
	}
  int packet_id = data[0];
  if(packet_id == 66){// this is a longshoard package
    uint32_t fi[2];
    fi[0] =(uint32_t) (data[4] << 24| data[3] << 16 |data[2] << 8| data[1]);
    fi[1] =(uint32_t) (data[8] << 24| data[7] << 16 |data[6] << 8| data[5]);
    duty_command = 		unpack754_32(fi[1]);
    brake_current_command = unpack754_32(fi[0]);
    brake_now = data[9];
    release_motor = data[10];
  }else{ // this is a vesc package
    commands_process_packet(data, len);
  }
}

static void process_longshoard_packet(unsigned char *data, unsigned int len) {
	commands_set_send_func(send_packet_wrapper);
  longshoard_commands_process_packet(data, len);
}

void app_longshoard_init(void) {
  packet_init(send_packet, process_longshoard_packet, PACKET_HANDLER);

  uartStart(&HW_UART_DEV, &uart_cfg);
  palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUDR_PULLUP);
  palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
      PAL_STM32_OSPEED_HIGHEST |
      PAL_STM32_PUDR_PULLUP);

  uart_cfg.speed = BAUDRATE;
	uartStart(&HW_UART_DEV, &uart_cfg);

	// Start the longshoard thread
	chThdCreateStatic(longshoard_thread_wa, sizeof(longshoard_thread_wa),
		NORMALPRIO, longshoard_thread, NULL);

  chThdCreateStatic(longshoard_packet_process_thread_wa,
    sizeof(longshoard_packet_process_thread_wa),
    NORMALPRIO,
    longshoard_packet_process_thread, NULL);
}

static THD_FUNCTION(longshoard_packet_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_LONGSHOARD_UART");

	longshoard_process_tp = chThdGetSelfX();

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

static THD_FUNCTION(longshoard_thread, arg) {
	(void)arg;

	chRegSetThreadName("APP_LONGSHOARD");

	for(;;) {
    // Read the pot value and scale it to a number between 0 and 1 (see hw_46.h)
		float pot = (float)ADC_Value[ADC_IND_EXT];
		pot /= 4095.0;

		float values[5];

		values[0] = mc_interface_get_rpm();
		values[1] = mc_interface_read_reset_avg_motor_current();
		values[2] = mc_interface_read_reset_avg_input_current();
		values[3] = GET_INPUT_VOLTAGE();
		values[4] = pot;

		uint32_t fi[5];
		fi[0] = pack754_32(values[0]);
		fi[1] = pack754_32(values[1]);
		fi[2] = pack754_32(values[2]);
		fi[3] = pack754_32(values[3]);
		fi[4] = pack754_32(values[4]);

    if(brake_now == 1){
      mc_interface_set_brake_current(brake_current_command);
      mc_interface_brake_now();
      mc_interface_set_duty(0);
    }else if(release_motor == 1){
      mc_interface_release_motor();
      if(duty_command>0){
        mc_interface_set_duty(duty_command);
      }
    }

	 	packet_send_packet((unsigned char*)&fi, sizeof(uint32_t)*5, PACKET_HANDLER);

		commands_printf("%f\n",pot);

		chThdSleepMilliseconds(50);

		// Reset the timeout
		timeout_reset();
	}
}
