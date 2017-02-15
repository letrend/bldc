#include "packet.h"
#include "ch.h" // ChibiOS
#include "commands.h" // terminal printf
#include "hal.h" // ChibiOS HAL
#include "mc_interface.h" // Motor control functions
#include "hw.h" // Pin mapping on this hardware
#include "timeout.h" // To reset the timeout
#include "crc.h"
#include "digital_filter.h"
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

enum LONGSHOARD_COMMAND{
    BRAKE = 40,
    RELEASE = 41,
    MODE = 42,
    MANUAL_CONTROL = 43,
    SET_BRAKE_CURRENT = 44,
    SET_DUTY = 45,
    SET_MIN_WEIGHT = 46,
    SET_MAX_WEIGHT = 47,
    RPM_DATA = 48,
    WEIGHT_DATA = 49,
    CURRENT_DATA = 50,
    END_DATA = 51,
    START_DATA = 52
};

#define CIRCULAR_BUFFER_LENGTH 256

typedef struct _CircularBuffert{
    uint8_t     mWrite;
    float       mBuffer[CIRCULAR_BUFFER_LENGTH];
}CircularBuffert;

#define CIRCBUF_write(cirbuf, data, size)    {                                 \
    uint8_t temp = ((cirbuf.mWrite +1 )& (size -1));                           \
    cirbuf.mBuffer[temp] = data;                                                 \
    cirbuf.mWrite = temp;                                                        \
}

float duty = 0, brake_current = 0, min_weight = 0.3f, max_weight = 0.1f;
bool brake_now = false, release_motor = true;
bool mode[4];
unsigned int data_point = 0;
unsigned short checksum = 0;
CircularBuffert rpm_measure, weight_measure, current_measure;
static float rpm_data[CIRCULAR_BUFFER_LENGTH], weight_data[CIRCULAR_BUFFER_LENGTH], current_data[CIRCULAR_BUFFER_LENGTH];
float std_rpm = 0, std_weight = 0, std_current = 0;
bool data_valid = false, receiving_data = false;
uint32_t total_number_of_data_points = 0;
bool manual_control = true;

float cross_correlate(CircularBuffert* signal, float* filter, float std_filter, float signal_mean){
  int j = signal->mWrite-1;
  float response = 0, std = 0;
  for(int i=0; i<total_number_of_data_points; i++){
      std+=(signal->mBuffer[j]-signal_mean)*(signal->mBuffer[j]-signal_mean);
      j--;
      if(j<0){
        j = CIRCULAR_BUFFER_LENGTH-1;
      }
  }
  std = sqrtf(std);
  j = signal->mWrite-1;
  for(int i=total_number_of_data_points-1; i>=0; i--){
    response += (filter[i]*(signal->mBuffer[j]-signal_mean));
    j--;
    if(j<0){
      j = CIRCULAR_BUFFER_LENGTH-1;
    }
  }
  return response/(std*std_filter);
}

float normalize(float *filter, int len){
  float mean = 0, std = 0;
  for(int i=0; i<len; i++){
    mean += filter[i];
  }
  mean/=len;
  for(int i=0; i<len; i++){
    std+=(filter[i]-mean)*(filter[i]-mean);
    filter[i]-=mean;
  }
  return sqrtf(std);
}

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

bool checkIntegrityAndMerge(float *des, unsigned char* data, unsigned int len ){
  checksum = crc16(&data[4], len-4);
  unsigned short frame_checksum = (unsigned short) ((data[2] << 8) | data[1]);
  bool valid = false;
  if(checksum==frame_checksum){ // check if data is not corrupted
    valid = true;
    unsigned int i = 0;
    int number_of_samples = data[3];
    for(i = 4; i<number_of_samples*sizeof(uint32_t); i+=sizeof(uint32_t)){
      uint32_t fi = (uint32_t) (data[i+3] << 24| data[i+2] << 16 |data[i+1] << 8| data[i]);
      des[data_point++] = unpack754_32(fi);
    }
  }
  return valid;
}

static void longshoard_commands_process_packet(unsigned char *data, unsigned int len){
  if (!len) {
		return;
	}
  int packet_id = data[0];
  if(packet_id >= 40){// this is a longshoard package
    switch(packet_id){
      case BRAKE:
        mc_interface_brake_now();
        break;
      case RELEASE:
        mc_interface_release_motor();
        break;
      case SET_BRAKE_CURRENT:{
        uint32_t fi = (uint32_t) (data[4] << 24| data[3] << 16 |data[2] << 8| data[1]);
        brake_current = unpack754_32(fi);
        mc_interface_set_brake_current(brake_current);
        break;
      }
      case SET_DUTY:{
        uint32_t fi = (uint32_t) (data[4] << 24| data[3] << 16 |data[2] << 8| data[1]);
        duty = unpack754_32(fi);
        mc_interface_set_duty(duty);
        break;
      }
      case SET_MIN_WEIGHT:{
        uint32_t fi = (uint32_t) (data[4] << 24| data[3] << 16 |data[2] << 8| data[1]);
        min_weight = unpack754_32(fi);
        break;
      }
      case SET_MAX_WEIGHT:{
        uint32_t fi = (uint32_t) (data[4] << 24| data[3] << 16 |data[2] << 8| data[1]);
        max_weight = unpack754_32(fi);
        break;
      }
      case MODE:{
        mode[data[1]] = data[2];
        break;
      }
      case MANUAL_CONTROL:{
        manual_control = !manual_control;
        break;
      }
      case RPM_DATA:{
        receiving_data = true;
        data_valid = checkIntegrityAndMerge( rpm_data, data, len );
        break;
      }
      case WEIGHT_DATA:{
        receiving_data = true;
        data_valid = checkIntegrityAndMerge( weight_data, data, len );
        break;
      }
      case CURRENT_DATA:{
        receiving_data = true;
        data_valid = checkIntegrityAndMerge( current_data, data, len );
        break;
      }
      case START_DATA:{
        receiving_data = true;
        data_point = 0;
        break;
      }
      case END_DATA:{
        receiving_data = false;
        total_number_of_data_points = (uint32_t) (data[5] << 24| data[4] << 16 |data[3] << 8| data[2]);
        if(data_point==total_number_of_data_points && data_valid){ // if data valid so far and the number of data points is correct
          data_valid = true;
          if(data[1]==RPM_DATA){
            std_rpm = normalize(rpm_data,total_number_of_data_points);
          }else if(data[1]==WEIGHT_DATA){
            std_weight = normalize(weight_data,total_number_of_data_points);
          }else if(data[1]==CURRENT_DATA){
            std_current = normalize(current_data,total_number_of_data_points);
          }
        }else{
          data_valid = false;
        }
        break;
      }
    }
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

  uint32_t count = 0;
  rpm_measure.mWrite = 0;
  current_measure.mWrite = 0;
  weight_measure.mWrite = 0;

  float rpm_mean = 0, weight_mean = 0, current_mean = 0;

  int iter = 1;

	for(;;) {
    // Read the pot value and scale it to a number between 0 and 1 (see hw_46.h)
		float pot = (float)ADC_Value[ADC_IND_EXT];
		pot /= 4095.0;

    float values[5];

    values[0] = mc_interface_get_rpm();
    values[1] = mc_interface_read_reset_avg_motor_current();
    values[2] = GET_INPUT_VOLTAGE();
    values[3] = pot;

    if(!manual_control){
      if(mode[0]){
        values[4] = cross_correlate(&rpm_measure,rpm_data,std_rpm,rpm_mean);
      }else if(mode[1]){
        values[4] = cross_correlate(&weight_measure,weight_data,std_weight,weight_mean);
      }else if(mode[2]){
        values[4] = cross_correlate(&current_measure,current_data,std_current,current_mean);
      }
    }else{
      values[4] = 0;
    }

    rpm_mean = (rpm_mean + values[0])/(float)iter;
    weight_mean = (weight_mean + values[3])/(float)iter;
    current_mean = (current_mean + values[1])/(float)iter;

    CIRCBUF_write(rpm_measure,values[0],CIRCULAR_BUFFER_LENGTH);
    CIRCBUF_write(current_measure,values[1],CIRCULAR_BUFFER_LENGTH);
    CIRCBUF_write(weight_measure,values[3],CIRCULAR_BUFFER_LENGTH);

		uint32_t fi[6];
		fi[0] = (receiving_data | data_valid << 1 | manual_control << 2 |
      mode[0] << 3 | mode[1] << 4 | mode[2] << 5 | mode[3] << 6);
		fi[1] = pack754_32(values[0]);
		fi[2] = pack754_32(values[1]);
		fi[3] = pack754_32(values[2]);
		fi[4] = pack754_32(values[3]);
    fi[5] = pack754_32(values[4]);

	 	packet_send_packet((unsigned char*)&fi, sizeof(uint32_t)*6, PACKET_HANDLER);

    // commands_printf("%d\n",total_number_of_data_points);

		chThdSleepMilliseconds(30);

		// Reset the timeout
		timeout_reset();

    iter++;
	}
}
