#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/twai.h"

static const int uart_rs485_num = UART_NUM_1;
#define UART_RX_BUF_SIZE 127
static uint8_t uart_rx_buffer[UART_RX_BUF_SIZE];
static uint8_t uart_log_buffer[UART_RX_BUF_SIZE];
static const int uart_log_num = UART_NUM_0;

#define PIN_485_TX 4
#define PIN_485_RX 3

#define PIN_CAN_TX 5
#define PIN_CAN_RX 8

#define PIN_LOG_TX 6
#define PIN_LOG_RX 7

#define RS485_ID_BMS_BOARD 20
#define VERSION_CODE_BMS_BOARD 201
#define TAG             "BMS"

#define BMS_LOG_BUFFER_SIZE 98
struct RS485_REGISTERS {
	uint16_t VMS_version_code; //0
	uint16_t VMS_ble_status; //1
	uint8_t VMS_ble_name[10]; //2
	uint16_t VMS_CAN_bus_scan_status; //7
	uint16_t VMS_CAN_bus_scan_timeout; //8
	uint8_t VMS_CAN_ID_241_bytes[8]; //9
	uint16_t VMS_ble_send_begin; //13
	uint8_t VMS_ble_tx_bytes[20]; //14
	uint8_t VMS_CAN_ID_242_bytes[8]; //24
	uint8_t VMS_ble_rx_bytes[32]; //28
	uint16_t VMS_dummy[20]; //44

	uint16_t BMS_version_code; //64
	uint16_t BMS_log_read_flag; //65
	uint16_t BMS_CAN_bus_scan_falg; //66
	uint16_t BMS_CAN_bus_scan_timeout; //67
	uint16_t BMS_CAN_bus_ID_received; //68

	uint16_t BMS_log_buffer[BMS_LOG_BUFFER_SIZE/2]; //69
};

//struct BMS_LOG_Structure {
//  uint16_t id;//69
//  uint16_t v_bplus;//70
//  int16_t ts[4];//71
//  uint16_t v_raw;//75
//  uint16_t v_bias;//76
//  uint16_t v_pack;//77
//  uint16_t v_chg;//78
//  uint16_t v_12v;//79
//  uint16_t v_cell[14];80
//  uint16_t charge_trips[8];
//  uint16_t discharge_trips[8];
//  uint8_t current_float_string[16];
//  //char myLetter;
//};

volatile static uint16_t BMS_CAN_bus_scan_status; //67

static struct RS485_REGISTERS rs485_regs;
//static struct BMS_LOG_Structure bms_log_values;
static uint16_t MODBUS_CRC16_V1(const uint8_t *buf, uint16_t len);
static bool MODBUS_CRC_pass(uint8_t *data, uint16_t data_len);

void hw_config() {
	uart_config_t uart_config_rs485 = { .baud_rate = 19200, .data_bits =
			UART_DATA_8_BITS, .parity = UART_PARITY_EVEN, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 122 };
	if (uart_driver_install(uart_rs485_num, UART_RX_BUF_SIZE * 2, 0, 0, NULL,
			0)== ESP_OK) {
		ESP_LOGI(TAG, "RS485 uart_driver_install OK");

		if (uart_param_config(uart_rs485_num, &uart_config_rs485) == ESP_OK) {
			ESP_LOGI(TAG, "RS485 uart_param_config OK");
			//uart_set_pin(uart_port_t uart_rs485_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num)
			uart_set_pin(uart_rs485_num, PIN_485_TX, PIN_485_RX, -1, -1);
			uart_set_rx_timeout(uart_rs485_num, 5);
			if (uart_set_mode(uart_rs485_num,
					UART_MODE_RS485_HALF_DUPLEX) == ESP_OK) {
				ESP_LOGI(TAG, "RS485 uart_set_mode OK");
			}

		}
	}

	uart_config_t uart_config_log = { .baud_rate = 115200, .data_bits =
			UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits =
			UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
			.rx_flow_ctrl_thresh = 122 };
	if (uart_driver_install(uart_log_num, UART_RX_BUF_SIZE * 2, 0, 0, NULL,
			0)== ESP_OK) {
		ESP_LOGI(TAG, "log uart_driver_install OK");

		if (uart_param_config(uart_log_num, &uart_config_log) == ESP_OK) {
			ESP_LOGI(TAG, "log uart_param_config OK");
			//uart_set_pin(uart_port_t uart_rs485_num, int tx_io_num, int rx_io_num, int rts_io_num, int cts_io_num)
			uart_set_pin(uart_log_num, PIN_LOG_TX, PIN_LOG_RX, -1, -1);
			uart_set_rx_timeout(uart_log_num, 5);
			if (uart_set_mode(uart_log_num,
					UART_MODE_RS485_HALF_DUPLEX) == ESP_OK) {
				ESP_LOGI(TAG, "log uart_set_mode OK");
			}

		}
	}

	//TWAI_GENERAL_CONFIG_DEFAULT(tx_io_num, rx_io_num, op_mode)
	twai_general_config_t g_config =
	TWAI_GENERAL_CONFIG_DEFAULT(5, 8, TWAI_MODE_NORMAL);
	twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
	twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

	//accept id 0x100
//	twai_filter_config_t f_config = { .acceptance_code = (0x241 << 21),
//			.acceptance_mask = ~(0x7FC << 21), .single_filter = true };

	//Install TWAI driver
	if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
		ESP_LOGI(TAG, "CAN bus twai_driver_install OK");
		//Start TWAI driver
		if (twai_start() == ESP_OK) {
			ESP_LOGI(TAG, "CAN bus twai_start OK");
		} else {
			ESP_LOGE(TAG, "CAN bus to start driver");
			return;
		}
	} else {
		ESP_LOGE(TAG, "CAN bus failed to install driver");
	}
}
static void oneshot_timer_callback(void *arg) {
	BMS_CAN_bus_scan_status = 0;
	ESP_LOGE(TAG, "oneshot_timer_callback %d", BMS_CAN_bus_scan_status);
}
void can_scan_timer_start(uint16_t timeout_ms) {

	BMS_CAN_bus_scan_status = 1;
	const esp_timer_create_args_t oneshot_timer_args = { .callback =
			&oneshot_timer_callback,
	/* argument specified here will be passed to timer callback function */
	.arg = NULL, .name = "one-shot" };
	esp_timer_handle_t oneshot_timer;
	esp_timer_create(&oneshot_timer_args, &oneshot_timer);
	esp_timer_start_once(oneshot_timer, timeout_ms * 1000);

}
void can_loop() {
	twai_message_t message;
	if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
		ESP_LOGI(TAG, "Message received");
		if (!(message.rtr)) {
			if (message.identifier == 0x044 && message.data_length_code == 8) {
				rs485_regs.BMS_CAN_bus_ID_received = 1;
			}
		}

	} else {
		//ESP_LOGE(TAG, "Failed to receive message");
		return;
	}

}
void app_main(void) {
	//can_scan_timer_start(2000);
	hw_config();

	memset(&rs485_regs, 0, sizeof(rs485_regs));
	rs485_regs.BMS_version_code = VERSION_CODE_BMS_BOARD;
	//ESP_LOGI(TAG, "bms_log_values size=%d", sizeof(bms_log_values));

	while (true) {

		int uart_rx_len = uart_read_bytes(uart_rs485_num, uart_rx_buffer,
		UART_RX_BUF_SIZE, 1);

		//Write data back to UART
		if (uart_rx_len > 4) {
			ESP_LOGI(TAG, "uart received len = %d, id=%d", uart_rx_len,
					uart_rx_buffer[0]);

			if (MODBUS_CRC_pass(uart_rx_buffer, uart_rx_len)) {

				if (uart_rx_buffer[0] == RS485_ID_BMS_BOARD) {
					ESP_LOGI(TAG, "RS485 len=%d", uart_rx_len);
					if (uart_rx_buffer[1] == 0x03 && uart_rx_len == 8) {
						uint16_t reg_len = (uart_rx_buffer[4] << 8)
								+ uart_rx_buffer[5];
						uint16_t reg_addr = (uart_rx_buffer[2] << 8)
								+ uart_rx_buffer[3];
						ESP_LOGI(TAG, "FC03 read reg addr = %d, len=%d",
								reg_addr, reg_len);
						if (reg_len > 0
								&& reg_addr * 2 + reg_len * 2
										<= sizeof(rs485_regs)) {
							uint8_t msg[5 + reg_len * 2];
							memset(msg, 0, 5 + reg_len * 2);
							msg[0] = uart_rx_buffer[0];
							msg[1] = 0x03;
							msg[2] = (uint8_t) (reg_len * 2);

							//memcpy(&msg[3], (&(rs485_regs.version_code)) + reg_addr, reg_len * 2);
							for (uint16_t i = 0; i < reg_len; i++) {
								uint16_t *a = (uint16_t*) (&(rs485_regs))
										+ reg_addr + i;
								//memcpy(&msg[3+i*2], a, 2);
								msg[3 + i * 2] = (uint8_t) (*a / 256);
								msg[4 + i * 2] = (uint8_t) (*a % 256);
							}
							// uint8_t msg[] = {uart_rx_buffer[0], 0x03, 0x06, 0xAE, 0x41, 0x56, 0x52, 0x43, 0x40, 0x49, 0xAD};
							uint16_t crc16 = MODBUS_CRC16_V1(msg,
									3 + reg_len * 2);
							msg[3 + reg_len * 2] = (uint8_t) (crc16 & 0xFF);
							msg[4 + reg_len * 2] = (uint8_t) (crc16 / 256);
							if (uart_write_bytes(uart_rs485_num, msg,
									5 + reg_len * 2) != 5 + reg_len * 2) {
								ESP_LOGE(TAG,
										"RS485 Send data critical failure.");
							}
						}
					} else if (uart_rx_buffer[1] == 0x10 && uart_rx_len > 9) {
						uint16_t reg_addr = (uart_rx_buffer[2] << 8)
								+ uart_rx_buffer[3];
						uint16_t reg_len = (uart_rx_buffer[4] << 8)
								+ uart_rx_buffer[5];
						ESP_LOGI(TAG, "FC16 write reg addr = %d, len=%d",
								reg_addr, reg_len);
						if (reg_len > 0
								&& reg_addr * 2 + reg_len * 2
										<= sizeof(rs485_regs)) {
							for (uint16_t i = 0; i < reg_len; i++) {

								uint16_t a = (uart_rx_buffer[7 + i * 2] << 8)
										+ uart_rx_buffer[8 + i * 2];
								uint16_t *addr = (uint16_t*) (&rs485_regs)
										+ reg_addr + i;
								*addr = a;
								//memcpy(&msg[3+i*2], a, 2);
								//msg[3+i*2] = (uint8_t)(*a/256);
								//msg[4+i*2] = (uint8_t)(*a%256);
							}
						}
						uint8_t msg[8];
						memset(msg, 0, 8);
						memcpy(msg, uart_rx_buffer, 6);
						uint16_t crc16 = MODBUS_CRC16_V1(msg, 6);
						msg[6] = (uint8_t) (crc16 & 0xFF);
						msg[7] = (uint8_t) (crc16 / 256);

						if (uart_write_bytes(uart_rs485_num, msg, 8) != 8) {
							ESP_LOGE(TAG, "Send data critical failure.");
						}
					}
				}
			} else {
				ESP_LOGE(TAG, "uart bytes CRC illegal, len=%d", uart_rx_len);
			}
		} else if (uart_rx_len > 0) {
			ESP_LOGE(TAG, "uart bytes two few, len=%d", uart_rx_len);
		} else {
			if (rs485_regs.BMS_log_read_flag > 0) {
				ESP_LOGI(TAG, "log read start");
				rs485_regs.BMS_log_read_flag = 0;
				memset(&rs485_regs.BMS_log_buffer, 0, BMS_LOG_BUFFER_SIZE);
				uint8_t byte = 'R';
				if (uart_write_bytes(uart_log_num, &byte, 1) != 1) {
					ESP_LOGE(TAG, "send log read failure.");
				}
				int uart_log_len = uart_read_bytes(uart_log_num, uart_log_buffer,
						UART_RX_BUF_SIZE, 100);

						//Write data back to UART
				if (uart_log_len > 0) {
					ESP_LOGI(TAG, "read log len = %d, %02X,%02X,", uart_log_len, uart_log_buffer[0], uart_log_buffer[1]);
					if(uart_log_buffer[0]==0x34 && uart_log_buffer[1]==0x12 && uart_log_len >= BMS_LOG_BUFFER_SIZE+2){
						if(MODBUS_CRC_pass(uart_log_buffer, BMS_LOG_BUFFER_SIZE+2)){
							ESP_LOGI(TAG, "read log CRC passed");
							memcpy(&rs485_regs.BMS_log_buffer, uart_log_buffer, BMS_LOG_BUFFER_SIZE);
						}



					}
				}

			}
			if (rs485_regs.BMS_CAN_bus_scan_falg > 0) {
				ESP_LOGI(TAG, "CAN bus scan start");
				rs485_regs.BMS_CAN_bus_scan_falg = 0;

				rs485_regs.BMS_CAN_bus_ID_received = 0;
				can_scan_timer_start(rs485_regs.BMS_CAN_bus_scan_timeout);

			}
			if (BMS_CAN_bus_scan_status > 0) {
				can_loop();
			}
		}
//		printf("Hello from app_main!\n");

//		 char* msg = "abc\r\n";
//		     if (uart_write_bytes(uart_rs485_num, (uint8_t*)msg, strlen(msg)) != strlen(msg)) {
//		         ESP_LOGE(TAG, "Send data critical failure.");
//		     }

//sleep(1);
	}
}

static uint16_t MODBUS_CRC16_V1(const uint8_t *buf, uint16_t len) {
	uint16_t crc1 = 0xFFFF;
	uint16_t i = 0;
	uint8_t b = 0;
	for (i = 0; i < len; i++) {
		crc1 ^= buf[i];

		for (b = 0; b < 8; b++) {
			if (crc1 & 0x0001) {
				crc1 >>= 1;
				crc1 ^= 0xA001;
			} else {
				crc1 >>= 1;
			}
		}
	}
	return crc1;
}
static bool MODBUS_CRC_pass(uint8_t *data, uint16_t data_len) {
	uint16_t crcValue = MODBUS_CRC16_V1(data, data_len - 2);
	if (((data[data_len - 1] << 8) + data[data_len - 2]) == crcValue) {

		return true;
	}
	return false;
}
