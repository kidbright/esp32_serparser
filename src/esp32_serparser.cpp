#include <stdio.h>
#include <string.h>
#include "esp32_serparser.h"

/*
<S FAULT>0| NECTEC IPP ExMPPT 220607 <TIME>03:15:02<COUNTER>11604

<Mode(1)>Auto|  <Gain-(2)>10| <Gain+(3)>10|   <rGain(18)>100|
<V-mppt(9)>480.0| <V-mppt min(16)>310.0| <V-under(17)>220.0|

<V-input>18.3|  <V-Open>0.0|
<V-Rate>0.0|
<Freq>0.0%| <PWM>0| <DAC OUT>OFF|  <Ripple>0.0|%
<AC Control(12)>Disable| 
<Inv contact> Open| <Num err>0|

<SENSOR>
<Float>   Open(Run)|
<Flow>    Open|
<Temp>  24.7| <ADC_2>0|
@CONFIG    @Min    @Max     @Ena/Dis  @Counter
@S Temp   <(14)>40|C  <(13)>90|C  <(13)>Ena|  <(11)>20|
@S Freq   < (5)>60|%  < (6)>90|%            <( 7)>30|
@S Flow                       <(10)>N/O|  <(11)>180|

<DC>NO| <MAG>OFF|
<SWITCH>RUN|
<RUN MPPT>1|
*/

TickType_t ESP32SerParser::get_tickcnt(void) {
	return xTaskGetTickCount();
}

bool ESP32SerParser::is_tickcnt_elapsed(TickType_t tickcnt, uint32_t tickcnt_ms) {
	TickType_t curr_tickcnt = xTaskGetTickCount();

	if ((curr_tickcnt - tickcnt) >= (tickcnt_ms / portTICK_RATE_MS)) {
		tickcnt = curr_tickcnt;
		return true;
	}

	return false;
}

ESP32SerParser::ESP32SerParser(uart_port_t _uart_num, gpio_num_t _tx_gpio, gpio_num_t _rx_gpio, gpio_num_t _led_gpio, int _led_active, uint32_t _timeout_ms) {
	int i, j;

	uart_num = _uart_num;
	led_gpio = _led_gpio;
	led_active = _led_active & 0x01;
	timeout_ms = _timeout_ms;
	tx_gpio = _tx_gpio;
	rx_gpio = _rx_gpio;

	// clear output lines
	for (j = 0; j < SERPARSER_MAX_LINES; j++) {
		for (i = 0; i < SERPARSER_MAX_PARAMS; i++) {
			output_lines[j][i][0] = '\0';
		}
	}
}

void ESP32SerParser::serparser_extract_params(int line_no, String str, ParamsType *params) {
	int i, start_index, g_index, o_index;
	String temp_str;
	
	// clear current line parameters
	for (i = 0; i < SERPARSER_MAX_PARAMS; i++) {
		output_lines[line_no][i][0] = '\0';
	}

	i = 0;
	start_index = 0;
	// extract params between ">" and "|"
	while ((g_index = str.indexOf('>', start_index)) != -1) {   
		if ((o_index = str.indexOf('|', g_index)) != -1) {
			temp_str = str.substring(g_index + 1, o_index);
			temp_str.trim();

			//Serial.print(temp_str);
			//Serial.print(" ");
			strncpy(output_lines[line_no][i], temp_str.c_str(), SERPARSER_MAX_PARAM_CHARS - 1);
		}

		if (o_index != -1) {
			start_index = o_index;
			i++;
		}
		else {
			break;
		}
  }

 // Serial.println();
}

void ESP32SerParser::SerParserTask(void *pvParameters) {
	ESP32SerParser *serparser = (ESP32SerParser *)pvParameters;
	int i, or_index;
	char ch;
	ParamsType params;

	while (1) {
		// serparser state
		switch (serparser->state) {
			case s_serparser_start:
				serparser->timeout_count = 0;
				serparser->line_index = 0;
				serparser->lines[0] = "";
				serparser->found_NECTEC = false;
				serparser->state = s_serparser_avail;
				break;

			case s_serparser_avail:			
				// read 1 byte
				if ((uart_read_bytes(serparser->uart_num, &ch, 1, SERPARSER_RX_TIMEOUT_MS / portTICK_RATE_MS)) == 1) {
					if ((ch != 0x00) && (ch != 0x0a) && (ch != 0x0c)) {
						// convert "0a 0d" to "0d 0a"
						if (ch == 0x0d) {
							ch = '\n';
						}

						// add char to current line
						serparser->lines[serparser->line_index] += ch;
							
						// check end of line
						if (ch == '\n') {
							//Serial.print(serparser->lines[serparser->line_index]);

							// scan @12 for NECTEC
							if (!serparser->found_NECTEC) {
								//if (serparser->lines[serparser->line_index].substring(12, 18) == "NECTEC") {
								if (serparser->lines[serparser->line_index].indexOf("NECTEC") >= 0) {
									serparser->found_NECTEC = true;
								}
								else {
									// NECTEC not found, go back to start
									serparser->state = s_serparser_start;
									// just break
									break;
								}
							}
							else {
								// scan @1 for RUN MPPT
								if (serparser->lines[serparser->line_index].substring(1, 4) == "RUN") {
									// go to process frame
									serparser->state = s_serparser_process;
									// just break
									break;
								}
							}
			
							serparser->line_index++;
							// line overflow, go back to start
							if (serparser->line_index >= SERPARSER_MAX_LINES) {
								serparser->state = s_serparser_start;
								break;
							}
					
							// clear next line
							serparser->lines[serparser->line_index] = "";
						}
					}
				}
				else {
					serparser->timeout_count++;
					if (serparser->timeout_count >= (serparser->timeout_ms / SERPARSER_RX_TIMEOUT_MS)) {
						serparser->error = true;
						serparser->state = s_serparser_start;
					}
				}
				break;

			case s_serparser_process:
				// verify last line index
				if (serparser->line_index == 22) {
					// clear error
					serparser->error = false;

					for (i = 0; i <= 22; i++) {
						serparser->serparser_extract_params(i, serparser->lines[i], &params);

						if (i == 0) {
							if (serparser->lines[i].substring(0, 4) == "@Set") {
								or_index = serparser->lines[i].indexOf('|');
								strncpy(serparser->output_lines[0][0], serparser->lines[i].substring(0, or_index).c_str(), SERPARSER_MAX_PARAM_CHARS - 1);
							}							
						}
					}				
				}

				serparser->state = s_serparser_start;
      			break;

			case s_serparser_idle:
				break;    
		}

		// led state machine
		switch (serparser->led_state) {
			case s_serparser_led_init:
				serparser->led_toggle_status = serparser->led_active ^ 0x01;
				serparser->led_tickcnt = serparser->get_tickcnt();
				serparser->led_state = s_serparser_led_idle;
				break;

			case s_serparser_led_idle:
				if (serparser->is_tickcnt_elapsed(serparser->led_tickcnt, SERPARSER_RX_TIMEOUT_MS)) {
					serparser->led_tickcnt = serparser->get_tickcnt();
					// update led toggle status
					serparser->led_toggle_status = serparser->led_toggle_status ^ 0x01;
					// update led					
					if (!serparser->error) {		
						gpio_set_level(serparser->led_gpio, serparser->led_active);
					}
					else {
						gpio_set_level(serparser->led_gpio, serparser->led_toggle_status);
					}											
				}			
				break;
		}	
	}
}

void ESP32SerParser::begin(uint32_t _baudrate) {
	gpio_config_t io_conf;		

	const uart_config_t uart_config = {
        .baud_rate = _baudrate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
	    .use_ref_tick = true
    };
    // config uart
    uart_driver_install(uart_num, SERPARSER_RX_BUF_SIZE * 2, SERPARSER_TX_BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, tx_gpio, rx_gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// led init
	io_conf.intr_type = GPIO_INTR_DISABLE; // disable interrupt
	io_conf.mode = GPIO_MODE_OUTPUT; // set as output mode
	io_conf.pin_bit_mask = (1ULL << led_gpio); // pin bit mask
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // disable pull-down mode
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // disable pull-up mode				
	gpio_set_level(led_gpio, led_active ^ 0x01);
	gpio_config(&io_conf);

	error = true;
	state = s_serparser_start;
	led_state = s_serparser_led_init;	

	// create modbus master task
	xTaskCreate(this->SerParserTask, "SerParser", SERPARSER_STACK_SIZE_MIN, this, SERPARSER_TASK_PRIORITY, NULL);
}

bool ESP32SerParser::is_error(void) {
	return error;
}

void ESP32SerParser::get_params(int line_no, int param_no, char *param) {
	// default null string
	strcpy(param, "");	
	if ((line_no < SERPARSER_MAX_LINES) && (param_no < SERPARSER_MAX_PARAMS)) {
		strncpy(param, output_lines[line_no][param_no], SERPARSER_MAX_PARAM_CHARS - 1);
	}
}

void ESP32SerParser::write_bytes(const char *data) {
        uart_write_bytes(uart_num, data, strlen(data));
}
