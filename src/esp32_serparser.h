#ifndef ESP32_SERPARSER_H
#define ESP32_SERPARSER_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "HardwareSerial.h"

#define SERPARSER_TX_BUF_SIZE       1024
#define SERPARSER_RX_BUF_SIZE       1024
#define SERPARSER_RX_TIMEOUT_MS     200
#define	SERPARSER_TASK_PRIORITY     8
#define SERPARSER_STACK_SIZE_MIN    8192
#define SERPARSER_MAX_LINES		      32
#define SERPARSER_MAX_PARAMS        8
#define SERPARSER_MAX_PARAM_CHARS   32

typedef struct ParamsStruct {
  int length;
  String params[8];
} ParamsType;

class ESP32SerParser {  
  private:
    TickType_t get_tickcnt(void);
    bool is_tickcnt_elapsed(TickType_t tickcnt, uint32_t tickcnt_ms);
    enum {
       s_serparser_start, s_serparser_avail, s_serparser_process, s_serparser_idle
    } state;
    enum {
      s_serparser_led_init, s_serparser_led_idle
    } led_state;    
    uart_port_t uart_num;
    int line_index, timeout_count;
    String lines[SERPARSER_MAX_LINES];
    bool found_NECTEC;
    uint32_t timeout_ms;
    gpio_num_t tx_gpio, rx_gpio, led_gpio;
    int led_active;
    uint8_t led_toggle_status;
    TickType_t tickcnt, led_tickcnt;
    bool error;
    char output_lines[SERPARSER_MAX_LINES][SERPARSER_MAX_PARAMS][SERPARSER_MAX_PARAM_CHARS];
    static void SerParserTask(void *pvParameters);
    void serparser_extract_params(int line_no, String str, ParamsType *params);

  public:
    ESP32SerParser(uart_port_t _uart_num, gpio_num_t _tx_gpio, gpio_num_t _rx_gpio, gpio_num_t _led_gpio, int _led_active, uint32_t _timeout_ms);
    void begin(uint32_t _baudrate);
    bool is_error(void);
    void get_params(int line_no, int param_no, char *param);
    void write_bytes(const char *data);

};

#endif
