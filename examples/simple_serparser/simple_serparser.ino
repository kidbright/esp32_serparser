#include "esp32_serparser.h"

ESP32SerParser SerParser(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, GPIO_NUM_2, 1, 2000); // uart number=2, tx=GPIO17, rx=GPIO16, led=GPIO2, led active high (1), read timeout 2000ms

void setup() {
  Serial.begin(115200);
  SerParser.begin(115200);
}

void loop() {
  int i, j;
  char str[SERPARSER_MAX_PARAM_CHARS];

  if (!SerParser.is_error()) {
    for (j = 0; j < 22; j++) {      
      for (i = 0; i < SERPARSER_MAX_PARAMS; i++) {
        SerParser.get_params(j, i, str);
        if (str[0] == '\0') {
          break;
        }
        else {
          Serial.print(str);
          Serial.print(" ");
        }
      }
      Serial.println("");
    }

    Serial.println("==================================================");
  }
  else {
    // no serial data
    Serial.println("ERROR!");
  }
  delay(1000);
}
