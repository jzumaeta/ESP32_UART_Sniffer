#include <Arduino.h>

//prototype functions
void readUART1(void *parameter);
void readUART2(void *parameter);
void printData(void *parameter);

// Structure to store data and timestamp
struct UARTData {
  byte data[256];          // Buffer for storing accumulated data
  unsigned int length;      // Length of the data
  unsigned long timestamp;  // Timestamp of the first byte in milliseconds
};

// Configurable inter-byte timeout in milliseconds
const unsigned long INTER_BYTE_TIMEOUT_MS = 10;

// Define queues to receive data from each UART
QueueHandle_t uart1Queue;
QueueHandle_t uart2Queue;

void setup() {
  // Initialize Serial for the monitor
  Serial.begin(115200);

  // Configure UART1 on GPIO32 (RX) and GPIO17 (TX) with baud rate 19200
  Serial1.begin(19200, SERIAL_8N1, 32, 17);

  // Configure UART2 on GPIO33 (RX) and GPIO5 (TX) with baud rate 19200
  Serial2.begin(19200, SERIAL_8N1, 33, 5);

  // Create queues to store data from each UART
  uart1Queue = xQueueCreate(10, sizeof(UARTData));
  uart2Queue = xQueueCreate(10, sizeof(UARTData));

  // Create FreeRTOS tasks for each UART and the print task
  xTaskCreate(readUART1, "Read UART1", 2048, NULL, 1, NULL);
  xTaskCreate(readUART2, "Read UART2", 2048, NULL, 1, NULL);
  xTaskCreate(printData, "Print Data", 4096, NULL, 1, NULL);

  Serial.println("Starting real-time UART reading with timestamps in hh:mm:ss.ms format...");
}

void loop() {
  // The main loop remains empty as FreeRTOS manages the execution
}

// Convert milliseconds to hh:mm:ss.ms format
String formatTimestamp(unsigned long millis) {
  unsigned long seconds = millis / 1000;
  unsigned int hours = (seconds / 3600) % 24;
  unsigned int minutes = (seconds % 3600) / 60;
  unsigned int secs = seconds % 60;
  unsigned int ms = millis % 1000;

  char timeString[13]; // Buffer for "hh:mm:ss.ms" format
  snprintf(timeString, sizeof(timeString), "%02u:%02u:%02u.%03u", hours, minutes, secs, ms);
  return String(timeString);
}

// Task to read data from UART1
void readUART1(void *parameter) {
  UARTData uartData;
  uartData.length = 0;
  unsigned long lastByteTime = 0;

  for (;;) {
    if (Serial1.available() > 0) {
      // If first byte, capture the timestamp
      if (uartData.length == 0) {
        uartData.timestamp = millis();
      }

      // Read the byte and store it in the buffer
      uartData.data[uartData.length++] = Serial1.read();
      lastByteTime = millis();

    } else if (uartData.length > 0 && (millis() - lastByteTime >= INTER_BYTE_TIMEOUT_MS)) {
      // If timeout has passed and data is accumulated, send it to the queue
      xQueueSend(uart1Queue, &uartData, portMAX_DELAY);
      uartData.length = 0;  // Reset buffer length for next message
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay to yield CPU
  }
}

// Task to read data from UART2
void readUART2(void *parameter) {
  UARTData uartData;
  uartData.length = 0;
  unsigned long lastByteTime = 0;

  for (;;) {
    if (Serial2.available() > 0) {
      // If first byte, capture the timestamp
      if (uartData.length == 0) {
        uartData.timestamp = millis();
      }

      // Read the byte and store it in the buffer
      uartData.data[uartData.length++] = Serial2.read();
      lastByteTime = millis();

    } else if (uartData.length > 0 && (millis() - lastByteTime >= INTER_BYTE_TIMEOUT_MS)) {
      // If timeout has passed and data is accumulated, send it to the queue
      xQueueSend(uart2Queue, &uartData, portMAX_DELAY);
      uartData.length = 0;  // Reset buffer length for next message
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay to yield CPU
  }
}

// Task to print data to the serial monitor
void printData(void *parameter) {
  UARTData uartData;

  for (;;) {
    // Check and receive data from UART1 queue
    while (xQueueReceive(uart1Queue, &uartData, 0) == pdTRUE) {
      Serial.print("UART1 [");
      Serial.print(formatTimestamp(uartData.timestamp)); // Print formatted timestamp with milliseconds
      Serial.print("]: ");
      for (int i = 0; i < uartData.length; i++) {
        char hexString[3]; // Buffer for each hexadecimal byte
        snprintf(hexString, sizeof(hexString), "%02X", uartData.data[i]);
        Serial.print(hexString); // Print each byte in hexadecimal format
        Serial.print(" ");
      }
      Serial.println();
    }

    // Check and receive data from UART2 queue
    while (xQueueReceive(uart2Queue, &uartData, 0) == pdTRUE) {
      Serial.print("UART2 [");
      Serial.print(formatTimestamp(uartData.timestamp)); // Print formatted timestamp with milliseconds
      Serial.print("]: ");
      for (int i = 0; i < uartData.length; i++) {
        char hexString[3]; // Buffer for each hexadecimal byte
        snprintf(hexString, sizeof(hexString), "%02X", uartData.data[i]);
        Serial.print(hexString); // Print each byte in hexadecimal format
        Serial.print(" ");
      }
      Serial.println();
    }

    vTaskDelay(1 / portTICK_PERIOD_MS); // Small delay to yield CPU
  }
}