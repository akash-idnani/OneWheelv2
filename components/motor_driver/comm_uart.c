#include "comm_uart.h"
#include "bldc_interface_uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/uart.h"
#include "esp_log.h"

#include <string.h>

#define TAG "YO"
#define EX_UART_NUM UART_NUM_2

static QueueHandle_t uart_queue;

static void send_packet(unsigned char *data, unsigned int len) {
	if (len > (PACKET_MAX_PL_LEN + 5)) {
		return;
	}

	static uint8_t buffer[PACKET_MAX_PL_LEN + 5];
	memcpy(buffer, data, len);
	
	uart_write_bytes(UART_NUM_2, (const char *) data, len);
}

void timer_task(void *pvParameters) {
	while(1) {
		bldc_interface_uart_run_timer();
		vTaskDelay(1 / portTICK_RATE_MS);
	}
}

void uart_event_task(void *pvParameters) {
	uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(2048);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, 2048);
            ESP_LOGI("HELLO", "uart[%d] event:", UART_NUM_2);
            switch(event.type) {
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
					for (int i = 0; i < event.size; i++) {
						bldc_interface_uart_process_byte(dtmp[i]);
					}
                    ESP_LOGI(TAG, "[DATA EVT]:");
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void comm_uart_init(void) {
	uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 17, 
        16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        
    const int uart_buffer_size = (1024 * 2);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, \
                                        uart_buffer_size, 20, &uart_queue, 0));
	bldc_interface_uart_init(send_packet);
    
	TaskHandle_t timer_task_handle = NULL;
    xTaskCreate(timer_task, "Timer", 2048, NULL, 5, &timer_task_handle); 

	xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}
