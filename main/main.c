#include <FreeRTOS.h>
#include <queue.h>
#include <semphr.h>
#include <stdio.h>
#include <task.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pins.h"
#include "ssd1306.h"

/* CONFERIR: pinos do sensor ultrassonico. Se o pins.h ja definir, usa os dele. */
#ifndef TRIGGER_PIN
#define TRIGGER_PIN 16
#endif
#ifndef ECHO_PIN
#define ECHO_PIN 17
#endif

#define PERIODO_LEITURA_MS  200   /* intervalo entre medicoes (HC-SR04 pede >= 60 ms) */
#define TIMEOUT_ECHO_MS     50    /* echo mais longo do sensor: ~38 ms */
#define TIMEOUT_DIST_MS     100   /* trigger + echo + calculo, com folga */
#define DIST_MAX_CM         400   /* alcance do HC-SR04 */
#define LIMITE_PERTO_CM     100   /* ate 1 m: verde; acima: amarelo */

/* Unicas variaveis globais: recursos do RTOS */
QueueHandle_t xQueueTime;
SemaphoreHandle_t xSemaphoreTrigger;
QueueHandle_t xQueueDistance;

/* === Callback do pino ECHO: so registra o instante da borda === */
void pin_callback(uint gpio, uint32_t events) {
    if (gpio == ECHO_PIN) {
        if ((events & GPIO_IRQ_EDGE_RISE) || (events & GPIO_IRQ_EDGE_FALL)) {
            uint64_t agora = to_us_since_boot(get_absolute_time());
            xQueueSendFromISR(xQueueTime, &agora, 0);
        }
    }
}

/* === OLED e LED (usados so pela oled_task) === */
void oled_display_init(ssd1306_t *disp) {
    i2c_init(i2c1, 400000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    disp->external_vcc = false;
    ssd1306_init(disp, 128, 64, 0x3C, i2c1);
    ssd1306_clear(disp);
    ssd1306_show(disp);

    gpio_init(SSD1306_PIN_LITE);
    gpio_set_dir(SSD1306_PIN_LITE, GPIO_OUT);
    gpio_put(SSD1306_PIN_LITE, 0);
}

void led_rgb_init(void) {
    gpio_init(LED_PIN_R);
    gpio_set_dir(LED_PIN_R, GPIO_OUT);
    gpio_put(LED_PIN_R, 1);

    gpio_init(LED_PIN_G);
    gpio_set_dir(LED_PIN_G, GPIO_OUT);
    gpio_put(LED_PIN_G, 1);

    gpio_init(LED_PIN_B);
    gpio_set_dir(LED_PIN_B, GPIO_OUT);
    gpio_put(LED_PIN_B, 1);
}

/* LED RGB da PicoDock e ativo em 0: 0 acende, 1 apaga */
void led_rgb_set(int r, int g, int b) {
    gpio_put(LED_PIN_R, !r);
    gpio_put(LED_PIN_G, !g);
    gpio_put(LED_PIN_B, !b);
}

/* === Tasks === */

void trigger_task(void *p) {
    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    while (true) {
        /* Descarta bordas antigas: cada medicao comeca com a fila vazia */
        xQueueReset(xQueueTime);

        /* Pulso de trigger: o sensor pede 10 us; 1 ms e o minimo do vTaskDelay */
        gpio_put(TRIGGER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));
        gpio_put(TRIGGER_PIN, 0);

        /* Avisa o OLED que uma leitura foi disparada */
        xSemaphoreGive(xSemaphoreTrigger);

        vTaskDelay(pdMS_TO_TICKS(PERIODO_LEITURA_MS));
    }
}

void echo_task(void *p) {
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(ECHO_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, &pin_callback);

    while (true) {
        uint64_t t_subida;
        uint64_t t_descida;

        /* Espera a borda de subida do echo e, em seguida, a de descida */
        if (xQueueReceive(xQueueTime, &t_subida, portMAX_DELAY) == pdTRUE) {
            if (xQueueReceive(xQueueTime, &t_descida, pdMS_TO_TICKS(TIMEOUT_ECHO_MS)) == pdTRUE) {
                /* Som a 343 m/s, ida e volta: distancia [cm] = tempo [us] / 58 */
                int distancia = (int)((t_descida - t_subida) / 58);

                if (distancia <= DIST_MAX_CM) {
                    xQueueSend(xQueueDistance, &distancia, 0);
                }
                /* fora do alcance: nao envia; o OLED trata como falha */
            }
        }
    }
}

void oled_task(void *p) {
    ssd1306_t disp;
    oled_display_init(&disp);
    led_rgb_init();

    char texto[24];

    while (true) {
        if (xSemaphoreTake(xSemaphoreTrigger, portMAX_DELAY) == pdTRUE) {
            int distancia;

            ssd1306_clear(&disp);

            if (xQueueReceive(xQueueDistance, &distancia, pdMS_TO_TICKS(TIMEOUT_DIST_MS)) == pdTRUE) {
                /* Leitura ok: valor + barra proporcional (0 a 400 cm -> 0 a 128 px) */
                snprintf(texto, sizeof(texto), "%d cm", distancia);
                ssd1306_draw_string(&disp, 8, 16, 2, texto);

                int largura = (distancia * 128) / DIST_MAX_CM;
                if (largura > 0) {
                    ssd1306_draw_square(&disp, 0, 48, largura, 10);
                }

                if (distancia <= LIMITE_PERTO_CM) {
                    led_rgb_set(0, 1, 0);   /* verde: ate 1 m */
                } else {
                    led_rgb_set(1, 1, 0);   /* amarelo: mais de 1 m */
                }
            } else {
                /* A distancia nao chegou no tempo: falha no sensor */
                ssd1306_draw_string(&disp, 8, 24, 2, "FALHA");
                led_rgb_set(1, 0, 0);       /* vermelho */
            }

            ssd1306_show(&disp);
        }
    }
}

int main() {
    stdio_init_all();

    xQueueTime = xQueueCreate(8, sizeof(uint64_t));
    xSemaphoreTrigger = xSemaphoreCreateBinary();
    xQueueDistance = xQueueCreate(8, sizeof(int));

    xTaskCreate(trigger_task, "Trigger", 4095, NULL, 1, NULL);
    xTaskCreate(echo_task, "Echo", 4095, NULL, 1, NULL);
    xTaskCreate(oled_task, "OLED", 4095, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true);
}