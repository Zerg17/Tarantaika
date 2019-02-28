#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/usart.h>
#include <stdlib.h>
#include <strings.h>
#include "delay.h"
#include "uart.h"

volatile uint32_t millis;
volatile uint8_t control = 0;
;
char bufC[64];
uint16_t rel = 0;
uint8_t data_old, uart_st = 0, count = 0;
uint16_t crc;
uint8_t activ = 0;
int32_t it = 0;
int16_t curs = 512;

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define constrain(amt, low, high) \
    ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define round(x) ((x) >= 0 ? (long)((x) + 0.5) : (long)((x)-0.5))
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

union {
    struct {
        int16_t angle;
        uint16_t sysa;
    } msg;
    uint8_t buf[64];
} pack;

void setDuty(int16_t duty) {
    //TIM_CCR2(TIM2) = constrain(duty, -1000, 1000) + 3000;
    TIM_CCR2(TIM2) = constrain(duty, -500, 500) + 3000;         // Тут преобразование мощности значения от -1000 до 1000 но веск ограничен в 70% так что по факту от -700 до 700
                                                                // и Миша просил цепь не убивать так что вообще 500 значения
}

void clock_setup() {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();
    delayInit();
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);
    rcc_periph_clock_enable(RCC_AFIO);
    rcc_periph_clock_enable(RCC_DMA1);
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_TIM2);
}

void systick_setup() {
    systick_set_reload(89999);
    systick_interrupt_enable();
    systick_counter_enable();
}

void gpio_setup() {}

void vesc_setup() {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL,
                  GPIO_TIM2_CH2);

    timer_enable_preload(TIM2);
    TIM_CCER(TIM2) |= TIM_CCER_CC2E;
    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
    timer_set_period(TIM2, 39999);
    timer_set_prescaler(TIM2, 35);
    timer_enable_counter(TIM2);
}

void spi_setup() {
    rcc_periph_clock_enable(RCC_SPI1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO4);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);
    spi_reset(SPI1);
    spi_init_master(
        SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_8BIT, SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);
}

int main(void) {
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();
    systick_setup();
    vesc_setup();

    // sprintf(bufC, "\n");
    uPD("\nTarantaika steering start...\n");  // Для определения прошивки

    nvic_enable_irq(NVIC_USART1_IRQ);
    USART_BRR(USART1) = rcc_apb2_frequency / 19200 + 1;
    USART_CR1(USART1) |= USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    nvic_enable_irq(NVIC_USART1_IRQ);

    systick_setup();
    activ = 255;
    setDuty(0);

    while (1) {
    }
}

void sys_tick_handler() {
    if (activ)
        activ--;
    else
        control = 0;

    GPIO_BRR(GPIOA) = GPIO4;
    _delay_us(5);
    GPIO_BSRR(GPIOA) = GPIO4;
    uint8_t lo = spi_xfer(SPI1, 0xAA);
    uint8_t hi = spi_xfer(SPI1, 0xAA);
    uint16_t enc = hi << 8 | lo;               // Значения с энкодера 
    int16_t err = -(curs - enc);               // curs требуемое значение
    it = constrain(it + err, -2000, 2000);     // кострация ишки
    //sprintf(bufC, "%d %d %d %d\n", curs, enc, err, it);
    //sprintf(bufC, "%d %X\n", (hi << 8 | lo), (hi << 8 | lo));
    
    //uPD(bufC);
    if (control && enc)                        // Проверка на наличае управления и наличе данных у энкодера
        setDuty(err*6 + it/5);                 // Хреновый пи регулятор надо убавить П добавить И, но если только будет Д пока ее нет и так норм 
    else
        setDuty(0);                            // Сбрасываем до 0  мощность но веск тормозит движок надо поправить 
}

void usart1_isr() {
    if ((USART_SR(USART1) & USART_SR_RXNE)) {
        uint8_t data = usart_recv(USART1);
        switch (uart_st) {
            case 0:
                if (data == 0x55 && data_old == 0x55) {
                    uart_st = 1;
                    crc = 211;
                    count = sizeof(pack.msg);
                }
                break;

            case 1:
                if (!--count) uart_st = 2;
                pack.buf[(sizeof(pack.msg) - 1) - count] = data;
                crc += data * 211;
                crc ^= crc >> 8;
                break;

            case 2:
                if (data == (crc & 0xff)) {
                    activ = 20;
                    control = pack.msg.sysa&(1<<0);
                    // curs=pack.msg.angle;
                    //setDuty(pack.msg.angle / 10);
                    
                    curs = map(pack.msg.angle, -10000, 10000, 923, 100);   // Тут можно подправить границы поворота колес
                    //sprintf(bufC, "%d\n", pack.msg.angle);
                    //uPD(bufC);
                    //uPD("P\n");
                }
                uart_st = 0;
                data_old = 0;
                break;

            default:
                break;
        }
        data_old = data;
    }
}