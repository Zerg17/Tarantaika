#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <stdlib.h>
#include <strings.h>
#include "delay.h"
#include "uart.h"

#define FR (1 << 0)  //Направление движения 0-вперед 1-назад
//#define PEDAL (1<<1)
#define INTERLOCK (1 << 2)  // Разрешение движения
#define PEDAL (1 << 3)      // Перехват педали газа
#define PANEL (1 << 4)      // Отключение панели
#define LIGHT_AI (1 << 5)   // Перехват света
#define DIMENS (1 << 6)     // Габариты
//#define PEDAL (1<<7)
#define PUPILS (1 << 8)  // Перехват поворотников
#define LEFT (1 << 9)    // Левый поворотник
#define RIGHT (1 << 10)  // Правый поворотник
#define BRAKE (1 << 11)  // Тормоз 0-нажат 1-отжат
//#define PEDAL (1<<12)
//#define PEDAL (1<<13)
//#define PEDAL (1<<14)
//#define PEDAL (1<<15)

volatile uint32_t millis;
char bufC[64];
uint16_t rel = 0;
uint8_t data_old, uart_st = 0, count = 0;
uint16_t crc;
uint8_t activ = 0;

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define round(x)     ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

union {
    struct {
        int16_t speed;
        uint16_t light;
        uint16_t sysa;
        uint16_t sysb;
    } msg;
    uint8_t buf[64];
} pack;

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

void spi_setup() {
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL,
                  GPIO2);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO4 | GPIO5 | GPIO7);
    spi_reset(SPI1);
    spi_init_master(
        SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
        SPI_CR1_CPHA_CLK_TRANSITION_2, SPI_CR1_DFF_16BIT, SPI_CR1_MSBFIRST);
    spi_enable_software_slave_management(SPI1);
    spi_set_nss_high(SPI1);
    spi_enable(SPI1);
}

void move_setup(){
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_TIM2_CH2);

    timer_enable_preload(TIM2);
    TIM_CCER(TIM2) |= TIM_CCER_CC2E;
    timer_set_oc_mode(TIM2, TIM_OC2, TIM_OCM_PWM1);
    timer_set_period(TIM2, 10000);
    timer_set_prescaler(TIM2, 6);  // Тут можно убавить частоту шима хз что там на оптопаре
    timer_enable_counter(TIM2);
}

void systick_setup() {
    systick_set_reload(89999);
    systick_interrupt_enable();
    systick_counter_enable();
}

void releSet(uint16_t r) {                   // Для установки непосредственного значения
    spi_write(SPI1, ~(r));
    while ((SPI1_SR & SPI_SR_BSY))
        ;
    GPIO_BSRR(GPIOA) = GPIO2;
    _delay_us(5);
    GPIO_BRR(GPIOA) = GPIO2;
}

void rele(uint16_t set, uint16_t res) {     // Для установки значения по маскам
    static uint16_t rele = 0;
    rel = (rel | set) & (~res);
    releSet(rel);
}

void releVal(uint16_t bit, uint16_t val) {  // Установка указанного значения в нужное состояние
    if (val)
        rele(bit, 0);
    else
        rele(0, bit);
}

void gpio_setup() {}

int main(void) {
    clock_setup();
    gpio_setup();
    usart_setup();
    spi_setup();
    move_setup();

    // sprintf(bufC, "\n");
    uPD("\nTarantaika motor start...\n");  // Для проверки того что залито

    nvic_enable_irq(NVIC_USART1_IRQ);

    USART_BRR(USART1) = rcc_apb2_frequency / 19200 + 1;
    USART_CR1(USART1) |= USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    nvic_enable_irq(NVIC_USART1_IRQ);

    systick_setup();
    releSet(0);
    activ=255;

    while (1) {
    }
}

void sys_tick_handler() {
    if (activ)   // Проверяем активность прихода данных
        activ--;
    else{
        releSet(LEFT | RIGHT | PUPILS);  // Если данных нет уходим на аварийку 
        TIM_CCR2(TIM2) = 0;
    }
}

void usart1_isr() {
    if ((USART_SR(USART1) & USART_SR_RXNE)) {
        uint8_t data = usart_recv(USART1);
        // sprintf(bufC, "D:%02X %d %d\n", data, uart_st, count);
        // uPD(bufC);
        switch (uart_st) {
            case 0:
                if (data == 0x55 && data_old == 0x55) {
                    uart_st = 1;
                    crc = 211;
                    count = sizeof(pack.msg);
                    // uPD("Start\n");
                }
                break;

            case 1:
                if (!--count) uart_st = 2;
                pack.buf[7 - count] = data;
                crc += data * 211;
                crc ^= crc >> 8;
                break;

            case 2:
                if (data == (crc & 0xff)) {
                    activ = 20;

                    //sprintf(bufC, "SYS %d\n", activ);
                    //uPD(bufC);

                    if (pack.msg.sysa & (1 << 0)) {
                        rele(PEDAL | PANEL | LIGHT_AI | PUPILS, 0);   // Если пульт включен переводим на автономное управление путем перехвата всего и вся лучше у влада спросить что тут происходит
                        releVal(DIMENS, pack.msg.light & (1 << 0));   // Включение передних фар а ниже поворотников
                        releVal(LEFT, pack.msg.light & (1 << 2));
                        releVal(RIGHT, pack.msg.light & (1 << 1));
                        if(pack.msg.speed){
                            rele(INTERLOCK, 0);                                           // Если скорочть не 0 то включаем интерлок
                            TIM_CCR2(TIM2) = constrain(abs(pack.msg.speed), 0, 10000);    // И задаем шим на скорость 
                        }else{
                            TIM_CCR2(TIM2) = 0;                                           // При 0 скорости просто шим в 0 но не выключаем интерлок чтобы ехать накатом
                        }
                        if(pack.msg.sysb & (1 << 1)) rele(0, INTERLOCK);                  // В теории включает рекупирацию но хз 

                        releVal(BRAKE, !(pack.msg.sysb & (1 << 0)));                      // Включаем тормоз

                        releVal(FR, pack.msg.speed>0);                                    // Устанавливаем напрвление движения

                    }
                    else{
                        TIM_CCR2(TIM2) = 0;                                               // Сбрасываем все реле и шим чтобы перейти в ркчное управление если пульт выключен
                        rele(0, 0xFFFF);
                    }
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