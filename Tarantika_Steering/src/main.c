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
volatile uint8_t control=0;;
char bufC[64];
uint16_t rel = 0;
uint8_t data_old, uart_st = 0, count = 0;
uint16_t crc;
uint8_t activ = 0;
int32_t it=0;
int16_t curs=512;

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

typedef enum {
    COMM_FW_VERSION = 0,
    COMM_JUMP_TO_BOOTLOADER,
    COMM_ERASE_NEW_APP,
    COMM_WRITE_NEW_APP_DATA,
    COMM_GET_VALUES,
    COMM_SET_DUTY,
    COMM_SET_CURRENT,
    COMM_SET_CURRENT_BRAKE,
    COMM_SET_RPM,
    COMM_SET_POS,
    COMM_SET_HANDBRAKE,
    COMM_SET_DETECT,
    COMM_SET_SERVO_POS,
    COMM_SET_MCCONF,
    COMM_GET_MCCONF,
    COMM_GET_MCCONF_DEFAULT,
    COMM_SET_APPCONF,
    COMM_GET_APPCONF,
    COMM_GET_APPCONF_DEFAULT,
    COMM_SAMPLE_PRINT,
    COMM_TERMINAL_CMD,
    COMM_PRINT,
    COMM_ROTOR_POSITION,
    COMM_EXPERIMENT_SAMPLE,
    COMM_DETECT_MOTOR_PARAM,
    COMM_DETECT_MOTOR_R_L,
    COMM_DETECT_MOTOR_FLUX_LINKAGE,
    COMM_DETECT_ENCODER,
    COMM_DETECT_HALL_FOC,
    COMM_REBOOT,
    COMM_ALIVE,
    COMM_GET_DECODED_PPM,
    COMM_GET_DECODED_ADC,
    COMM_GET_DECODED_CHUK,
    COMM_FORWARD_CAN,
    COMM_SET_CHUCK_DATA,
    COMM_CUSTOM_APP_DATA,
    COMM_NRF_START_PAIRING
} COMM_PACKET_ID;

const uint16_t crc16_tab[] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

uint16_t crc16(uint8_t *buf, uint16_t len) {
    uint16_t cksum = 0;
    for (uint16_t i = 0; i < len; i++)
        cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
    return cksum;
}

void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = number >> 24;
    buffer[(*index)++] = number >> 16;
    buffer[(*index)++] = number >> 8;
    buffer[(*index)++] = number;
}

void packSendPayload(uint8_t *payload, uint16_t lenPay) {
    uint16_t crcPayload = crc16(payload, lenPay);
    uint16_t count = 0;
    uint8_t messageSend[256];

    if (lenPay <= 256) {
        messageSend[count++] = 2;
        messageSend[count++] = lenPay;
    } else {
        messageSend[count++] = 3;
        messageSend[count++] = (uint8_t)(lenPay >> 8);
        messageSend[count++] = (uint8_t)(lenPay & 0xFF);
    }

    memcpy(&messageSend[count], payload, lenPay);

    count += lenPay;
    messageSend[count++] = (uint8_t)(crcPayload >> 8);
    messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
    messageSend[count++] = 3;
    messageSend[count] = '\0';

    while (DMA_CNDTR(DMA1, DMA_CHANNEL7))
        ;
    DMA_CCR(DMA1, DMA_CHANNEL7) &= ~DMA_CCR_EN;
    DMA_CMAR(DMA1, DMA_CHANNEL7) = (uint32_t)messageSend;
    DMA_CNDTR(DMA1, DMA_CHANNEL7) = count;
    DMA_CCR(DMA1, DMA_CHANNEL7) |= DMA_CCR_EN;
}

void setDutyF(float duty) {
    int32_t index = 0;
    uint8_t payload[5];

    payload[index++] = COMM_SET_DUTY;
    buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

    packSendPayload(payload, 5);
}

void setDuty(int32_t duty) {
    int32_t index = 0;
    uint8_t payload[5];

    duty=constrain(duty, -70000, 70000);

    payload[index++] = COMM_SET_DUTY;
    buffer_append_int32(payload, duty, &index);

    packSendPayload(payload, 5);
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
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_USART3);
    rcc_periph_clock_enable(RCC_SPI1);
}

void systick_setup() {
    systick_set_reload(89999);
    systick_interrupt_enable();
    systick_counter_enable();
}

void gpio_setup() {}

void spi_setup(){
	rcc_periph_clock_enable(RCC_SPI1);
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO4);
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO5 | GPIO7);
  	spi_reset(SPI1);
  	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_64, SPI_CR1_CPOL_CLK_TO_1_WHEN_IDLE,
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

    // sprintf(bufC, "\n");
    uPD("\nTarantaika steering start...\n");

    nvic_enable_irq(NVIC_USART1_IRQ);
    USART_BRR(USART1) = rcc_apb2_frequency / 57600 + 1;
    USART_CR1(USART1) |= USART_CR1_UE | USART_CR1_RE | USART_CR1_RXNEIE;
    nvic_enable_irq(NVIC_USART1_IRQ);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART2_TX);
    USART_BRR(USART2) = rcc_apb1_frequency / 115200 + 1;
    USART_CR1(USART2) |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
    USART_CR3(USART2) |= USART_CR3_DMAT;
    DMA_CPAR(DMA1, DMA_CHANNEL7) = (uint32_t)&USART_DR(USART2);
    DMA_CCR(DMA1, DMA_CHANNEL7) = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_EN;

    systick_setup();
    activ = 255;
    setDuty(0);

    while (1) {
        
    }
}

void sys_tick_handler() {
    if (activ) activ--;
    else control=0;

    GPIO_BRR(GPIOA) = GPIO4;
    _delay_us(5);
    GPIO_BSRR(GPIOA) = GPIO4;
    uint8_t lo=spi_xfer(SPI1, 0xAA);
    uint8_t hi=spi_xfer(SPI1, 0xAA);
    int16_t err = -(curs-(int16_t)(hi<<8 | lo));
    it=constrain(it+err, -2000, 2000);
    //sprintf(bufC, "%d %d %d\n", curs, err, it);
    //uPD(bufC);
    if(control)
        setDuty(err*80+it/2);
    else
        setDuty(0);
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
                    control=1;
                    //curs=pack.msg.angle;
                    curs=map(pack.msg.angle, -10000, 10000, 200, 800);
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