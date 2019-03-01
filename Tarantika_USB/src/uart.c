#include "uart.h"

void usart_setup(){
    //AFIO_MAPR |= AFIO_MAPR_USART1_REMAP;
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
	usart_set_baudrate(USART1, 115200);
	usart_set_mode(USART1, USART_MODE_TX_RX);
	usart_enable_tx_dma(USART1);
	usart_enable(USART1);
	DMA_CPAR(DMA1, DMA_CHANNEL4) = (uint32_t)&USART_DR(USART1);
	DMA_CCR(DMA1, DMA_CHANNEL4) = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_EN;	
}

void uPD(char *data){
	while(DMA_CNDTR(DMA1, DMA_CHANNEL4));
	DMA_CCR(DMA1, DMA_CHANNEL4) &= ~DMA_CCR_EN;
	DMA_CMAR(DMA1, DMA_CHANNEL4) = (uint32_t)data;
	DMA_CNDTR(DMA1, DMA_CHANNEL4) = strl(data);
	DMA_CCR(DMA1, DMA_CHANNEL4) |= DMA_CCR_EN;
}

uint16_t strl(char *str){
	uint16_t l=0;
    while (*str++ != 0)l++;
	return l;
}