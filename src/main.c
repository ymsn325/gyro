#include <stm32l476xx.h>
#define SPI_READ_CMD 0x80

uint8_t rxdata;

void led_init(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
  GPIOE->MODER &= ~GPIO_MODER_MODE8;
  GPIOE->MODER |= GPIO_MODER_MODE8_0;
}

void led_on(void) { GPIOE->BSRR = GPIO_BSRR_BS8; }

void led_off(void) { GPIOE->BSRR = GPIO_BSRR_BR8; }

void spi2_init(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
  GPIOD->MODER &= ~(GPIO_MODER_MODE1 | GPIO_MODER_MODE3 | GPIO_MODER_MODE4 |
                    GPIO_MODER_MODE7);
  GPIOD->MODER |= GPIO_MODER_MODE1_1 | GPIO_MODER_MODE3_1 | GPIO_MODER_MODE4_1 |
                  GPIO_MODER_MODE7_0;
  GPIOD->OSPEEDR |=
      GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED3 | GPIO_OSPEEDR_OSPEED4;
  GPIOD->AFR[0] |= (5 << GPIO_AFRL_AFSEL1_Pos) | (5 << GPIO_AFRL_AFSEL3_Pos) |
                   (5 << GPIO_AFRL_AFSEL4_Pos);
  GPIOD->BSRR = GPIO_BSRR_BS7;
  RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
  SPI2->CR1 |= SPI_CR1_CPOL | SPI_CR1_CPHA;
  SPI2->CR1 |= SPI_CR1_SSM;
  SPI2->CR1 |= SPI_CR1_SSI;
  SPI2->CR1 |= SPI_CR1_MSTR;
  SPI2->CR2 |= SPI_CR2_SSOE;
}

uint8_t spi2_read8bit(uint8_t addr) {
  uint8_t data;
  SPI2->CR2 |= (8 - 1) << SPI_CR2_DS_Pos;
  SPI2->CR2 |= SPI_CR2_FRXTH;
  SPI2->CR1 |= SPI_CR1_SPE;
  GPIOD->BSRR = GPIO_BSRR_BR7;  // SPEがアサート時にリセットすること
  while (!(SPI2->SR & SPI_SR_TXE_Msk))
    ;
  SPI2->DR = SPI_READ_CMD | addr;
  while (!(SPI2->SR & SPI_SR_RXNE_Msk))
    ;
  data = ((SPI2->DR) >> 8) & 0xff;
  GPIOD->BSRR = GPIO_BSRR_BS7;
  while (SPI2->SR & SPI_SR_FTLVL_Msk)
    ;
  while (SPI2->SR & SPI_SR_BSY_Msk)
    ;
  SPI2->CR1 &= SPI_CR1_SPE;
  return data;
}

int main(void) {
  led_init();
  spi2_init();
  while (1) {
    led_off();
    rxdata = spi2_read8bit(0x0f);
    for (uint32_t i = 0; i < 10000; i++)
      ;
    led_on();
    for (uint32_t i = 0; i < 10000; i++)
      ;
  }
  return 0;
}