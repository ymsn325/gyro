#include <l3gd20.h>
#include <stdio.h>
#include <stm32l476xx.h>

#define SPI_READ_CMD 0x80
#define SPI_INC_CMD 0x40

uint8_t txdata[256];

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
  GPIOD->OSPEEDR |= GPIO_OSPEEDR_OSPEED1 | GPIO_OSPEEDR_OSPEED3 |
                    GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED7;
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
  uint8_t data, dummy_data;
  SPI2->CR2 &= ~SPI_CR2_DS;
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
  while (SPI2->SR & SPI_SR_FRLVL_Msk) {
    dummy_data = SPI2->DR;
  }

  SPI2->CR1 &= ~SPI_CR1_SPE;
  return data;
}

int16_t spi2_read16bit(uint8_t addr) {
  int16_t data;
  SPI2->CR2 &= ~SPI_CR2_DS;
  SPI2->CR2 |= (16 - 1) << SPI_CR2_DS_Pos;
  SPI2->CR2 &= ~SPI_CR2_FRXTH;
  SPI2->CR1 |= SPI_CR1_SPE;
  GPIOD->BSRR = GPIO_BSRR_BR7;  // SPEがアサート時にリセットすること
  while (!(SPI2->SR & SPI_SR_TXE_Msk))
    ;
  uint16_t send_data = (SPI_READ_CMD | SPI_INC_CMD | addr) << 8;
  SPI2->DR = send_data;
  while (!(SPI2->SR & SPI_SR_RXNE_Msk))
    ;
  data = SPI2->DR & 0xffff;
  GPIOD->BSRR = GPIO_BSRR_BS7;
  while (SPI2->SR & SPI_SR_FTLVL_Msk)
    ;
  while (SPI2->SR & SPI_SR_BSY_Msk)
    ;
  SPI2->CR1 &= ~SPI_CR1_SPE;
  return data;
}

void spi2_write8bit(uint8_t addr, uint8_t dat) {
  uint16_t dummy_dat;
  SPI2->CR2 &= ~SPI_CR2_DS;
  SPI2->CR2 |= (16 - 1) << SPI_CR2_DS_Pos;
  // SPI2->CR2 &= SPI_CR2_FRXTH;
  SPI2->CR1 |= SPI_CR1_SPE;
  GPIOD->BSRR = GPIO_BSRR_BR7;  // SPEがアサート時にリセットすること
  while (!(SPI2->SR & SPI_SR_TXE_Msk))
    ;
  SPI2->DR = (addr << 8) | dat;
  while (!(SPI2->SR & SPI_SR_RXNE_Msk))
    ;
  while (SPI2->SR & SPI_SR_FTLVL_Msk)
    ;
  while (SPI2->SR & SPI_SR_BSY_Msk)
    ;
  dummy_dat = SPI2->DR;
  GPIOD->BSRR = GPIO_BSRR_BS7;
  SPI2->CR1 &= ~SPI_CR1_SPE;
}

typedef struct gyro_data {
  float val_x;
  float val_y;
  float val_z;
} GyroData;

void gyro_read(GyroData *gyro_data) {
  uint8_t val_x_l, val_x_h, val_y_l, val_y_h, val_z_l, val_z_h;
  int16_t val_x_int, val_y_int, val_z_int;
  val_x_l = spi2_read8bit(L3GD20_OUT_X_L_ADDR);
  val_x_h = spi2_read8bit(L3GD20_OUT_X_H_ADDR);
  val_y_l = spi2_read8bit(L3GD20_OUT_Y_L_ADDR);
  val_y_h = spi2_read8bit(L3GD20_OUT_Y_H_ADDR);
  val_z_l = spi2_read8bit(L3GD20_OUT_Z_L_ADDR);
  val_z_h = spi2_read8bit(L3GD20_OUT_Z_H_ADDR);
  val_x_int = (val_x_h << 8) | val_x_l;
  val_y_int = (val_y_h << 8) | val_y_l;
  val_z_int = (val_z_h << 8) | val_z_l;
  gyro_data->val_x = val_x_int * L3GD20_SENSITIVITY_250DPS / 1000.0f;
  gyro_data->val_y = val_y_int * L3GD20_SENSITIVITY_250DPS / 1000.0f;
  gyro_data->val_z = val_z_int * L3GD20_SENSITIVITY_250DPS / 1000.0f;
}

void usart2_init(void) {
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIODEN;
  GPIOD->MODER &= ~(GPIO_MODER_MODE5 | GPIO_MODER_MODE6);
  GPIOD->MODER |= (GPIO_MODER_MODE5_1 | GPIO_MODER_MODE6_1);
  GPIOD->AFR[0] |= (7 << GPIO_AFRL_AFSEL5_Pos) | (7 << GPIO_AFRL_AFSEL6_Pos);
  RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
  USART2->BRR = SystemCoreClock / 115200;
  USART2->CR1 |= USART_CR1_UE;
  USART2->CR3 |= USART_CR3_DMAT;
  USART2->CR1 |= USART_CR1_TE;
}

void usart_dma_init(void) {
  RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
  DMA1_CSELR->CSELR |= 2 << DMA_CSELR_C7S_Pos;
  DMA1_Channel7->CPAR = (uint32_t)&USART2->TDR;
  DMA1_Channel7->CMAR = (uint32_t)txdata;
  DMA1_Channel7->CNDTR = 255;
  DMA1_Channel7->CCR |= (DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_MINC);
  DMA1_Channel7->CCR |= DMA_CCR_EN;
}

GyroData gyro_data;
uint8_t rxdata;

int main(void) {
  led_init();
  spi2_init();
  usart_dma_init();
  usart2_init();
  spi2_write8bit(L3GD20_CTRL_REG1_ADDR, L3GD20_MODE_ACTIVE | L3GD20_X_ENABLE |
                                            L3GD20_Y_ENABLE | L3GD20_Z_ENABLE);
  rxdata = spi2_read8bit(L3GD20_CTRL_REG1_ADDR);
  while (1) {
    gyro_read(&gyro_data);
    sprintf(txdata, "%0f, %0f, %0f\r\n", gyro_data.val_x, gyro_data.val_y,
            gyro_data.val_z);
    led_off();
    for (uint32_t i = 0; i < 10000; i++)
      ;
    led_on();
    for (uint32_t i = 0; i < 10000; i++)
      ;
  }
  return 0;
}