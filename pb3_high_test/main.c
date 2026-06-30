#include "stm32f4xx.h"

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    GPIOB->ODR |= (1 << 3);

    GPIOB->MODER &= ~(3 << (3 * 2));
    GPIOB->MODER |=  (1 << (3 * 2));
    GPIOB->OTYPER &= ~(1 << 3);
    GPIOB->OSPEEDR &= ~(3 << (3 * 2));
    GPIOB->OSPEEDR |=  (2 << (3 * 2));
    GPIOB->PUPDR &= ~(3 << (3 * 2));

    while (1) {
        GPIOB->BSRR = (1 << 3);
    }
}
