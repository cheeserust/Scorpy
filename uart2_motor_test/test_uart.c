#include "stm32f4xx.h"
#include <stdint.h>

#define SYSCLK_HZ 96000000
#define UART_BAUD 115200
#define UART_PCLK_HZ (SYSCLK_HZ / 2)

#define AXIS_COUNT 4
#define DEFAULT_STEP_PULSES 400
#define MAX_STEP_PULSES 20000
#define STEP_HIGH_US 5
#define STEP_LOW_US 995

#define TMC_MOSI_PORT GPIOB
#define TMC_MOSI_PIN  1
#define TMC_MISO_PORT GPIOB
#define TMC_MISO_PIN  0
#define TMC_CLK_PORT  GPIOB
#define TMC_CLK_PIN   5
#define MOTOR_EN_PORT GPIOB
#define MOTOR_EN_PIN  3

#define TMC_REG_GCONF       0x00
#define TMC_REG_GSTAT       0x01
#define TMC_REG_IOIN        0x04
#define TMC_REG_IHOLD_IRUN  0x10
#define TMC_REG_CHOPCONF    0x6C
#define TMC_REG_DRV_STATUS  0x6F
#define TMC_REG_PWMCONF     0x70

typedef struct {
    GPIO_TypeDef *step_port;
    uint8_t step_pin;
    GPIO_TypeDef *dir_port;
    uint8_t dir_pin;
    GPIO_TypeDef *cs_port;
    uint8_t cs_pin;
} AxisPins;

static const AxisPins axis_pins[AXIS_COUNT] = {
    {GPIOA, 1,  GPIOA, 0,  GPIOB, 12},
    {GPIOC, 15, GPIOC, 14, GPIOA, 4},
    {GPIOB, 9,  GPIOB, 8,  GPIOB, 10},
    {GPIOB, 7,  GPIOB, 6,  GPIOB, 2},
};

static uint8_t motors_enabled;

static void clock_init_96mhz(void)
{
    RCC->CR |= (1 << 0);
    while ((RCC->CR & (1 << 1)) == 0) {}

    RCC->APB1ENR |= (1 << 28);
    (void)RCC->APB1ENR;
    PWR->CR |= (3 << 14);

    FLASH->ACR = (1 << 10) | (1 << 9) | (1 << 8) | (3 << 0);

    RCC->CR &= ~(1 << 24);
    while (RCC->CR & (1 << 25)) {}

    RCC->PLLCFGR = (8 << 24) |
                   (0 << 22) |
                   (1 << 16) |
                   (192 << 6) |
                   (8 << 0);

    RCC->CFGR = (0 << 13) |
                (4 << 10) |
                (0 << 4);

    RCC->CR |= (1 << 24);
    while ((RCC->CR & (1 << 25)) == 0) {}

    RCC->CFGR &= ~(0x3 << 0);
    RCC->CFGR |= (0x2 << 0);
    while (((RCC->CFGR >> 2) & 0x3) != 0x2) {}

    SystemCoreClock = SYSCLK_HZ;
}

static void dwt_delay_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SYSCLK_HZ / 1000000);
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {}
}

static void gpio_set(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1 << pin);
}

static void gpio_clear(GPIO_TypeDef *port, uint8_t pin)
{
    port->BSRR = (1 << (pin + 16));
}

static void gpio_output(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));
    port->MODER |=  (1 << (pin * 2));
    port->OTYPER &= ~(1 << pin);
    port->OSPEEDR &= ~(3 << (pin * 2));
    port->OSPEEDR |=  (2 << (pin * 2));
    port->PUPDR &= ~(3 << (pin * 2));
}

static void gpio_input_pullup(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));
    port->PUPDR &= ~(3 << (pin * 2));
    port->PUPDR |=  (1 << (pin * 2));
}

static void gpio_af7(GPIO_TypeDef *port, uint8_t pin)
{
    port->MODER &= ~(3 << (pin * 2));
    port->MODER |=  (2 << (pin * 2));
    port->OTYPER &= ~(1 << pin);
    port->OSPEEDR &= ~(3 << (pin * 2));
    port->OSPEEDR |=  (3 << (pin * 2));
    port->PUPDR &= ~(3 << (pin * 2));
    port->PUPDR |=  (1 << (pin * 2));
    port->AFR[pin / 8] &= ~(0xF << ((pin % 8) * 4));
    port->AFR[pin / 8] |=  (0x7 << ((pin % 8) * 4));
}

static void uart2_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;

    gpio_af7(GPIOA, 2);  // PA2 = USART2_TX
    gpio_af7(GPIOA, 3);  // PA3 = USART2_RX

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->BRR = (UART_PCLK_HZ + (UART_BAUD / 2)) / UART_BAUD;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

static uint8_t uart2_readable(void)
{
    return (USART2->SR & USART_SR_RXNE) ? 1 : 0;
}

static char uart2_getc(void)
{
    while (!uart2_readable()) {}
    return (char)(USART2->DR & 0xFF);
}

static void uart2_putc(char ch)
{
    while ((USART2->SR & USART_SR_TXE) == 0) {}
    USART2->DR = (uint8_t)ch;
}

static void uart2_puts(const char *str)
{
    while (*str) {
        uart2_putc(*str++);
    }
}

static void uart2_put_u32(uint32_t value)
{
    char buf[10];
    uint8_t len = 0;

    if (value == 0) {
        uart2_putc('0');
        return;
    }

    while (value && len < sizeof(buf)) {
        buf[len++] = (char)('0' + (value % 10));
        value /= 10;
    }
    while (len) {
        uart2_putc(buf[--len]);
    }
}

static void uart2_put_hex8(uint8_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    uart2_puts("0x");
    uart2_putc(hex[(value >> 4) & 0x0F]);
    uart2_putc(hex[value & 0x0F]);
}

static void uart2_put_hex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";

    uart2_puts("0x");
    for (int8_t shift = 28; shift >= 0; shift -= 4) {
        uart2_putc(hex[(value >> (uint8_t)shift) & 0x0F]);
    }
}

static void motor_gpio_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        gpio_clear(axis_pins[i].step_port, axis_pins[i].step_pin);
        gpio_set(axis_pins[i].cs_port, axis_pins[i].cs_pin);
        gpio_output(axis_pins[i].step_port, axis_pins[i].step_pin);
        gpio_output(axis_pins[i].dir_port, axis_pins[i].dir_pin);
        gpio_output(axis_pins[i].cs_port, axis_pins[i].cs_pin);
    }

    gpio_output(TMC_MOSI_PORT, TMC_MOSI_PIN);
    gpio_input_pullup(TMC_MISO_PORT, TMC_MISO_PIN);
    gpio_output(TMC_CLK_PORT, TMC_CLK_PIN);
    gpio_set(TMC_CLK_PORT, TMC_CLK_PIN);

    gpio_set(MOTOR_EN_PORT, MOTOR_EN_PIN);  // MOTOR_EN active-low, boot disabled
    gpio_output(MOTOR_EN_PORT, MOTOR_EN_PIN);
    motors_enabled = 0;
}

static void motor_enable(void)
{
    gpio_clear(MOTOR_EN_PORT, MOTOR_EN_PIN);
    motors_enabled = 1;
}

static void motor_disable(void)
{
    gpio_set(MOTOR_EN_PORT, MOTOR_EN_PIN);
    motors_enabled = 0;
}

static void tmc_cs(uint8_t axis, uint8_t selected)
{
    if (selected) {
        gpio_clear(axis_pins[axis].cs_port, axis_pins[axis].cs_pin);
    } else {
        gpio_set(axis_pins[axis].cs_port, axis_pins[axis].cs_pin);
    }
}

static uint8_t tmc_transfer_byte(uint8_t tx)
{
    uint8_t rx = 0;

    for (int8_t bit = 7; bit >= 0; bit--) {
        gpio_clear(TMC_CLK_PORT, TMC_CLK_PIN);
        if (tx & (uint8_t)(1 << bit)) {
            gpio_set(TMC_MOSI_PORT, TMC_MOSI_PIN);
        } else {
            gpio_clear(TMC_MOSI_PORT, TMC_MOSI_PIN);
        }
        delay_us(2);
        gpio_set(TMC_CLK_PORT, TMC_CLK_PIN);
        delay_us(2);
        if (TMC_MISO_PORT->IDR & (1 << TMC_MISO_PIN)) {
            rx |= (uint8_t)(1 << bit);
        }
    }

    return rx;
}

static uint8_t tmc_transfer(uint8_t axis, uint8_t addr, uint32_t tx_data, uint32_t *rx_data)
{
    uint8_t status;
    uint32_t rx = 0;

    if (axis >= AXIS_COUNT) return 0;

    tmc_cs(axis, 1);
    delay_us(1);
    status = tmc_transfer_byte(addr);
    rx |= (uint32_t)tmc_transfer_byte((uint8_t)(tx_data >> 24)) << 24;
    rx |= (uint32_t)tmc_transfer_byte((uint8_t)(tx_data >> 16)) << 16;
    rx |= (uint32_t)tmc_transfer_byte((uint8_t)(tx_data >> 8)) << 8;
    rx |= (uint32_t)tmc_transfer_byte((uint8_t)tx_data);
    delay_us(1);
    tmc_cs(axis, 0);
    delay_us(10);

    if (rx_data != 0) *rx_data = rx;
    return status;
}

static uint8_t tmc_read(uint8_t axis, uint8_t reg, uint32_t *value)
{
    (void)tmc_transfer(axis, (uint8_t)(reg & 0x7F), 0, 0);
    return tmc_transfer(axis, (uint8_t)(reg & 0x7F), 0, value);
}

static void print_tmc_reg(uint8_t axis, const char *name, uint8_t reg)
{
    uint32_t value;
    uint8_t status = tmc_read(axis, reg, &value);

    uart2_puts("  ");
    uart2_puts(name);
    uart2_putc('=');
    uart2_put_hex32(value);
    uart2_puts(" spi_status=");
    uart2_put_hex8(status);
    uart2_puts("\r\n");
}

static void print_tmc_status(void)
{
    for (uint8_t axis = 0; axis < AXIS_COUNT; axis++) {
        uart2_puts("TMC axis");
        uart2_put_u32((uint32_t)axis + 1);
        uart2_puts("\r\n");
        print_tmc_reg(axis, "GSTAT", TMC_REG_GSTAT);
        print_tmc_reg(axis, "IOIN", TMC_REG_IOIN);
        print_tmc_reg(axis, "DRV_STATUS", TMC_REG_DRV_STATUS);
        print_tmc_reg(axis, "CHOPCONF", TMC_REG_CHOPCONF);
    }
    uart2_puts("> ");
}

static void tmc_write(uint8_t axis, uint8_t addr, uint32_t data)
{
    uint8_t tx[5];

    tx[0] = addr | 0x80;
    tx[1] = (uint8_t)(data >> 24);
    tx[2] = (uint8_t)(data >> 16);
    tx[3] = (uint8_t)(data >> 8);
    tx[4] = (uint8_t)data;

    tmc_cs(axis, 1);
    for (uint8_t byte = 0; byte < 5; byte++) {
        for (int8_t bit = 7; bit >= 0; bit--) {
            gpio_clear(TMC_CLK_PORT, TMC_CLK_PIN);
            if (tx[byte] & (uint8_t)(1 << bit)) {
                gpio_set(TMC_MOSI_PORT, TMC_MOSI_PIN);
            } else {
                gpio_clear(TMC_MOSI_PORT, TMC_MOSI_PIN);
            }
            delay_us(2);
            gpio_set(TMC_CLK_PORT, TMC_CLK_PIN);
            delay_us(2);
        }
    }
    tmc_cs(axis, 0);
    delay_us(10);
}

static void tmc_init_all(void)
{
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if (i == 3) continue;
        
        tmc_write(i, TMC_REG_GCONF, 0x00000000);
        tmc_write(i, TMC_REG_IHOLD_IRUN, 0x00061004);
        tmc_write(i, TMC_REG_CHOPCONF, 0x040100C3);
        tmc_write(i, TMC_REG_PWMCONF, 0x00050480);
    }
}

static void step_axis_once(uint8_t axis)
{
    gpio_set(axis_pins[axis].step_port, axis_pins[axis].step_pin);
    delay_us(STEP_HIGH_US);
    gpio_clear(axis_pins[axis].step_port, axis_pins[axis].step_pin);
    delay_us(STEP_LOW_US);
}

static uint8_t stop_requested(void)
{
    if (!uart2_readable()) return 0;

    char ch = (char)(USART2->DR & 0xFF);
    if (ch == 's' || ch == 'S') {
        motor_disable();
        uart2_puts("\r\nSTOP\r\n> ");
        return 1;
    }
    return 0;
}

static void move_axis(uint8_t axis, uint8_t dir, uint32_t pulses)
{
    if (axis >= AXIS_COUNT) return;

    if (!motors_enabled) {
        uart2_puts("ERR: send E first\r\n");
        return;
    }

    if (pulses == 0 || pulses > MAX_STEP_PULSES) {
        uart2_puts("ERR: pulse range 1..");
        uart2_put_u32(MAX_STEP_PULSES);
        uart2_puts("\r\n");
        return;
    }

    if (dir) {
        gpio_set(axis_pins[axis].dir_port, axis_pins[axis].dir_pin);
    } else {
        gpio_clear(axis_pins[axis].dir_port, axis_pins[axis].dir_pin);
    }

    uart2_puts("M");
    uart2_putc((char)('1' + axis));
    uart2_putc(dir ? '+' : '-');
    uart2_puts(" ");
    uart2_put_u32(pulses);
    uart2_puts(" pulses\r\n");

    for (uint32_t i = 0; i < pulses; i++) {
        if (stop_requested()) return;
        step_axis_once(axis);
    }

    uart2_puts("OK\r\n");
}

static uint8_t parse_u32(const char *str, uint32_t *value)
{
    uint32_t result = 0;
    uint8_t found = 0;

    while (*str >= '0' && *str <= '9') {
        found = 1;
        result = (result * 10) + (uint32_t)(*str - '0');
        str++;
    }

    if (*str != '\0') return 0;
    if (!found) result = DEFAULT_STEP_PULSES;

    *value = result;
    return 1;
}

static void print_help(void)
{
    uart2_puts("\r\nUART2 motor test, PA2 TX / PA3 RX, 115200 8N1\r\n");
    uart2_puts("Wire: USB-TTL TX->PA3, RX->PA2, GND->GND, 3.3V level\r\n");
    uart2_puts("TMC SPI: MISO PB0, CLK PB5, MOSI PB1, MOTEN PB3\r\n");
    uart2_puts("CS: axis1 PB12, axis2 PA4, axis3 PB10, axis4 PB2\r\n");
    uart2_puts("Commands:\r\n");
    uart2_puts("  E       enable motor drivers\r\n");
    uart2_puts("  D       disable motor drivers\r\n");
    uart2_puts("  S       stop/disable during a move\r\n");
    uart2_puts("  tmc     read TMC5160 registers\r\n");
    uart2_puts("  1+      arm axis 2 forward, default 400 pulses\r\n");
    uart2_puts("  j1+1000 same as 1+1000\r\n");
    uart2_puts("  1-      arm axis 2 reverse, default 400 pulses\r\n");
    uart2_puts("  2+ 2-   arm axis 3 forward/reverse\r\n");
    uart2_puts("  3+ 3-   arm axis 4 forward/reverse\r\n");
    uart2_puts("  4+ 4-   arm axis 5 forward/reverse\r\n");
    uart2_puts("  1+1600  arm axis 2 forward, 1600 pulses\r\n");
    uart2_puts("> ");
}

static void run_command(char *line)
{
    if (line[0] == '\0') {
        uart2_puts("> ");
        return;
    }

    if ((line[0] == 'h' || line[0] == 'H' || line[0] == '?') && line[1] == '\0') {
        print_help();
        return;
    }

    if ((line[0] == 'e' || line[0] == 'E') && line[1] == '\0') {
        motor_enable();
        uart2_puts("ENABLED\r\n> ");
        return;
    }

    if ((line[0] == 'd' || line[0] == 'D') && line[1] == '\0') {
        motor_disable();
        uart2_puts("DISABLED\r\n> ");
        return;
    }

    if ((line[0] == 's' || line[0] == 'S') && line[1] == '\0') {
        motor_disable();
        uart2_puts("STOP\r\n> ");
        return;
    }

    if ((line[0] == 't' || line[0] == 'T') &&
        (line[1] == 'm' || line[1] == 'M') &&
        (line[2] == 'c' || line[2] == 'C') &&
        line[3] == '\0') {
        print_tmc_status();
        return;
    }

    if ((line[0] == 'j' || line[0] == 'J') &&
        line[1] >= '1' && line[1] <= '4' &&
        (line[2] == '+' || line[2] == '-')) {
        uint32_t pulses;
        if (!parse_u32(&line[3], &pulses)) {
            uart2_puts("ERR: bad pulse count\r\n> ");
            return;
        }
        move_axis((uint8_t)(line[1] - '1'), (uint8_t)(line[2] == '+'), pulses);
        uart2_puts("> ");
        return;
    }

    if (line[0] >= '1' && line[0] <= '4' && (line[1] == '+' || line[1] == '-')) {
        uint32_t pulses;
        if (!parse_u32(&line[2], &pulses)) {
            uart2_puts("ERR: bad pulse count\r\n> ");
            return;
        }
        move_axis((uint8_t)(line[0] - '1'), (uint8_t)(line[1] == '+'), pulses);
        uart2_puts("> ");
        return;
    }

    uart2_puts("ERR: unknown command, send ?\r\n> ");
}

static void terminal_loop(void)
{
    char line[24];
    uint8_t len = 0;

    print_help();

    while (1) {
        char ch = uart2_getc();

        if (ch == '\r' || ch == '\n') {
            uart2_puts("\r\n");
            line[len] = '\0';
            run_command(line);
            len = 0;
            continue;
        }

        if (ch == '\b' || ch == 0x7F) {
            if (len > 0) {
                len--;
                uart2_puts("\b \b");
            }
            continue;
        }

        if (len < (sizeof(line) - 1)) {
            line[len++] = ch;
            uart2_putc(ch);
        }
    }
}

int main(void)
{
    __disable_irq();
    clock_init_96mhz();
    dwt_delay_init();
    uart2_init();
    motor_gpio_init();
    tmc_init_all();
    __enable_irq();

    terminal_loop();
}
